#!/usr/bin/env python3

import threading
import queue
import time
import json
from datetime import datetime
from typing import List, Optional

from robot_manager import Task, TaskType, TaskStatus, Priority, RobotState
from intelligent_scheduler import IntelligentTaskScheduler
from command_parser import CommandParser
from robot_arm_interface import RobotArmInterface
# 새로 만든 파일들을 import 합니다.
from task_executor import TaskExecutor
from confirmation_manager import ConfirmationManager

class ROSATaskProcessor:
    """(리팩토링 최종본) 작업 처리 및 지능형 스케줄링의 메인 컨트롤러 (지휘자)"""

    def __init__(self, robot_manager):
        self.robot_manager = robot_manager
        self.scheduler = IntelligentTaskScheduler(robot_manager)
        self.command_parser = CommandParser(robot_manager, self)
        self.arm_interface = RobotArmInterface(robot_manager, self)
        
        # 역할 분리! 실행 및 확인 담당 매니저 생성
        self.confirmation_manager = ConfirmationManager(self)
        self.executor = TaskExecutor(self)
        
        self.global_task_queue: List[Task] = []
        self.completed_tasks: List[Task] = []
        self.command_queue = queue.Queue()
        self._task_counter = 0

        # 월급루팡 방지 시스템
        self.robot_task_start_times = {}
        self.stuck_timeout = 60.0

        # 픽업대 해제 타이머 시스템 (TaskProcessor에서 관리)
        self.pickup_release_timer = None
        self.pickup_release_delay = 0.5  # 0.5초 후 다음 로봇 호출 (빠른 반응)
        self.pickup_area_released_robots = set()  # 이미 해제 처리된 로봇들 (중복 처리 방지)

        self.status_only = False
        self.input_only = False
        self.gui = None

        self.robot_manager.create_timer(0.5, self.process_commands)
        self.robot_manager.create_timer(1.0, self.task_processor_loop)
        self.robot_manager.create_timer(10.0, self.optimize_task_queue)
        self.robot_manager.create_timer(1.0, self.confirmation_manager.process_confirmation_queue)
        self.robot_manager.create_timer(30.0, self.check_stuck_robots)
        # 픽업대 상태 모니터링은 task_processor_loop에서 처리됨
        self.robot_manager.create_timer(300.0, self.cleanup_expired_data) # 5분마다 메모리 정리
        
        # 글로벌 카메라 긴급 알림 구독 (주석 처리)
        # self._setup_emergency_subscribers()  # 글로벌 카메라 연동 시 주석 해제
        
        self.log_message("📋 ROSA 작업 처리기 시작! (리팩토링 버전)", is_command=False)

    def get_current_time(self):
        return time.time()

    def set_gui(self, gui):
        self.gui = gui

    def log_message(self, message: str, level: str = "INFO", is_command: bool = False):
        log_type = "COMMAND" if is_command else "SYSTEM"
        if self.status_only:
            if is_command: print(f"[{datetime.now().strftime('%H:%M:%S')}] {message}")
            elif self.gui: self.gui.add_log(message, level)
        elif self.input_only: self.robot_manager.publish_log_message(message, level, log_type)
        else: print(f"[{datetime.now().strftime('%H:%M:%S')}] {message}")

    def command_log(self, message: str, level: str = "INFO"):
        self.log_message(message, level, is_command=True)

    def system_log(self, message: str, level: str = "INFO"):
        self.log_message(message, level, is_command=True)

    def start_input_thread(self):
        threading.Thread(target=self._input_thread_func, daemon=True).start()

    def _input_thread_func(self):
        prompt = "🤖 [ROSA] 명령 > "
        while True:
            try:
                cmd = input(prompt).strip()
                if cmd: self.command_queue.put(cmd)
            except (EOFError, KeyboardInterrupt):
                self.log_message("👋 종료")
                import rclpy
                rclpy.shutdown()
                break

    def process_commands(self):
        if self.status_only: return
        while not self.command_queue.empty():
            cmd = self.command_queue.get().strip()
            if not cmd: continue
            if cmd.lower() in ['quit', 'exit', '종료']:
                self.log_message("👋 시스템 종료", is_command=True)
                import rclpy
                rclpy.shutdown()
                return
            self.command_parser.parse_intelligent_command(cmd)

    def add_task_to_queue(self, task: Task, robot_name: Optional[str], is_return_command: bool, is_charge_station_command: bool):
        if robot_name:
            robot = self.robot_manager.robots[robot_name]
            if is_return_command or is_charge_station_command:
                self.pending_return_task = task
            if self.handle_preemption(robot, is_return_command, is_charge_station_command):
                return
            task.robot_name = robot_name
        
        self.global_task_queue.append(task)
        self.publish_task_queue_update()
        self.log_message(f"✅ 작업 생성 완료 (ID: {task.id})", "SUCCESS", is_command=True)

        if task.robot_name:
            self._start_task_execution(task.robot_name, task)
        else:
            self.try_assign_tasks_to_available_robots()
    
    def handle_preemption(self, robot, is_return_command, is_charge_station_command) -> bool:
        if robot.is_permanently_stationed:
            self.log_message(f"🔄 {robot.name} 영구 대기 해제", is_command=True)
            robot.is_permanently_stationed = False

        if robot.current_task and not (is_return_command or is_charge_station_command):
            self.log_message(f"❌ {robot.name}이 다른 업무 진행 중", "ERROR", is_command=True)
            return True

        if robot.current_task and (is_return_command or is_charge_station_command):
            self.confirmation_manager.queue_confirmation_request(
                robot_name=robot.name, prompt="add_to_queue",
                interrupted_task=robot.current_task
            )
            robot.current_task = None
            return True
        return False

    def task_processor_loop(self):
        for robot_name, robot in self.robot_manager.robots.items():
            if robot.state == RobotState.LOW_BATTERY:
                if robot.current_task: self.suspend_robot_task(robot_name)
                continue
            if not robot.current_task: self.assign_next_task(robot_name)
            if robot.current_task: self.process_robot_task(robot_name)
        self.process_location_queues()

    def assign_next_task(self, robot_name: str):
        robot = self.robot_manager.robots[robot_name]
        if robot.is_permanently_stationed or not robot.current_pose or robot.current_task: return

        for i, task in enumerate(self.global_task_queue):
            if task.robot_name == robot_name:
                robot.current_task = self.global_task_queue.pop(i)
                self.publish_task_queue_update()
                self._start_task_execution(robot_name, robot.current_task)
                return

        if robot.state in [RobotState.IDLE, RobotState.RETURNING, RobotState.CHARGING]:
            self.try_assign_single_robot(robot_name)

    def try_assign_tasks_to_available_robots(self):
        unassigned_tasks = [t for t in self.global_task_queue if not t.robot_name]
        if not unassigned_tasks: return
        unassigned_tasks.sort(key=lambda t: self.scheduler.calculate_priority_score(t), reverse=True)
        
        for task in unassigned_tasks:
            robot_name = self.scheduler.find_optimal_robot(task, self.log_message)
            if robot_name:
                task.robot_name = robot_name
                self.assign_next_task(robot_name)
                break

    def try_assign_single_robot(self, robot_name: str):
        robot = self.robot_manager.robots[robot_name]
        if robot.current_task: return
        unassigned_tasks = [t for t in self.global_task_queue if not t.robot_name]
        if not unassigned_tasks: return
        unassigned_tasks.sort(key=lambda t: self.scheduler.calculate_priority_score(t), reverse=True)
        unassigned_tasks[0].robot_name = robot_name
        self.assign_next_task(robot_name)

    def process_robot_task(self, robot_name: str):
        robot = self.robot_manager.robots[robot_name]
        task = robot.current_task
        if not task or robot.state == RobotState.WAITING_FOR_CONFIRMATION: return
        
        # 실제 작업 수행은 executor에게 위임
        self.executor.process_robot_task(robot, task)
    
    def _start_task_execution(self, robot_name: str, task):
        if not task: return
        robot = self.robot_manager.robots[robot_name]
        self.robot_task_start_times[robot_name] = self.get_current_time()
        robot.state = RobotState.RETURNING if task.type == TaskType.RETURN_TO_CHARGE else RobotState.WORKING
        
        self.publish_task_queue_update()
        if task.type == TaskType.DELIVERY and robot.task_step == 0:
            robot.task_step = 1
        
        robot.last_goal_time = 0

    def _plan_path_sequence(self, robot_name: str, destination_name: str) -> Optional[List[str]]:
        robot = self.robot_manager.robots.get(robot_name)
        if not robot or not robot.current_pose: return None
        start_y = robot.current_pose.position.y
        end_coords = self.robot_manager.location_manager.get_location_coordinates(destination_name)
        if not end_coords: return None
        end_y = end_coords[1]
        
        if start_y < end_y: return ["highway_up"]
        else: return ["highway_down"]
    
    def _execute_pending_return_task(self):
        if self.pending_return_task:
            robot = self.robot_manager.robots[self.pending_return_robot]
            robot.current_task = self.pending_return_task
            self._start_task_execution(robot.name, robot.current_task)

    def suspend_robot_task(self, robot_name: str):
        robot = self.robot_manager.robots[robot_name]
        if not robot.current_task: return
        task = robot.current_task
        task.robot_name = ""
        self.global_task_queue.insert(0, task)
        robot.current_task = None
        self.robot_manager.location_manager.leave_queue('픽업대', robot_name)
        
    def check_stuck_robots(self):
        current_time = self.get_current_time()
        for robot_name, start_time in list(self.robot_task_start_times.items()):
            robot = self.robot_manager.robots[robot_name]
            if robot.current_task and not robot.is_moving and (current_time - start_time) > self.stuck_timeout:
                 if robot.state != RobotState.WAITING_FOR_CONFIRMATION:
                    self.command_log(f"🚨 {robot_name} 월급루팡 감지! 강제 복귀!")
                    self.confirmation_manager.fail_robot_task(robot_name, "Stuck timeout")

    def _cleanup_robot_tracking_data(self, robot_name: str):
        if robot_name in self.robot_task_start_times: 
            del self.robot_task_start_times[robot_name]
        # confirmation_sent_times는 이제 confirmation_manager에서 관리됨

    def process_location_queues(self):
        if not self.robot_manager.location_manager.is_location_busy('픽업대'):
            next_robot = self.robot_manager.location_manager.get_next_in_queue('픽업대')
            if next_robot and self.robot_manager.robots[next_robot].task_step == 0.5:
                 self.robot_manager.robots[next_robot].last_goal_time = 0
                 self.command_log(f"📢 {next_robot} 픽업대 차례! 이동 재개.")
    
    def optimize_task_queue(self):
        self.global_task_queue.sort(key=lambda t: self.scheduler.calculate_priority_score(t), reverse=True)

    def clear_all_tasks(self):
        self.global_task_queue.clear()

    def publish_task_queue_update(self):
        if self.input_only:
            tasks_data = []
            all_tasks = self.global_task_queue + [
                r.current_task for r in self.robot_manager.robots.values() if r.current_task
            ]
            for task in all_tasks:
                tasks_data.append({
                    'id': task.id,
                    'type': task.type.value,
                    'destination': task.destination,
                    'robot_name': task.robot_name,
                    'item': task.item,
                    'priority': task.priority.value,
                    'status': task.status.value,
                    'created_time': task.created_time,
                    'estimated_duration': task.estimated_duration
                })
            self.robot_manager.publish_task_queue({'tasks': tasks_data})

    def cleanup_expired_data(self):
        """🧹 메모리 누수 방지: 만료된 데이터 정리"""
        current_time = time.time()
        
        # 24시간 이상 된 완료 작업 정리
        expired_tasks = [
            task for task in self.completed_tasks
            if hasattr(task, 'created_time') and current_time - task.created_time > 86400
        ]
        for task in expired_tasks:
            self.completed_tasks.remove(task)
        
        # 1시간 이상 된 확인 요청 시간 정리 (confirmation_manager에서 관리)
        if hasattr(self.confirmation_manager, 'confirmation_sent_times'):
            expired_confirmations = [
                robot_name for robot_name, sent_time in self.confirmation_manager.confirmation_sent_times.items()
                if current_time - sent_time > 3600
            ]
            for robot_name in expired_confirmations:
                del self.confirmation_manager.confirmation_sent_times[robot_name]
        
        # 1시간 이상 된 작업 시작 시간 정리 (비활성 로봇)
        expired_task_times = [
            robot_name for robot_name, start_time in self.robot_task_start_times.items()
            if current_time - start_time > 3600 and not self.robot_manager.robots[robot_name].current_task
        ]
        for robot_name in expired_task_times:
            del self.robot_task_start_times[robot_name]
        
        if expired_tasks or (hasattr(self.confirmation_manager, 'confirmation_sent_times') and expired_confirmations) or expired_task_times:
            self.command_log(f"🧹 메모리 정리: 작업 {len(expired_tasks)}개, 확인요청 {len(expired_confirmations) if hasattr(self.confirmation_manager, 'confirmation_sent_times') else 0}개, 추적데이터 {len(expired_task_times)}개 정리됨")