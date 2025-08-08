#!/usr/bin/env python3

import threading
import queue
import time
import json
from datetime import datetime
from typing import List, Optional

from robot_manager import Task, TaskType, TaskStatus, Priority, RobotState
from pinky_interfaces.msg import Confirmation
from dataclasses import dataclass
from intelligent_scheduler import IntelligentTaskScheduler

@dataclass
class ConfirmationRequest:
    """확인 요청 데이터 구조"""
    robot_name: str
    prompt: str
    interrupted_task: Optional[Task] = None
    interrupted_robot: str = ""
    pending_return_task: Optional[Task] = None
    pending_return_robot: str = ""
    retry_on_no: bool = False  # NO 선택 시 재시도 여부 (픽업/배달용)

from command_parser import CommandParser
from robot_arm_interface import RobotArmInterface  # 로봇팔 연동 완료
# from global_camera_interface import GlobalCameraInterface  # 글로벌 카메라 연동 시 주석 해제

class ROSATaskProcessor:
    """작업 처리 및 AI 스케줄링의 메인 컨트롤러"""

    def __init__(self, robot_manager):
        self.robot_manager = robot_manager
        self.scheduler = IntelligentTaskScheduler(robot_manager)
        self.command_parser = CommandParser(robot_manager, self)
        
        # 로봇팔 인터페이스 추가
        self.arm_interface = RobotArmInterface(robot_manager, self)  # 로봇팔 연동 완료
        
        # 글로벌 카메라 인터페이스 추가 (임시로 None으로 설정)
        self.camera_interface = None  # GlobalCameraInterface(self)  # 글로벌 카메라 연동 시 주석 해제
        
        self.global_task_queue: List[Task] = []
        self.suspended_tasks: List[Task] = []
        self.completed_tasks: List[Task] = []
        self.command_queue = queue.Queue()

        # 기존 단일 확인 시스템 (호환성 유지)
        self.waiting_for_global_confirmation = False
        self.global_confirmation_prompt = ""
        self.confirmation_robot = ""
        self.interrupted_task = None
        self.interrupted_robot = ""
        self.pending_return_task = None
        self.pending_return_robot = ""
        
        # 새로운 확인 요청 큐 시스템
        self.confirmation_queue = queue.Queue()
        self.current_confirmation = None
        
        # 픽업대 해제 타이머 시스템
        self.pickup_release_timer = None
        self.pickup_release_delay = 0.5  # 0.5초 후 다음 로봇 호출 (빠른 반응)
        
        # Task ID 카운터 (고유한 ID 생성용)
        self._task_counter = 0
        self.pickup_area_released_robots = set()  # 이미 해제 처리된 로봇들 (중복 처리 방지)
        
        # 🚨 월급루팡 방지 시스템
        self.robot_task_start_times = {}  # 로봇별 작업 시작 시간 추적
        self.robot_last_move_times = {}   # 로봇별 마지막 이동 시간 추적
        self.stuck_timeout = 60.0         # 1분 = 60초 (월급루팡 판정 기준 - 더 빠르게 감지)
        self.confirmation_sent_times = {} # 확인창 발송 시간 추적 (응답 없음 감지용)

        self.status_only = False
        self.input_only = False
        self.gui = None

        self.robot_manager.create_timer(0.5, self.process_commands)  # 0.1초 → 0.5초로 최적화
        self.robot_manager.create_timer(1.0, self.task_processor_loop)
        self.robot_manager.create_timer(10.0, self.optimize_task_queue)
        self.robot_manager.create_timer(1.0, self.process_confirmation_queue)  # 0.5초 → 1초로 최적화
        self.robot_manager.create_timer(30.0, self.check_stuck_robots)  # 🚨 월급루팡 감시 (30초마다)  # 확인 요청 큐 처리
        self.robot_manager.create_timer(0.5, self._check_pickup_area_status)  # 픽업대 상태 모니터링
        self.robot_manager.create_timer(300.0, self.cleanup_expired_data)  # 5분마다 메모리 정리
        
        # 글로벌 카메라 긴급 알림 구독 (주석 처리)
        # self._setup_emergency_subscribers()  # 글로벌 카메라 연동 시 주석 해제
        
        self.log_message("📋 ROSA 작업 처리기 시작!", is_command=False)

    def _setup_emergency_subscribers(self):
        """글로벌 카메라 긴급 알림 구독 설정"""
        # 긴급 알림 구독
        self.robot_manager.create_subscription(
            'String', '/rosa/emergency_alert', 
            self.handle_emergency_alert, 10
        )
        
        # 위치 보정 정보 구독  
        self.robot_manager.create_subscription(
            'String', '/rosa/position_correction',
            self.handle_position_correction, 10
        )
        
        self.log_message("🎥 글로벌 카메라 긴급 알림 시스템 활성화", is_command=False)

    def handle_emergency_alert(self, msg):
        """글로벌 카메라로부터 긴급 알림 처리"""
        try:
            alert_data = json.loads(msg.data)
            alert_type = alert_data.get('type', 'unknown')
            
            if alert_type == 'fall_detected':
                self._handle_fall_emergency(alert_data)
            else:
                self.log_message(f"⚠️ 알 수 없는 긴급 알림 타입: {alert_type}", "WARN")
                
        except json.JSONDecodeError as e:
            self.log_message(f"❌ 긴급 알림 데이터 파싱 실패: {e}", "ERROR")

    def _handle_fall_emergency(self, alert_data):
        """낙상 감지 긴급 상황 처리"""
        location = alert_data.get('location', {})
        fall_x, fall_y = location.get('x', 0), location.get('y', 0)
        
        self.log_message(f"🚨 낙상 감지! 위치: ({fall_x:.2f}, {fall_y:.2f})", "URGENT", is_command=True)
        
        # 긴급 작업 생성
        emergency_task = Task(
            id=f"EMERGENCY_FALL_{int(time.time())}",
            type=TaskType.DELIVERY,  # 응급처치 키트 배달로 임시 설정
            destination="낙상위치",  # 실제로는 좌표 변환 필요
            item="응급처치키트",
            priority=Priority.URGENT,
            status=TaskStatus.PENDING
        )
        
        # 가장 가까운 로봇 찾아서 즉시 할당
        closest_robot = self._find_closest_available_robot(fall_x, fall_y)
        
        if closest_robot:
            # 현재 작업 중단하고 긴급 작업 우선 처리
            self._preempt_for_emergency(closest_robot, emergency_task)
            self.log_message(f"🚑 긴급 파견: {closest_robot} → 낙상 위치", "URGENT", is_command=True)
        else:
            # 사용 가능한 로봇이 없으면 대기열 최우선으로 추가
            self.global_task_queue.insert(0, emergency_task)
            self.log_message("⚠️ 사용 가능한 로봇 없음 - 긴급 작업 대기열 추가", "WARN", is_command=True)

    def _find_closest_available_robot(self, target_x: float, target_y: float) -> Optional[str]:
        """목표 위치에 가장 가까운 사용 가능한 로봇 찾기"""
        closest_robot = None
        min_distance = float('inf')
        
        for robot_name, robot in self.robot_manager.robots.items():
            # 배터리 부족, 충전 중, 오류 상태인 로봇은 제외
            if robot.battery_level < 30 or robot.state in [RobotState.CHARGING, RobotState.LOW_BATTERY]:
                continue
                
            # 로봇 현재 위치 가져오기
            robot_pos = self.robot_manager.get_robot_position(robot_name)
            if not robot_pos:
                continue
                
            # 거리 계산
            distance = ((robot_pos['x'] - target_x) ** 2 + (robot_pos['y'] - target_y) ** 2) ** 0.5
            
            if distance < min_distance:
                min_distance = distance
                closest_robot = robot_name
                
        return closest_robot

    def _preempt_for_emergency(self, robot_name: str, emergency_task: Task):
        """긴급 상황으로 로봇의 현재 작업 중단 및 긴급 작업 할당"""
        robot = self.robot_manager.robots[robot_name]
        
        # 현재 작업 중단 및 보존
        if robot.current_task:
            robot.current_task.status = TaskStatus.PENDING
            self.global_task_queue.insert(1, robot.current_task)  # 긴급 작업 다음 순서로
            self.log_message(f"⏸️ {robot_name} 기존 작업 중단 및 보존", is_command=True)
        
        # 긴급 작업 즉시 할당
        emergency_task.robot_name = robot_name
        emergency_task.estimated_duration = self.scheduler.estimate_task_duration(robot, emergency_task)
        robot.current_task = emergency_task
        robot.task_step = 0
        
        # 작업 실행 시작
        self._start_task_execution(robot_name, emergency_task)

    def handle_position_correction(self, msg):
        """글로벌 카메라로부터 위치 보정 정보 처리"""
        try:
            correction_data = json.loads(msg.data)
            robot_id = correction_data.get('robot_id')
            correction_type = correction_data.get('correction_type')
            
            if robot_id in self.robot_manager.robots:
                # 위치 보정 정보를 로봇 매니저에 전달
                if correction_type == 'global_camera_tracking':
                    position = correction_data.get('position', {})
                    self.robot_manager.update_robot_global_position(robot_id, position)
                elif correction_type == 'aruco_precise_tracking':
                    precise_position = correction_data.get('precise_position', {})
                    self.robot_manager.update_robot_precise_position(robot_id, precise_position)
                    
        except json.JSONDecodeError as e:
            self.log_message(f"❌ 위치 보정 데이터 파싱 실패: {e}", "ERROR")

    def add_emergency_task(self, emergency_data: dict):
        """외부에서 긴급 작업 추가 (글로벌 카메라 인터페이스용)"""
        if emergency_data.get('type') == 'fall_detected':
            self._handle_fall_emergency(emergency_data)

    def set_gui(self, gui):
        self.gui = gui

    def log_message(self, message: str, level: str = "INFO", is_command: bool = False):
        log_type = "COMMAND" if is_command else "SYSTEM"
        if self.status_only:
            if is_command:
                print(f"[{datetime.now().strftime('%H:%M:%S')}] {message}")
            elif self.gui:
                system_keywords = ["연결", "확인", "응답", "GUI", "TF", "배터리", "복구", "중단", "불안정"]
                if any(keyword in message for keyword in system_keywords):
                    self.gui.add_log(message, level)
        elif self.input_only:
            self.robot_manager.publish_log_message(message, level, log_type)
        else:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] {message}")

    def command_log(self, message: str, level: str = "INFO"):
        self.log_message(message, level, is_command=True)

    def system_log(self, message: str, level: str = "INFO"):
        self.log_message(message, level, is_command=False)

    def start_input_thread(self):
        threading.Thread(target=self._input_thread_func, daemon=True).start()

    def _input_thread_func(self):
        prompt = "🤖 [ROSA] 명령 > "
        while True:
            try:
                cmd = input(prompt).strip()
                if cmd:
                    self.command_queue.put(cmd)
            except (EOFError, KeyboardInterrupt):
                self.log_message("👋 종료")
                import rclpy
                rclpy.shutdown()
                break
            except Exception as e:
                print(f"⚠️ 입력 오류: {e}")

    def process_commands(self):
        if self.status_only:
            return
        while not self.command_queue.empty():
            cmd = self.command_queue.get().strip()
            if not cmd:
                continue
            
            if cmd.lower() in ['quit', 'exit', '종료']:
                self.log_message("👋 시스템 종료", is_command=True)
                import rclpy
                rclpy.shutdown()
                return
            
            if cmd.lower() == 'debug': self.show_debug_info()
            elif cmd.lower() == 'status': self.show_status()
            elif cmd.lower() == 'queue': self.show_queue()
            elif cmd.lower() == 'clear': self.clear_all_tasks()
            else: self.command_parser.parse_intelligent_command(cmd)

    def add_task_to_queue(self, task: Task, robot_name: Optional[str], is_return_command: bool, is_charge_station_command: bool):
        if robot_name:
            robot = self.robot_manager.robots[robot_name]
            # 복귀 작업인 경우 임시 저장
            if is_return_command or is_charge_station_command:
                self.pending_return_task = task
                self.pending_return_robot = robot_name
            if self.handle_preemption(robot, is_return_command, is_charge_station_command):
                return
            task.robot_name = robot_name
            task.estimated_duration = self.scheduler.estimate_task_duration(robot, task)
        else:
            self.log_message("⏳ 대기열에 추가 - 로봇 가용시 자동 할당", is_command=True)
            task.robot_name = ""
            task.estimated_duration = 300.0

        self.global_task_queue.append(task)
        self.publish_task_queue_update()
        self.log_message("✅ 작업 생성 완료!", "SUCCESS", is_command=True)
        self.log_message(f"🆔 작업 ID: {task.id}", is_command=True)

        if task.robot_name:
            self.execute_or_wait_for_robot(task)
        else:
            self.try_assign_tasks_to_available_robots()

    def handle_preemption(self, robot, is_return_command, is_charge_station_command) -> bool:
        if robot.is_permanently_stationed:
            self.log_message(f"🔄 {robot.name} 영구 대기 해제 - 새 명령 처리", is_command=True)
            robot.is_permanently_stationed = False
        if robot.state == RobotState.WAITING_FOR_CONFIRMATION:
            self.log_message(f"🔄 {robot.name} 응답 대기 해제 - 지정 명령 처리", is_command=True)
            robot.state = RobotState.IDLE
            robot.waiting_for_confirmation = False
            robot.confirmation_prompt = ""

        if robot.current_task and not (is_return_command or is_charge_station_command):
            self.log_message(f"❌ {robot.name}이 다른 업무 진행 중 (ID: {robot.current_task.id})", "ERROR", is_command=True)
            return True

        if robot.current_task and (is_return_command or is_charge_station_command):
            self.log_message(f"🏠 {robot.name}이 충전소로 복귀합니다. 진행중인 업무를 대기열에 추가하시겠습니까?", "WARNING", is_command=True)
            # 복귀 명령 확인 요청을 큐에 추가 (복귀 작업은 나중에 생성)
            self.queue_confirmation_request(
                robot_name=robot.name,
                prompt="add_to_queue",
                interrupted_task=robot.current_task,
                interrupted_robot=robot.name,
                pending_return_task=None,  # 복귀 작업은 확인 후 생성됨
                pending_return_robot=robot.name
            )
            robot.current_task = None
            robot.task_step = 0
            return True
        
        return False

    def execute_or_wait_for_robot(self, task):
        robot = self.robot_manager.robots[task.robot_name]
        self.log_message(f"🤖 할당 로봇: {task.robot_name}", is_command=True)
        
        if not robot.current_pose:
            self.log_message(f"⏳ {task.robot_name}의 연결이 확인되면 '{task.destination}' 작업을 시작합니다.", "WARNING", is_command=True)
        else:
            self._start_task_execution(task.robot_name, task)

    def task_processor_loop(self):
        for robot_name, robot in self.robot_manager.robots.items():
            if robot.state == RobotState.LOW_BATTERY:
                if robot.current_task:
                    self.suspend_robot_task(robot_name)
                continue
            
            if not robot.current_task:
                self.assign_next_task(robot_name)
            
            if robot.current_task:
                self.process_robot_task(robot_name)
        
        self.process_location_queues()

    def assign_next_task(self, robot_name: str):
        robot = self.robot_manager.robots[robot_name]
        if robot.is_permanently_stationed or not robot.current_pose or robot.current_task:
            return

        for i, task in enumerate(self.global_task_queue):
            if task.robot_name == robot_name:
                robot.current_task = self.global_task_queue.pop(i)
                self.log_message(f"🎯 {robot_name} 예약된/기존 할당 업무 시작!")
                self.publish_task_queue_update()  # 대기열에서 제거 시 GUI 업데이트
                self._start_task_execution(robot_name, robot.current_task)
                return

        if robot.state in [RobotState.IDLE, RobotState.RETURNING, RobotState.CHARGING]:
            self.try_assign_single_robot(robot_name)

    def try_assign_tasks_to_available_robots(self):
        unassigned_tasks = [task for task in self.global_task_queue if not task.robot_name]
        if not unassigned_tasks: return
        
        unassigned_tasks.sort(key=lambda t: self.scheduler.calculate_priority_score(t), reverse=True)
        
        for task in unassigned_tasks:
            robot_name = self.scheduler.find_optimal_robot(task, self.log_message)
            if robot_name:
                task_in_queue = next((t for t in self.global_task_queue if t.id == task.id), None)
                if task_in_queue and not task_in_queue.robot_name:
                    task_in_queue.robot_name = robot_name
                    task_in_queue.estimated_duration = self.scheduler.estimate_task_duration(self.robot_manager.robots[robot_name], task_in_queue)
                    self.system_log(f"🎯 {robot_name} 대기열에서 새 업무 자동 할당 (ID: {task.id})")
                    self.assign_next_task(robot_name)
                    break

    def try_assign_single_robot(self, robot_name: str):
        robot = self.robot_manager.robots[robot_name]
        if robot.current_task: return

        unassigned_tasks = [task for task in self.global_task_queue if not task.robot_name]
        if not unassigned_tasks: return

        unassigned_tasks.sort(key=lambda t: self.scheduler.calculate_priority_score(t), reverse=True)
        
        best_task = unassigned_tasks[0]
        best_task.robot_name = robot_name
        best_task.estimated_duration = self.scheduler.estimate_task_duration(robot, best_task)
        self.system_log(f"🎯 {robot_name} 대기열에서 새 업무 자동 할당 (ID: {best_task.id})")
        self.assign_next_task(robot_name)

    def process_robot_task(self, robot_name: str):
        robot = self.robot_manager.robots[robot_name]
        task = robot.current_task
        if not task: return

        if robot.state == RobotState.WAITING_FOR_CONFIRMATION:
            return
        if self.waiting_for_global_confirmation and self.confirmation_robot != robot_name:
            return

        if task.type == TaskType.DELIVERY: self._process_delivery_task(robot, task)
        elif task.type == TaskType.SIMPLE_MOVE: self._process_simple_move_task(robot, task)
        elif task.type == TaskType.RETURN_TO_CHARGE: self._process_return_task(robot, task)

    # [신규] _plan_path_sequence 함수
    def _plan_path_sequence(self, robot_name: str, destination_name: str) -> Optional[List[str]]:
        """시작점과 도착점을 보고 최적의 웨이포인트 경로 조합을 반환합니다."""
        robot = self.robot_manager.robots.get(robot_name)
        if not robot or not robot.current_pose:
            self.system_log(f"경로 계획 실패: {robot_name}의 위치 정보 없음", "ERROR")
            return None

        # y 좌표를 기준으로 상행/하행 판단
        start_y = robot.current_pose.position.y
        end_coords = self.robot_manager.location_manager.get_location_coordinates(destination_name)
        if not end_coords:
            self.system_log(f"경로 계획 실패: 목적지 {destination_name}의 좌표 없음", "ERROR")
            return None
        end_y = end_coords[1]
        
        # 단순화를 위해, y좌표가 증가하면 상행, 감소하면 하행으로 결정
        if start_y < end_y:
            return ["highway_up"]
        else:
            return ["highway_down"]

    def is_first_in_pickup_order(self, robot_name: str) -> bool:
        """명령 순서 기반 픽업대 사용권 판정 (먼저 명령받은 로봇이 우선권) - 멈춘 로봇 감지 개선"""
        current_robot = self.robot_manager.robots[robot_name]
        if not current_robot.current_task:
            return True
            
        current_task_time = current_robot.current_task.created_time
        current_time = time.time()
        
        # 픽업대 관련 작업 중인 다른 로봇들과 명령 시간 비교
        for name, robot in self.robot_manager.robots.items():
            if (name != robot_name and 
                robot.current_task and 
                robot.current_task.type == TaskType.DELIVERY and 
                robot.task_step in [1, 1.5]):  # 픽업대로 가는 중 또는 픽업 중
                
                # 다른 로봇이 더 먼저 명령을 받았다면...
                if robot.current_task.created_time < current_task_time:
                    # 🚨 하지만 그 로봇이 30초 이상 멈춰있으면 무시
                    robot_task_start_time = self.robot_task_start_times.get(name, current_time)
                    robot_stuck_time = current_time - robot_task_start_time
                    
                    if robot_stuck_time > 15.0 and not robot.is_moving:
                        self.command_log(f"⚠️ {name} 로봇이 30초 이상 멈춤 - {robot_name}에게 우선권 넘김")
                        return True  # 멈춘 로봇 무시하고 우선권 획득
                    
                    step_desc = "픽업 중" if robot.task_step == 1.5 else "이동 중"
                    self.command_log(f"⏰ {name}이 먼저 명령받음 ({step_desc}) - {robot_name} 대기")
                    return False
        
        return True  # 픽업대 사용 우선권 있음

    def check_stuck_robots(self):
        """🚨 월급루팡 방지: 2분 이상 멈춰있는 로봇 복귀시키기"""
        current_time = time.time()
        
        for robot_name, robot in self.robot_manager.robots.items():
            if not robot.current_task:
                continue
                
            # 작업 시작 시간 추적
            if robot_name not in self.robot_task_start_times:
                self.robot_task_start_times[robot_name] = current_time
                continue
                
            # 마지막 이동 시간 업데이트 (이동 중이면)
            if robot.is_moving:
                self.robot_last_move_times[robot_name] = current_time
                
            # 작업 시작 후 경과 시간 계산
            task_elapsed = current_time - self.robot_task_start_times[robot_name]
            
            # 마지막 이동 후 경과 시간 계산
            last_move_time = self.robot_last_move_times.get(robot_name, self.robot_task_start_times[robot_name])
            stuck_time = current_time - last_move_time
            
            # 🚨 월급루팡 판정 (YES/NO 대기 중은 제외)
            is_waiting_confirmation = (robot.state == RobotState.WAITING_FOR_CONFIRMATION or 
                                     self.waiting_for_global_confirmation and self.confirmation_robot == robot_name)
            
            # YES/NO 대기 중이 아닌 경우에만 월급루팡 체크
            should_check_stuck = not is_waiting_confirmation
            
            # 디버그: 로봇 상태 출력 (60초마다)
            if int(current_time) % 60 == 0 and robot.current_task:
                self.command_log(f"🔍 DEBUG {robot_name}: 작업={robot.current_task.id}, 상태={robot.state.name}, 이동중={robot.is_moving}, 작업시간={task_elapsed:.0f}초, 정지시간={stuck_time:.0f}초")
            
            # 🆘 YES/NO 확인창 응답 없음 감지 (30초 이상 응답 없으면 문제)
            confirmation_timeout = False
            if is_waiting_confirmation and robot_name in self.confirmation_sent_times:
                confirmation_wait_time = current_time - self.confirmation_sent_times[robot_name]
                if confirmation_wait_time > 30.0:  # 30초 이상 응답 없음
                    self.command_log(f"🆘 {robot_name} 확인창 응답 없음! {confirmation_wait_time:.0f}초 대기중")
                    confirmation_timeout = True
            
            # 월급루팡 또는 확인창 응답 타임아웃 (더 빠른 감지)
            if should_check_stuck and (task_elapsed > self.stuck_timeout or stuck_time > 15.0):
                self.command_log(f"🚨 {robot_name} 월급루팡 감지! 작업시간: {task_elapsed:.0f}초, 정지시간: {stuck_time:.0f}초")
                self.command_log(f"📢 {robot_name} 강제 복귀 - 나쁜 녀석! 월급루팡 금지!")
            elif confirmation_timeout:
                self.command_log(f"🆘 {robot_name} 확인창 응답 없음! 강제 복귀")
                self.command_log(f"📢 {robot_name} 멍때리기 금지! 확인창이 안 나타나는 문제!")
            else:
                continue  # 문제없음, 다음 로봇 체크
                
            # 중단된 작업 정보 저장 (확인창에서 사용)
            self.interrupted_task = robot.current_task
            self.interrupted_robot = robot_name
            
            # 픽업대 사용 중이었다면 해제
            if robot.task_step in [1, 1.5]:  # 픽업대로 이동 중이거나 픽업 중
                # location_occupancy를 직접 확인
                current_occupant = self.robot_manager.location_manager.location_occupancy.get("픽업대")
                if current_occupant == robot_name:
                    self.robot_manager.location_manager.release_location("픽업대", robot_name)
                    self.command_log(f"📦 {robot_name} 픽업대 해제됨 (월급루팡 감지)")
                    
                    # 다음 대기 로봇이 자동으로 픽업대로 이동하도록 처리
                    self.command_log("⏱️ 2초 후 다음 대기 로봇을 픽업대로 호출합니다")
            
            # 로봇 강제 복귀
            self.force_robot_return(robot_name)
            
            # 📋 중단된 작업 처리 방법 확인 요청
            self.command_log(f"🤔 {robot_name} 중단된 작업 '{self.interrupted_task.id}' 처리 방법 확인 필요")
            self.send_confirmation_request_with_retry(robot_name, "add_to_queue")

    def force_robot_return(self, robot_name: str):
        """로봇 강제 복귀"""
        robot = self.robot_manager.robots[robot_name]
        
        # 현재 작업 정리
        robot.current_task = None
        robot.task_step = 0
        robot.state = RobotState.RETURNING
        robot.queue_position = -1
        
        # 픽업대 해제
        self.robot_manager.location_manager.release_location('픽업대', robot_name)
        self.robot_manager.location_manager.leave_queue('픽업대', robot_name)
        
        # 추적 정보 초기화
        if robot_name in self.robot_task_start_times:
            del self.robot_task_start_times[robot_name]
        if robot_name in self.robot_last_move_times:
            del self.robot_last_move_times[robot_name]
        
        # 충전소로 강제 복귀
        charge_station = self.robot_manager.get_charge_station_for_robot(robot_name)
        self.robot_manager.move_robot_to_location(robot_name, charge_station)
    
    def _cleanup_robot_tracking_data(self, robot_name: str):
        """🔧 메모리 누수 방지: 로봇 추적 정보 정리"""
        if robot_name in self.robot_task_start_times:
            del self.robot_task_start_times[robot_name]
        if robot_name in self.robot_last_move_times:
            del self.robot_last_move_times[robot_name]
        if robot_name in self.confirmation_sent_times:
            del self.confirmation_sent_times[robot_name]
    
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
        
        # 1시간 이상 된 확인 요청 시간 정리
        expired_confirmations = [
            robot_name for robot_name, sent_time in self.confirmation_sent_times.items()
            if current_time - sent_time > 3600
        ]
        for robot_name in expired_confirmations:
            del self.confirmation_sent_times[robot_name]
        
        # 1시간 이상 된 작업 시작 시간 정리 (비활성 로봇)
        expired_task_times = [
            robot_name for robot_name, start_time in self.robot_task_start_times.items()
            if current_time - start_time > 3600 and not self.robot_manager.robots[robot_name].current_task
        ]
        for robot_name in expired_task_times:
            del self.robot_task_start_times[robot_name]
            if robot_name in self.robot_last_move_times:
                del self.robot_last_move_times[robot_name]
        
        if expired_tasks or expired_confirmations or expired_task_times:
            self.command_log(f"🧹 메모리 정리: 작업 {len(expired_tasks)}개, 확인요청 {len(expired_confirmations)}개, 추적데이터 {len(expired_task_times)}개 정리됨")

    def _process_delivery_task(self, robot, task):
        navigator = self.robot_manager.navigators.get(robot.name)
        if not navigator:
            self.fail_robot_task(robot.name, "Navigator not available")
            return

        # --- 1단계: 픽업대로 이동 및 대기 처리 ---
        if robot.task_step in [0.5, 1]:
            # [최초 실행] 아직 이동 명령을 내리지 않았다면, 픽업대 상태를 확인하고 경로를 결정합니다.
            if robot.last_goal_time == 0:
                pickup_occupied = self.robot_manager.location_manager.is_location_busy('픽업대', robot.name)
                next_in_queue = self.robot_manager.location_manager.get_next_in_queue('픽업대')

                # Case 1: 픽업대가 꽉 찼거나, 아직 내 순서가 아니면 대기장소로 이동합니다.
                if pickup_occupied or (next_in_queue and next_in_queue != robot.name):
                    queue_pos = self.robot_manager.location_manager.join_queue('픽업대', robot.name)
                    robot.task_step = 0.5
                    robot.queue_position = queue_pos
                    robot.state = RobotState.WAITING_IN_QUEUE
                    self.command_log(f"📦 {robot.name} 픽업대 혼잡/순서 대기 -> 대기장소로 이동 ({queue_pos + 1}번째)")
                    # 대기 장소로 가는 것은 간단한 이동이므로 goal_pose를 사용합니다.
                    self.robot_manager.move_robot_to_location(robot.name, "픽업대기장소")
                    
                # Case 2: 픽업대가 비어있고, 내 순서가 맞으면 픽업대로 이동을 시작합니다.
                else:
                    robot.task_step = 1 # 이동 단계로 전환
                    self.command_log(f"📦 {robot.name} 픽업대로 이동 시작")
                    path_sequence = self._plan_path_sequence(robot.name, "픽업대")
                    if path_sequence and self.robot_manager.follow_waypoints(robot.name, path_sequence[0]):
                        robot.last_goal_time = time.time() # 이동 시작 시간 기록
                    else:
                        self.fail_robot_task(robot.name, "Path planning/start failed")
                return # 첫 명령 후에는 한 턴 쉽니다.

            # [이동 중] Nav2가 경로 주행을 완료했다고 알려주면, 도착한 것입니다.
            if robot.task_step == 1 and navigator.isTaskComplete():
                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    robot.last_goal_time = 0 # 이동 완료
                    robot.task_step = 1.5      # 적재 대기 단계로 전환
                    self.robot_manager.location_manager.occupy_location('픽업대', robot.name)
                    self.command_log(f"📦 {robot.name} 픽업대 도착 -> {task.item} 적재 대기")
                    # 로봇팔 호출 및 GUI 확인 요청
                    self.arm_interface.request_pickup(robot.name, task.item)
                    self.send_confirmation_request_with_retry(robot.name, "item_loaded")
                else:
                    self.fail_robot_task(robot.name, f"Navigation to pickup failed with status: {result}")

        # --- 2단계: 픽업대에서 적재 대기 --- (수정할 필요 없음)
        elif robot.task_step == 1.5:
            # 외부 신호(GUI/로봇팔)를 기다리는 상태
            pass

        # --- 3단계: 배달지로 이동 ---
        elif robot.task_step == 2:
            # [최초 실행] 배달지 이동 명령을 내립니다.
            if robot.last_goal_time == 0:
                self.command_log(f"🚚 {robot.name} -> {task.destination} 배달 시작")
                path_sequence = self._plan_path_sequence(robot.name, task.destination)
                if path_sequence and self.robot_manager.follow_waypoints(robot.name, path_sequence[0]):
                    robot.last_goal_time = time.time()
                else:
                    self.fail_robot_task(robot.name, "Path planning/start failed")
                return

            # [이동 중] Nav2가 경로 주행을 완료했다고 알려주면, 도착한 것입니다.
            if navigator.isTaskComplete():
                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    robot.last_goal_time = 0
                    robot.task_step = 2.5 # 최종 확인 단계로 전환
                    self.command_log(f"📍 {robot.name} {task.destination} 도착 -> 전달 완료 확인 대기")
                    self.send_confirmation_request_with_retry(robot.name, "item_delivered")
                else:
                    self.fail_robot_task(robot.name, f"Navigation to destination failed with status: {result}")
        
        # --- 4단계: 최종 전달 확인 --- (수정할 필요 없음)
        elif robot.task_step == 2.5:
            # GUI의 확인 응답을 기다리는 상태
            pass

    def _process_simple_move_task(self, robot, task):
        navigator = self.robot_manager.navigators.get(robot.name)
        if not navigator:
            self.fail_robot_task(robot.name, "Navigator not available")
            return

        # [최초 실행] 이동 명령을 내립니다.
        if robot.last_goal_time == 0:
            self.command_log(f"🚶 {robot.name} -> {task.destination} 이동 시작")
            path_sequence = self._plan_path_sequence(robot.name, task.destination)
            if path_sequence and self.robot_manager.follow_waypoints(robot.name, path_sequence[0]):
                robot.last_goal_time = time.time()
            else:
                self.fail_robot_task(robot.name, "Path planning/start failed")
            return
        
        # [이동 중] Nav2가 작업 완료를 알려주면 도착한 것입니다.
        if navigator.isTaskComplete():
            # 대기/복귀 여부를 묻는 확인 창을 띄웁니다.
            if hasattr(task, 'is_charge_station_command') and task.is_charge_station_command:
                self.complete_robot_task_without_return(robot.name)
            else:
                self.send_confirmation_request(robot.name, "stay_or_return")

    def _process_return_task(self, robot, task):
        navigator = self.robot_manager.navigators.get(robot.name)
        if not navigator:
            self.fail_robot_task(robot.name, "Navigator not available")
            return
            
        # [최초 실행] 복귀 명령을 내립니다.
        if robot.last_goal_time == 0:
            self.command_log(f"🏠 {robot.name} -> {task.destination} 복귀 시작")
            path_sequence = self._plan_path_sequence(robot.name, task.destination)
            if path_sequence and self.robot_manager.follow_waypoints(robot.name, path_sequence[0]):
                robot.last_goal_time = time.time()
            else:
                self.fail_robot_task(robot.name, "Path planning/start failed")
            return

        # [이동 중] Nav2가 작업 완료를 알려주면 도착한 것입니다.
        if navigator.isTaskComplete():
            self.command_log(f"🏠 {robot.name} {task.destination} 복귀 완료")
            self.complete_robot_task_without_return(robot.name)

    # --- 로봇팔 연동을 위한 함수들 (신규 및 수정) ---

    def _confirm_pickup_and_proceed(self, robot_name: str):
        """(신규) 픽업 확인 후 다음 단계를 진행하는 공통 헬퍼 함수"""
        # 이미 처리가 시작되었는지 확인 (중복 실행 방지)
        if not self.waiting_for_global_confirmation and self.confirmation_robot != robot_name:
             # 확인 상태가 이미 클리어되었다면, 다른 쪽에서 이미 처리한 것이므로 무시
            return

        robot = self.robot_manager.robots.get(robot_name)
        if not robot or not robot.current_task or robot.task_step != 1.5:
            return

        self.command_log(f"👍 Pickup confirmed for {robot_name}. Starting delivery.")
        self.robot_manager.location_manager.release_location('픽업대', robot_name)
        robot.task_step = 2
        
        self.clear_confirmation_state()
        self._start_task_execution(robot_name, robot.current_task)

    def handle_arm_pickup_completion(self):
        """로봇팔의 'completed' 신호를 받았을 때 호출됩니다."""
        waiting_robot_name = None
        for name, robot in self.robot_manager.robots.items():
            if robot.current_task and robot.current_task.type == TaskType.DELIVERY and robot.task_step == 1.5:
                waiting_robot_name = name
                break
        
        if waiting_robot_name:
            self.system_log(f"🦾 Arm completion signal received for {waiting_robot_name}. Proceeding automatically.")
            self._confirm_pickup_and_proceed(waiting_robot_name)
        else:
            self.system_log("Arm completion signal received, but no robot was waiting for pickup.", "WARNING")

    def fail_robot_task(self, robot_name: str, reason: str):
        robot = self.robot_manager.robots.get(robot_name)
        if not robot or not robot.current_task: return
        task = robot.current_task
        self.command_log(f"Task failed (ID: {task.id}): {reason}", "ERROR")
        task.status = TaskStatus.FAILED
        self.completed_tasks.append(task)
        robot.current_task = None
        robot.task_step = 0
        if task.type == TaskType.DELIVERY:
            self.robot_manager.location_manager.release_location('픽업대', robot_name)
        self.clear_confirmation_state()
        self.complete_robot_task(robot_name)

    def send_confirmation_request(self, robot_name: str, prompt: str):
        """기존 방식과의 호환성을 위한 래퍼 함수"""
        # 현재 처리 중인 확인이 있으면 큐에 추가
        if self.waiting_for_global_confirmation:
            self.queue_confirmation_request(
                robot_name=robot_name,
                prompt=prompt,
                interrupted_task=self.interrupted_task,
                interrupted_robot=self.interrupted_robot,
                pending_return_task=self.pending_return_task,
                pending_return_robot=self.pending_return_robot
            )
        else:
            # 즉시 처리
            self.waiting_for_global_confirmation = True
            self.global_confirmation_prompt = prompt
            self.confirmation_robot = robot_name
            msg = Confirmation()
            msg.robot_name = robot_name
            msg.prompt = prompt
            msg.confirmed = False  # 요청임을 나타냄
            
            # 🔍 디버그: 확인 요청 전송 상세 로깅
            self.command_log(f"🔍 확인 요청 ROS 메시지 전송: {robot_name} - {prompt}")
            self.robot_manager.confirmation_request_pub.publish(msg)
            self.command_log(f"📋 즉시 확인 요청 전송 완료: {robot_name} - {prompt}")
            self.system_log(f"📋 즉시 확인 요청: {robot_name} - {prompt}")

    def send_confirmation_request_with_retry(self, robot_name: str, prompt: str):
        """픽업/배달 확인 요청 (NO 시 재시도)"""
        self.command_log(f"🔍 재시도 확인 요청 시작: {robot_name} - {prompt} (현재 대기 상태: {self.waiting_for_global_confirmation})")
        
        # 현재 처리 중인 확인이 있으면 큐에 추가 (재시도 옵션 포함)
        if self.waiting_for_global_confirmation:
            self.command_log(f"⏳ {robot_name} 확인 요청을 큐에 추가: {prompt}")
            self.queue_confirmation_request(
                robot_name=robot_name,
                prompt=prompt,
                interrupted_task=self.interrupted_task,
                interrupted_robot=self.interrupted_robot,
                pending_return_task=self.pending_return_task,
                pending_return_robot=self.pending_return_robot,
                retry_on_no=True
            )
        else:
            # 즉시 처리
            self.command_log(f"⚡ {robot_name} 확인 요청 즉시 처리: {prompt}")
            self.waiting_for_global_confirmation = True
            self.global_confirmation_prompt = prompt
            self.confirmation_robot = robot_name
            
            # 🆘 확인창 발송 시간 기록 (응답 없음 감지용)
            import time
            self.confirmation_sent_times[robot_name] = time.time()
            
            msg = Confirmation()
            msg.robot_name = robot_name
            msg.prompt = prompt
            msg.confirmed = False  # 요청임을 나타냄
            
            self.command_log(f"🔍 ROS 메시지 전송: {robot_name} - {prompt}")
            self.robot_manager.confirmation_request_pub.publish(msg)
            self.command_log(f"📋 재시도 가능 확인 요청 전송 완료: {robot_name} - {prompt}")
            self.system_log(f"📋 픽업/배달 확인 요청: {robot_name} - {prompt} (재시도 가능)")

    def handle_confirmation_from_ros(self, msg: Confirmation):
        if (self.waiting_for_global_confirmation and msg.robot_name == self.confirmation_robot and msg.prompt == self.global_confirmation_prompt):
            self.system_log(f"📨 GUI 응답 수신: {msg.robot_name} - {'YES' if msg.confirmed else 'NO'}")
            self.handle_confirmation('yes' if msg.confirmed else 'no')
        else:
            self.system_log(f"⚠️ 무시된 확인 응답: {msg.robot_name} (예상: {self.confirmation_robot})", "WARNING")

    def handle_confirmation(self, response: str):
        robot_name = self.confirmation_robot
        prompt = self.global_confirmation_prompt
        
        # 안전성 체크
        if not robot_name or robot_name not in self.robot_manager.robots:
            self.system_log(f"❌ 잘못된 로봇 이름: '{robot_name}'", "ERROR")
            self.clear_confirmation_state()
            return
            
        robot = self.robot_manager.robots[robot_name]
        confirmed = response.lower() in ['yes', 'y', 'ㅇ', '네', '예']

        if prompt == "item_loaded":
            if confirmed:
                self.command_log(f"✅ {robot_name} 로봇팔 픽업 확인 - 배달 시작")
                # 새로운 헬퍼 함수 호출
                self._confirm_pickup_and_proceed(robot_name)
            else:
                # NO 선택 시 재시도 (픽업 작업은 계속되어야 함)
                self.command_log(f"❌ {robot.name} 로봇팔 픽업 실패 확인 - 다른 작업 처리 후 재시도", "WARNING")
                robot.state = RobotState.WAITING_FOR_CONFIRMATION
                
                # 현재 확인 요청을 큐 마지막에 다시 추가 (다른 확인창들 먼저 처리)
                self.queue_confirmation_request(
                    robot_name=robot_name,
                    prompt="item_loaded",
                    retry_on_no=True
                )
                self.clear_confirmation_state()
                
        elif prompt == "manual_pickup":
            if confirmed:
                self.command_log(f"✅ {robot_name} 수동 적재 확인 - 배달 시작")
                # 새로운 헬퍼 함수 호출
                self._confirm_pickup_and_proceed(robot_name)
            else:
                # NO 선택 시 재시도 (픽업 작업은 계속되어야 함)
                self.command_log(f"❌ {robot.name} 수동 적재 거부 - 다른 작업 처리 후 재시도", "WARNING")
                robot.state = RobotState.WAITING_FOR_CONFIRMATION
                
                # 현재 확인 요청을 큐 마지막에 다시 추가
                self.queue_confirmation_request(
                    robot_name=robot_name,
                    prompt="manual_pickup",
                    retry_on_no=True
                )
                self.clear_confirmation_state()
        
        elif prompt == "item_delivered":
            if confirmed:
                self.command_log(f"✅ {robot_name} 배달 완료 확인")
                self.clear_confirmation_state()
                
                # 🚀 대기열에 업무가 있으면 바로 다음 업무로 진행 (유기적 워크플로우)
                if self.global_task_queue:
                    self.command_log(f"📋 {robot_name} 대기열에 업무 있음 - 즉시 다음 업무 진행")
                    self.complete_robot_task_without_return(robot_name)  # 복귀 없이 업무 완료
                else:
                    self.command_log(f"📋 {robot_name} 대기열 비어있음 - 충전소 복귀")
                    self.complete_robot_task(robot_name)  # 충전소로 복귀
            else:
                # NO 선택 시 재시도 (배달 작업은 계속되어야 함)
                self.command_log(f"❌ {robot.name} 배달 거부 - 다른 작업 처리 후 재시도", "WARNING")
                robot.state = RobotState.WAITING_FOR_CONFIRMATION
                
                # 현재 확인 요청을 큐 마지막에 다시 추가
                self.queue_confirmation_request(
                    robot_name=robot_name,
                    prompt="item_delivered",
                    retry_on_no=True
                )
                self.clear_confirmation_state()

        elif prompt == "stay_or_return":
            if confirmed:
                self.command_log(f"✅ {robot_name} 현재 위치에서 영구 대기")
                robot.is_permanently_stationed = True
                self.clear_confirmation_state()
                self.complete_robot_task_without_return(robot_name)
            else:
                self.command_log(f"🏠 {robot.name} 즉시 충전소로 복귀 시작")
                self.clear_confirmation_state()
                self.complete_robot_task(robot_name)

        elif prompt == "add_to_queue":
            if confirmed:
                # YES: 중단된 업무를 미할당 대기열에 가장 우선순위로 추가
                self.command_log(f"✅ {robot_name} 중단된 업무를 대기열 최우선으로 추가")
                if self.interrupted_task:
                    self.interrupted_task.robot_name = ""  # 미할당 상태로 변경
                    self.global_task_queue.insert(0, self.interrupted_task)  # 가장 우선순위로 추가
                    self.publish_task_queue_update()
            else:
                # NO: 중단된 업무를 완료 처리
                self.command_log(f"❌ {robot_name} 중단된 업무 완료 처리")
                if self.interrupted_task:
                    self.completed_tasks.append(self.interrupted_task)
            
            # 복귀 작업 실행 (YES/NO 상관없이 복귀는 진행)
            if self.pending_return_task and self.pending_return_robot:
                self.command_log(f"🏠 {robot_name} 복귀 작업 시작")
                self._execute_pending_return_task()
            
            self.clear_confirmation_state()
            return

    def clear_confirmation_state(self):
        # 🆘 확인창 응답 받음 - 추적 제거
        if self.confirmation_robot in self.confirmation_sent_times:
            del self.confirmation_sent_times[self.confirmation_robot]
            
        self.waiting_for_global_confirmation = False
        self.global_confirmation_prompt = ""
        self.confirmation_robot = ""
        self.interrupted_task = None
        self.interrupted_robot = ""
        self.pending_return_task = None
        self.pending_return_robot = ""
        self.current_confirmation = None
        
        # 다음 확인 요청 즉시 처리
        self.process_confirmation_queue()

    def process_confirmation_queue(self):
        """확인 요청 큐를 순차적으로 처리"""
        queue_size = self.confirmation_queue.qsize()
        
        # 현재 확인 진행 중이면 대기
        if self.waiting_for_global_confirmation:
            if queue_size > 0:
                self.system_log(f"⏳ 확인창 진행중 - 큐 대기: {queue_size}개")
            return
            
        # 현재 확인이 완료되었고 큐에 대기 중인 요청이 있으면 처리
        if not self.confirmation_queue.empty():
            next_request = self.confirmation_queue.get()
            self.system_log(f"🔄 큐에서 다음 확인창 처리: {next_request.robot_name} - {next_request.prompt} (남은 큐: {queue_size-1}개)")
            self._activate_confirmation_request(next_request)
        elif queue_size == 0:
            # 큐가 완전히 비어있음을 주기적으로 확인하지 않도록 (로그 스팸 방지)
            pass

    def _activate_confirmation_request(self, request: ConfirmationRequest):
        """확인 요청을 활성화"""
        self.waiting_for_global_confirmation = True
        self.global_confirmation_prompt = request.prompt
        self.confirmation_robot = request.robot_name
        self.interrupted_task = request.interrupted_task
        self.interrupted_robot = request.interrupted_robot
        self.pending_return_task = request.pending_return_task
        self.pending_return_robot = request.pending_return_robot
        self.current_confirmation = request
        
        # ROS 메시지 발행
        msg = Confirmation()
        msg.robot_name = request.robot_name
        msg.prompt = request.prompt
        msg.confirmed = False  # 요청임을 나타냄
        self.robot_manager.confirmation_request_pub.publish(msg)
        
        self.system_log(f"📋 확인 요청 활성화: {request.robot_name} - {request.prompt}")

    def queue_confirmation_request(self, robot_name: str, prompt: str, interrupted_task=None, interrupted_robot="", pending_return_task=None, pending_return_robot="", retry_on_no=False):
        """확인 요청을 큐에 추가"""
        request = ConfirmationRequest(
            robot_name=robot_name,
            prompt=prompt,
            interrupted_task=interrupted_task,
            interrupted_robot=interrupted_robot,
            pending_return_task=pending_return_task,
            pending_return_robot=pending_return_robot,
            retry_on_no=retry_on_no
        )
        
        self.confirmation_queue.put(request)
        queue_size = self.confirmation_queue.qsize()
        self.system_log(f"📝 확인 요청 대기열 추가: {robot_name} (대기: {queue_size}개)")

    def _execute_pending_return_task(self):
        """임시 저장된 복귀 작업을 실행"""
        if self.pending_return_task and self.pending_return_robot:
            task = self.pending_return_task
            robot_name = self.pending_return_robot
            robot = self.robot_manager.robots[robot_name]
            
            # 복귀 작업을 로봇에게 즉시 할당
            robot.current_task = task
            robot.task_step = 0
            robot.state = RobotState.RETURNING  # 복귀중 상태 (새 업무 배정 가능)
            
            self.command_log(f"🏠 {robot_name} → {task.destination} 복귀 시작")
            self.publish_task_queue_update()
            self._start_task_execution(robot_name, task)

    def _start_task_execution(self, robot_name: str, task):
        # 안전성 체크: task가 None이면 실행하지 않음
        if not task:
            self.command_log(f"❌ {robot_name} 작업이 None입니다. 실행 중단.", "ERROR")
            return
            
        robot = self.robot_manager.robots[robot_name]
        
        # 새로운 작업 시작 시 픽업대 해제 목록에서 제거 (재사용 가능하게 함)
        if robot_name in self.pickup_area_released_robots:
            self.pickup_area_released_robots.remove(robot_name)
        
        # 🚨 월급루팡 방지: 작업 시작 시간 기록
        import time
        self.robot_task_start_times[robot_name] = time.time()
        self.robot_last_move_times[robot_name] = time.time()
        
        # 복귀 작업인 경우 RETURNING 상태 유지, 아니면 WORKING 상태
        if task.type == TaskType.RETURN_TO_CHARGE:
            robot.state = RobotState.RETURNING  # 복귀중 (새 업무 배정 가능)
        else:
            robot.state = RobotState.WORKING
        
        # 작업 시작 시 GUI 업데이트
        self.publish_task_queue_update()
        
        if task.type == TaskType.DELIVERY:
            if robot.task_step == 0:
                robot.task_step = 1  # 픽업대로 이동 단계
            elif robot.task_step == 2:
                # 픽업 완료 후 배달지로 이동 시작
                self.command_log(f"🚛 {robot_name} {task.destination}로 배달 이동 시작")
                self.robot_manager.move_robot_to_location(robot_name, task.destination)
        elif task.type == TaskType.SIMPLE_MOVE:
            self.robot_manager.move_robot_to_location(robot_name, task.destination)
        elif task.type == TaskType.RETURN_TO_CHARGE:
            self.robot_manager.move_robot_to_location(robot_name, task.destination)

    def complete_robot_task(self, robot_name: str):
        robot = self.robot_manager.robots[robot_name]
        if robot.current_task:
            self.completed_tasks.append(robot.current_task)
            robot.current_task = None
            robot.task_step = 0
        
        # 🔧 메모리 누수 방지: 추적 정보 정리
        self._cleanup_robot_tracking_data(robot_name)
        
        # 작업 완료 후 GUI 업데이트
        self.publish_task_queue_update()
        
        charge_station = self.robot_manager.get_charge_station_for_robot(robot_name)
        return_task = Task(id=f"RETURN_{int(time.time())}", type=TaskType.RETURN_TO_CHARGE, destination=charge_station)
        robot.current_task = return_task
        robot.state = RobotState.RETURNING  # 복귀중 상태 (새 업무 배정 가능)
        self._start_task_execution(robot_name, return_task)

    def complete_robot_task_without_return(self, robot_name: str):
        robot = self.robot_manager.robots[robot_name]
        if robot.current_task:
            self.completed_tasks.append(robot.current_task)
            robot.current_task = None
            robot.task_step = 0
        robot.state = RobotState.IDLE
        
        # 작업 완료 후 GUI 업데이트
        self.publish_task_queue_update()

    def _execute_robot_return(self, robot_name: str):
        charge_station = self.robot_manager.get_charge_station_for_robot(robot_name)
        return_task = Task(id=f"RETURN_{int(time.time())}", type=TaskType.RETURN_TO_CHARGE, destination=charge_station, robot_name=robot_name)
        self.global_task_queue.insert(0, return_task)
        self.assign_next_task(robot_name)
        
    def _execute_robot_return_immediately(self, robot_name: str):
        """복귀 명령을 즉시 실행 (중단된 업무 처리 후)"""
        robot = self.robot_manager.robots[robot_name]
        charge_station = self.robot_manager.get_charge_station_for_robot(robot_name)
        
        # 복귀 작업 생성 및 즉시 할당
        return_task = Task(
            id=f"RETURN_{int(time.time())}", 
            type=TaskType.RETURN_TO_CHARGE, 
            destination=charge_station, 
            robot_name=robot_name
        )
        
        # 로봇에게 즉시 할당하여 실행
        robot.current_task = return_task
        robot.task_step = 0
        robot.state = RobotState.WORKING
        
        self.command_log(f"🏠 {robot_name} 즉시 {charge_station}로 복귀 시작")
        self.publish_task_queue_update()
        self._start_task_execution(robot_name, return_task)

    def sync_task_queue_from_data(self, queue_data: dict):
        try:
            new_queue = []
            for task_data in queue_data.get('tasks', []):
                task = Task(
                    id=task_data['id'],
                    type=TaskType(task_data['type']),
                    destination=task_data['destination'],
                    robot_name=task_data.get('robot_name', ''),
                    item=task_data.get('item', ''),
                    priority=Priority(task_data['priority']),
                    status=TaskStatus(task_data['status']),
                    created_time=task_data['created_time'],
                    estimated_duration=task_data['estimated_duration']
                )
                new_queue.append(task)
            self.global_task_queue = new_queue
        except Exception as e:
            self.system_log(f"❌ 작업 대기열 동기화 실패: {str(e)}", "ERROR")

    def publish_task_queue_update(self):
        if self.input_only:
            tasks_data = []
            for task in self.global_task_queue:
                task_dict = {
                    'id': task.id,
                    'type': task.type.value,
                    'destination': task.destination,
                    'robot_name': task.robot_name,
                    'item': task.item,
                    'priority': task.priority.value,
                    'status': task.status.value,
                    'created_time': task.created_time,
                    'estimated_duration': task.estimated_duration,
                    'is_charge_station_command': getattr(task, 'is_charge_station_command', False),
                    'is_return_command': getattr(task, 'is_return_command', False)
                }
                tasks_data.append(task_dict)
            
            queue_data = {'tasks': tasks_data}
            self.robot_manager.publish_task_queue(queue_data)
    
    def process_location_queues(self):
        if not self.robot_manager.location_manager.is_location_busy('픽업대'):
            next_robot_name = self.robot_manager.location_manager.get_next_in_queue('픽업대')
            if next_robot_name:
                robot = self.robot_manager.robots[next_robot_name]
                if robot.current_task and robot.task_step == 0.5:
                    self.command_log(f"📢 {next_robot_name} 픽업대 차례입니다! 이동하세요.")
                    robot.task_step = 1
                    self._start_task_execution(next_robot_name, robot.current_task)

    def suspend_robot_task(self, robot_name: str):
        robot = self.robot_manager.robots[robot_name]
        if not robot.current_task: return
        task = robot.current_task
        task.robot_name = ""
        self.suspended_tasks.append(task)
        robot.current_task = None
        robot.task_step = 0
        self.robot_manager.location_manager.leave_queue('픽업대', robot_name)

    def optimize_task_queue(self):
        if len(self.global_task_queue) > 1:
            self.global_task_queue.sort(key=lambda task: self.scheduler.calculate_priority_score(task), reverse=True)

    def clear_all_tasks(self):
        cleared_count = len(self.global_task_queue)
        self.global_task_queue.clear()
        self.command_log(f"🗑️ {cleared_count}개 작업이 취소되었습니다.")

    def show_pickup_status(self):
        """픽업대 관련 실시간 상태 표시 (로봇팔 연동용)"""
        pickup_bound_robots = []  # 픽업대로 향하는 로봇들
        waiting_robots = []       # 픽업대기장소에서 대기하는 로봇들
        pickup_active_robot = None  # 픽업대에서 작업 중인 로봇
        
        for name, robot in self.robot_manager.robots.items():
            if robot.current_task and robot.current_task.type == TaskType.DELIVERY:
                if robot.task_step == 1:  # 픽업대로 이동 중
                    distance_to_pickup = self.calculate_distance_to_pickup(name)
                    pickup_bound_robots.append((name, distance_to_pickup, robot.current_task.item))
                elif robot.task_step == 0.5:  # 픽업대기장소에서 대기
                    waiting_robots.append((name, robot.queue_position, robot.current_task.item))
                elif robot.task_step == 1.5:  # 픽업대에서 작업 중
                    pickup_active_robot = (name, robot.current_task.item)
        
        self.command_log("\n📦 실시간 픽업대 상태 (로봇팔 연동용)")
        
        if pickup_active_robot:
            name, item = pickup_active_robot
            self.command_log(f"  🎯 픽업 진행중: {name} - {item}")
        
        if pickup_bound_robots:
            # 거리순 정렬
            pickup_bound_robots.sort(key=lambda x: x[1])
            self.command_log(f"  🚀 픽업대로 향하는 로봇들:")
            for name, distance, item in pickup_bound_robots:
                eta = int(distance / 0.3)  # 0.3m/s 속도 가정
                self.command_log(f"    - {name}: {distance:.2f}m, ETA {eta}초, 물품: {item}")
                
                # 🔧 로봇팔 준비 판정 (거리 기반)
                if distance < 1.0:  # 1m 이내 접근시 로봇팔 준비 신호
                    self.command_log(f"    🦾 [로봇팔 준비 신호] {name}용 {item} 픽업 준비 시작!")
                    # 여기서 로봇팔에게 미리 준비 신호 보냄
                    self.arm_interface.prepare_pickup(name, item)  # 로봇팔 사전 준비
        
        if waiting_robots:
            self.command_log(f"  ⏳ 픽업대기장소 대기열:")
            for name, pos, item in waiting_robots:
                self.command_log(f"    - {name}: {pos+1}번째, 물품: {item}")

    def calculate_distance_to_pickup(self, robot_name: str) -> float:
        """로봇과 픽업대 간 거리 계산"""
        robot = self.robot_manager.robots[robot_name]
        if not robot.current_pose:
            return 999.0
        
        pickup_x, pickup_y = 0.15, -0.4  # 픽업대 좌표
        robot_x = robot.current_pose.position.x
        robot_y = robot.current_pose.position.y
        
        return math.sqrt((pickup_x - robot_x)**2 + (pickup_y - robot_y)**2)

    def show_status(self):
        self.command_log("\n📊 지능형 ROSA 상태")
        for name, robot in self.robot_manager.robots.items():
            self.command_log(f"  🤖 {name}: {robot.state.value} (배터리: {robot.battery_level:.1f}%) @ {self.robot_manager.get_current_location_name(name)}")
            if robot.current_task:
                self.command_log(f"     - 작업: {robot.current_task.id} ({robot.current_task.type.value}) → {robot.current_task.destination}")
        
        # 픽업대 상태도 함께 표시
        self.show_pickup_status()

    def show_queue(self):
        self.command_log("\n📝 작업 대기열")
        if not self.global_task_queue:
            self.command_log("대기 중인 작업이 없습니다.")
        for i, task in enumerate(self.global_task_queue, 1):
            self.command_log(f"  {i}. {task.id} ({task.robot_name or '미할당'}) → {task.destination}")

    def show_debug_info(self):
        self.command_log("\n🔍 상세 디버그 정보")
        for name, robot in self.robot_manager.robots.items():
            self.command_log(f"  🤖 {name}: pose_ok={robot.current_pose is not None}, moving={robot.is_moving}, task_step={robot.task_step}")

    def _start_pickup_release_timer(self, robot_name: str):
        """픽업대 해제 및 2초 후 다음 로봇 호출"""
        # 중복 처리 방지
        if robot_name in self.pickup_area_released_robots:
            return
            
        # 픽업대 즉시 해제
        self.robot_manager.location_manager.release_location('픽업대', robot_name)
        self.pickup_area_released_robots.add(robot_name)
        self.command_log(f"📦 {robot_name} 픽업대 해제됨")
        
        # 픽업 해제 타이머 시작 (threading.Timer 사용)
        if self.pickup_release_timer:
            # 기존 타이머 취소
            self.pickup_release_timer.cancel()
        
        # 2초 후 다음 로봇 호출
        self.pickup_release_timer = threading.Timer(self.pickup_release_delay, self._call_next_robot_to_pickup)
        self.pickup_release_timer.start()
        
        self.command_log(f"⏱️ 2초 후 다음 대기 로봇을 픽업대로 호출합니다")

    def _call_next_robot_to_pickup(self):
        """대기열의 다음 로봇을 픽업대로 호출"""
        next_robot_name = self.robot_manager.location_manager.get_next_in_queue('픽업대')
        if next_robot_name:
            robot = self.robot_manager.robots[next_robot_name]
            if robot.current_task and robot.task_step == 0.5:
                self.command_log(f"📢 {next_robot_name} 픽업대 차례입니다! 이동하세요.")
                robot.task_step = 1
                robot.state = RobotState.WORKING
                self.robot_manager.move_robot_to_location(next_robot_name, "픽업대")
        
        # 타이머 정리
        self.pickup_release_timer = None

    def _check_pickup_area_status(self):
        """픽업대 상태를 지속적으로 모니터링하여 로봇이 떠났는지 감지"""
        pickup_occupant = self.robot_manager.location_manager.location_occupancy.get('픽업대')
        
        if pickup_occupant:
            robot = self.robot_manager.robots[pickup_occupant]
            current_location = self.robot_manager.get_current_location_name(pickup_occupant)
            
            # 로봇이 배달중 상태가 되었는지 확인 (task_step이 2 이상)
            pickup_completed = (robot.current_task and robot.task_step >= 2)
            
            # 로봇이 물리적으로 픽업대를 벗어났는지 확인
            physically_left_pickup = (current_location != "픽업대" and 
                                    not current_location.startswith("(") and  # 좌표가 아닌 인식된 위치명인 경우만
                                    current_location != "알 수 없음")
            
            if pickup_completed:
                # 배달 시작 - 정상적인 픽업대 해제
                self.command_log(f"🚚 {pickup_occupant} 배달 시작 - 픽업대 해제 및 다음 로봇 호출")
                self._start_pickup_release_timer(pickup_occupant)
            elif physically_left_pickup:
                # 예상치 못한 픽업대 이탈 - 경고 후 해제
                self.command_log(f"⚠️ {pickup_occupant} 픽업대를 예기치 않게 벗어남 ({current_location}) - 해제", "WARNING")
                self._start_pickup_release_timer(pickup_occupant)