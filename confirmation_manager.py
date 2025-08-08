#!/usr/bin/env python3

import time
import queue
from dataclasses import dataclass
from typing import Optional
from robot_manager import Task, TaskType, TaskStatus, RobotState
from pinky_interfaces.msg import Confirmation

@dataclass
class ConfirmationRequest:
    robot_name: str
    prompt: str
    interrupted_task: Optional[Task] = None
    pending_return_task: Optional[Task] = None
    retry_on_no: bool = False

class ConfirmationManager:
    """사용자 확인 및 작업 완료/실패 처리를 전담하는 클래스"""
    def __init__(self, task_processor):
        self.task_processor = task_processor
        self.robot_manager = task_processor.robot_manager
        self.command_log = task_processor.command_log
        self.system_log = task_processor.system_log

        # 확인 관련 상태 변수들
        self.waiting_for_global_confirmation = False
        self.global_confirmation_prompt = ""
        self.confirmation_robot = ""
        self.interrupted_task = None
        self.interrupted_robot = ""
        self.pending_return_task = None
        self.pending_return_robot = ""
        self.confirmation_queue = queue.Queue()
        self.confirmation_sent_times = {}
        self.current_confirmation = None
    
    def _confirm_pickup_and_proceed(self, robot_name: str):
        if not self.waiting_for_global_confirmation and self.confirmation_robot != robot_name: return
        robot = self.robot_manager.robots.get(robot_name)
        if not robot or not robot.current_task or robot.task_step != 1.5: return

        self.command_log(f"👍 {robot_name} 픽업 확인 -> 배달 시작")
        self.robot_manager.location_manager.release_location('픽업대', robot_name)
        robot.task_step = 2
        self.clear_confirmation_state()
        self.task_processor._start_task_execution(robot_name, robot.current_task)

    def handle_arm_pickup_completion(self):
        waiting_robot_name = next((name for name, r in self.robot_manager.robots.items() if r.current_task and r.task_step == 1.5), None)
        if waiting_robot_name:
            self.system_log(f"🦾 {waiting_robot_name} 로봇팔 완료 신호 수신")
            self._confirm_pickup_and_proceed(waiting_robot_name)

    def fail_robot_task(self, robot_name: str, reason: str):
        robot = self.robot_manager.robots.get(robot_name)
        if not robot or not robot.current_task: return
        task = robot.current_task
        self.command_log(f"❌ 작업 실패 (ID: {task.id}): {reason}", "ERROR")
        task.status = TaskStatus.FAILED
        self.task_processor.completed_tasks.append(task)
        if task.type == TaskType.DELIVERY:
            self.robot_manager.location_manager.release_location('픽업대', robot_name)
        self.clear_confirmation_state()
        self.complete_robot_task(robot_name)

    def send_confirmation_request(self, robot_name: str, prompt: str):
        if self.waiting_for_global_confirmation:
            self.queue_confirmation_request(robot_name=robot_name, prompt=prompt)
        else:
            self._activate_confirmation_request(ConfirmationRequest(robot_name=robot_name, prompt=prompt))

    def send_confirmation_request_with_retry(self, robot_name: str, prompt: str):
        req = ConfirmationRequest(robot_name=robot_name, prompt=prompt, retry_on_no=True)
        if self.waiting_for_global_confirmation:
            self.queue_confirmation_request(**req.__dict__)
        else:
            self._activate_confirmation_request(req)

    def handle_confirmation_from_ros(self, msg: Confirmation):
        if (self.waiting_for_global_confirmation and msg.robot_name == self.confirmation_robot and msg.prompt == self.global_confirmation_prompt):
            self.system_log(f"📨 GUI 응답 수신: {msg.robot_name} - {'YES' if msg.confirmed else 'NO'}")
            self.handle_confirmation('yes' if msg.confirmed else 'no')

    def handle_confirmation(self, response: str):
        robot_name = self.confirmation_robot
        prompt = self.global_confirmation_prompt
        if not robot_name or robot_name not in self.robot_manager.robots:
            self.clear_confirmation_state()
            return
            
        robot = self.robot_manager.robots[robot_name]
        confirmed = response.lower() in ['yes', 'y']

        if prompt in ["item_loaded", "manual_pickup"]:
            if confirmed: self._confirm_pickup_and_proceed(robot_name)
            else: self.fail_robot_task(robot_name, "User rejected pickup")
        elif prompt == "item_delivered":
            if confirmed:
                self.command_log(f"✅ {robot_name} 배달 완료")
                self.clear_confirmation_state()
                if self.task_processor.global_task_queue:
                    self.complete_robot_task_without_return(robot_name)
                else:
                    self.complete_robot_task(robot_name)
            else: self.fail_robot_task(robot_name, "User rejected delivery")
        elif prompt == "stay_or_return":
            self.clear_confirmation_state()
            if confirmed:
                robot.is_permanently_stationed = True
                self.complete_robot_task_without_return(robot_name)
            else: self.complete_robot_task(robot_name)
        elif prompt == "add_to_queue":
            if confirmed and self.interrupted_task:
                self.interrupted_task.robot_name = ""
                self.task_processor.global_task_queue.insert(0, self.interrupted_task)
            if self.pending_return_task:
                self._execute_pending_return_task()
            self.clear_confirmation_state()

    def clear_confirmation_state(self):
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
        time.sleep(0.5)
        self.process_confirmation_queue()

    def process_confirmation_queue(self):
        if self.waiting_for_global_confirmation: return
        if not self.confirmation_queue.empty():
            self._activate_confirmation_request(self.confirmation_queue.get())

    def _activate_confirmation_request(self, request: ConfirmationRequest):
        self.waiting_for_global_confirmation = True
        self.global_confirmation_prompt = request.prompt
        self.confirmation_robot = request.robot_name
        self.interrupted_task = request.interrupted_task
        self.interrupted_robot = getattr(request, 'interrupted_robot', "") # interrupted_robot 추가
        self.pending_return_task = request.pending_return_task
        self.pending_return_robot = getattr(request, 'pending_return_robot', "") # pending_return_robot 추가
        self.current_confirmation = request
        self.robot_manager.confirmation_request_pub.publish(
            Confirmation(robot_name=request.robot_name, prompt=request.prompt))

    def queue_confirmation_request(self, **kwargs):
        self.confirmation_queue.put(ConfirmationRequest(**kwargs))
    
    def complete_robot_task(self, robot_name: str):
        robot = self.robot_manager.robots[robot_name]
        if robot.current_task: self.task_processor.completed_tasks.append(robot.current_task)
        robot.current_task = None
        self.task_processor._cleanup_robot_tracking_data(robot_name)
        
        charge_station = self.robot_manager.get_charge_station_for_robot(robot_name)
        return_task = Task(id=f"RETURN_{int(time.time())}", type=TaskType.RETURN_TO_CHARGE, destination=charge_station)
        robot.current_task = return_task
        self.task_processor._start_task_execution(robot_name, return_task)

    def complete_robot_task_without_return(self, robot_name: str):
        robot = self.robot_manager.robots[robot_name]
        if robot.current_task: self.task_processor.completed_tasks.append(robot.current_task)
        robot.current_task = None
        robot.state = RobotState.IDLE
        self.task_processor._cleanup_robot_tracking_data(robot_name)

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
            self.task_processor.publish_task_queue_update()
            self.task_processor._start_task_execution(robot_name, task)