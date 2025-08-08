#!/usr/bin/env python3

from robot_manager import Task, TaskType, RobotState
from nav2_simple_commander.robot_navigator import TaskResult

class TaskExecutor:
    """실제 로봇의 작업 수행 로직을 담당하는 클래스"""
    def __init__(self, task_processor):
        self.task_processor = task_processor
        self.robot_manager = task_processor.robot_manager
        self.arm_interface = task_processor.arm_interface
        self.confirmation_manager = task_processor.confirmation_manager
        self.command_log = task_processor.command_log

    def process_robot_task(self, robot, task):
        """작업 유형에 따라 적절한 실행 함수를 호출"""
        if task.type == TaskType.DELIVERY:
            self._process_delivery_task(robot, task)
        elif task.type == TaskType.SIMPLE_MOVE:
            self._process_simple_move_task(robot, task)
        elif task.type == TaskType.RETURN_TO_CHARGE:
            self._process_return_task(robot, task)

    def _plan_path_sequence(self, robot_name, destination_name):
        return self.task_processor._plan_path_sequence(robot_name, destination_name)

    def _process_delivery_task(self, robot, task):
        navigator = self.robot_manager.navigators.get(robot.name)
        if not navigator:
            self.confirmation_manager.fail_robot_task(robot.name, "Navigator not available")
            return

        # 1단계: 픽업대로 이동 및 대기 처리
        if robot.task_step in [0.5, 1]:
            if robot.last_goal_time == 0:
                pickup_occupied = self.robot_manager.location_manager.is_location_busy('픽업대', robot.name)
                next_in_queue = self.robot_manager.location_manager.get_next_in_queue('픽업대')

                if pickup_occupied or (next_in_queue and next_in_queue != robot.name):
                    queue_pos = self.robot_manager.location_manager.join_queue('픽업대', robot.name)
                    robot.task_step = 0.5
                    robot.queue_position = queue_pos
                    robot.state = RobotState.WAITING_IN_QUEUE
                    self.command_log(f"📦 {robot.name} 픽업대 혼잡/순서 대기 -> 대기장소로 이동 ({queue_pos + 1}번째)")
                    self.robot_manager.move_robot_to_location(robot.name, "픽업대기장소")
                else:
                    robot.task_step = 1
                    self.command_log(f"📦 {robot.name} 픽업대로 이동 시작")
                    # 웨이포인트 경로 계산 비활성화 - 직접 이동
                    if self.robot_manager.move_robot_to_location(robot.name, "픽업대"):
                        robot.last_goal_time = self.task_processor.get_current_time()
                    else:
                        self.confirmation_manager.fail_robot_task(robot.name, "Path planning/start failed")
                return

            # goal_pose 토픽 사용 시 도착 여부는 위치 기반으로 확인
            if robot.task_step == 1 and self.robot_manager.robot_has_arrived_at(robot.name, "픽업대"):
                robot.last_goal_time = 0
                robot.task_step = 1.5
                self.robot_manager.location_manager.occupy_location('픽업대', robot.name)
                self.command_log(f"📦 {robot.name} 픽업대 도착 -> {task.item} 적재 대기")
                self.arm_interface.request_pickup(robot.name, task.item)
                self.confirmation_manager.send_confirmation_request_with_retry(robot.name, "item_loaded")

        # 2단계: 적재 대기
        elif robot.task_step == 1.5:
            pass

        # 3단계: 배달지로 이동
        elif robot.task_step == 2:
            if robot.last_goal_time == 0:
                self.command_log(f"🚚 {robot.name} -> {task.destination} 배달 시작")
                # 웨이포인트 경로 계산 비활성화 - 직접 이동  
                if self.robot_manager.move_robot_to_location(robot.name, task.destination):
                    robot.last_goal_time = self.task_processor.get_current_time()
                else:
                    self.confirmation_manager.fail_robot_task(robot.name, "Path planning/start failed")
                return

            # goal_pose 토픽 사용 시 도착 여부는 위치 기반으로 확인
            if self.robot_manager.robot_has_arrived_at(robot.name, task.destination):
                robot.last_goal_time = 0
                robot.task_step = 2.5
                self.command_log(f"📍 {robot.name} {task.destination} 도착 -> 전달 완료 확인 대기")
                self.confirmation_manager.send_confirmation_request_with_retry(robot.name, "item_delivered")
        
        # 4단계: 최종 전달 확인
        elif robot.task_step == 2.5:
            pass

    def _process_simple_move_task(self, robot, task):
        navigator = self.robot_manager.navigators.get(robot.name)
        if not navigator:
            self.confirmation_manager.fail_robot_task(robot.name, "Navigator not available")
            return

        if robot.last_goal_time == 0:
            self.command_log(f"🚶 {robot.name} -> {task.destination} 이동 시작")
            # 웨이포인트 경로 계산 비활성화
            if self.robot_manager.move_robot_to_location(robot.name, task.destination):
                robot.last_goal_time = self.task_processor.get_current_time()
            else:
                self.confirmation_manager.fail_robot_task(robot.name, "Path planning/start failed")
            return
        
        # goal_pose 토픽 사용 시 도착 여부는 위치 기반으로 확인
        if self.robot_manager.robot_has_arrived_at(robot.name, task.destination):
            if hasattr(task, 'is_charge_station_command') and task.is_charge_station_command:
                self.confirmation_manager.complete_robot_task_without_return(robot.name)
            else:
                self.confirmation_manager.send_confirmation_request(robot.name, "stay_or_return")

    def _process_return_task(self, robot, task):
        navigator = self.robot_manager.navigators.get(robot.name)
        if not navigator:
            self.confirmation_manager.fail_robot_task(robot.name, "Navigator not available")
            return
            
        if robot.last_goal_time == 0:
            self.command_log(f"🏠 {robot.name} -> {task.destination} 복귀 시작")
            # 웨이포인트 경로 계산 비활성화
            if self.robot_manager.move_robot_to_location(robot.name, task.destination):
                robot.last_goal_time = self.task_processor.get_current_time()
            else:
                self.confirmation_manager.fail_robot_task(robot.name, "Path planning/start failed")
            return

        # goal_pose 토픽 사용 시 도착 여부는 위치 기반으로 확인
        if self.robot_manager.robot_has_arrived_at(robot.name, task.destination):
            self.command_log(f"🏠 {robot.name} {task.destination} 복귀 완료")
            self.confirmation_manager.complete_robot_task_without_return(robot.name)