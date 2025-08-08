#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32, String
from pinky_interfaces.msg import RobotStatus, Confirmation
import time
import math
import yaml
from datetime import datetime
from enum import Enum
from dataclasses import dataclass
from typing import List, Optional, Dict, Tuple
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import os  # [추가] 파일 경로를 위해 import
from ament_index_python.packages import get_package_share_directory # [추가] 패키지 경로 검색

class TaskStatus(Enum):
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    WAITING_CONFIRMATION = "waiting_confirmation"
    COMPLETED = "completed"
    FAILED = "failed"

class TaskType(Enum):
    DELIVERY = "delivery"
    SIMPLE_MOVE = "move"
    RETURN_TO_CHARGE = "return_to_charge"  # 새로운 복귀 명령 타입

class RobotState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    WORKING = "working"
    CHARGING = "charging"
    LOW_BATTERY = "low_battery"
    WAITING_IN_QUEUE = "waiting_in_queue"
    RETURNING = "returning"  # 복귀 중 상태 추가
    WAITING_FOR_CONFIRMATION = "waiting_for_confirmation"  # 응답대기중 상태 추가

class Priority(Enum):
    LOW = 1
    NORMAL = 2
    HIGH = 3
    URGENT = 4

@dataclass
class Task:
    id: str
    type: TaskType
    destination: str
    robot_name: str = ""
    item: str = ""
    priority: Priority = Priority.NORMAL
    status: TaskStatus = TaskStatus.PENDING
    created_time: float = 0.0
    estimated_duration: float = 0.0
    deadline: Optional[float] = None
    is_charge_station_command: bool = False  # 충전소 명령 플래그
    is_return_command: bool = False  # 복귀 명령 플래그 추가

class RobotInfo:
    def __init__(self, name: str):
        self.name = name
        self.state = RobotState.IDLE
        self.battery_level = 100.0
        self.current_pose = None
        self.is_moving = False
        self.last_goal_time = 0
        self.last_goal_pos = None
        self.current_task: Optional[Task] = None
        self.task_step = 0
        self.waiting_for_confirmation = False
        self.confirmation_prompt = ""
        self.task_history: List[Task] = []
        self.average_speed = 0.3
        self.efficiency_score = 1.0
        self.last_pose_time = 0.0
        self.queue_position = -1
        self.is_permanently_stationed = False  # 영구 대기 상태 플래그

class LocationManager:
    def __init__(self):
        self.main_locations = {
            '왼쪽방': (0.0, 0.9),
            '오른쪽방': (0.4, 0.9),
            '픽업대': (0.15, -0.4),
            '3번 충전소': (-0.2, 0.0),
            '8번 충전소': (-0.2, 0.25),
            '9번 충전소': (-0.2, 0.5),
            '면회실': (0.5, 0.0),
            '출입구': (0.5, 0.2),
        }
        self.waiting_areas = {
            '픽업대기장소': (-0.2, -0.4),
        }
        self.all_locations = {**self.main_locations, **self.waiting_areas}
        self.location_queues = {
            '픽업대': [],
        }
        self.location_occupancy = {
            '픽업대': None,
        }
    
    def get_location_coordinates(self, location_name: str) -> Optional[Tuple[float, float]]:
        return self.all_locations.get(location_name)
    
    def is_location_busy(self, location_name: str, asking_robot: str = "") -> bool:
        occupant = self.location_occupancy.get(location_name)
        if occupant is None:
            return False
        if occupant == asking_robot:
            return False
        return True
    
    def get_queue_length(self, location_name: str) -> int:
        return len(self.location_queues.get(location_name, []))
    
    def join_queue(self, location_name: str, robot_name: str) -> int:
        if location_name not in self.location_queues:
            return -1
        if robot_name not in self.location_queues[location_name]:
            self.location_queues[location_name].append(robot_name)
        return self.location_queues[location_name].index(robot_name)
    
    def leave_queue(self, location_name: str, robot_name: str):
        if location_name in self.location_queues:
            if robot_name in self.location_queues[location_name]:
                self.location_queues[location_name].remove(robot_name)
    
    def get_next_in_queue(self, location_name: str) -> Optional[str]:
        queue_list = self.location_queues.get(location_name, [])
        return queue_list[0] if queue_list else None
    
    def occupy_location(self, location_name: str, robot_name: str):
        self.location_occupancy[location_name] = robot_name
        self.leave_queue(location_name, robot_name)
    
    def release_location(self, location_name: str, robot_name: str):
        if self.location_occupancy.get(location_name) == robot_name:
            self.location_occupancy[location_name] = None
    
    def get_waiting_position_for_queue(self, location_name: str, queue_position: int) -> Optional[Tuple[float, float]]:
        if location_name == '픽업대':
            base_x, base_y = self.waiting_areas['픽업대기장소']
            offset_y = queue_position * 0.5
            return (base_x, base_y - offset_y)
        return None

class ROSARobotManager(Node):
    """로봇 상태 관리 및 기본 ROS 통신"""
    
    def __init__(self):
        super().__init__('rosa_robot_manager')
        
        # 기본 설정
        self.robot_names = ['DP_03', 'DP_08', 'DP_09']
        self.battery_threshold = 40.0
        self.status_only = False
        self.input_only = False
        
        # 로봇 정보 및 위치 관리
        self.robots: Dict[str, RobotInfo] = {}
        for robot_name in self.robot_names:
            self.robots[robot_name] = RobotInfo(robot_name)
        
        self.location_manager = LocationManager()
        
        self.robot_charge_station_map = {
            'DP_03': '3번 충전소',
            'DP_08': '8번 충전소',
            'DP_09': '9번 충전소',
        }
        
        # 외부 참조
        self.task_processor = None
        self.gui = None
        
        # ROS 통신 설정
        self.status_pub = self.create_publisher(RobotStatus, '/rosa/robot_status', 10)
        
        # 확인 요청/응답 토픽 분리
        self.confirmation_request_pub = self.create_publisher(Confirmation, '/rosa/confirmation_request', 10)
        self.confirmation_response_pub = self.create_publisher(Confirmation, '/rosa/confirmation_response', 10)

        # GUI는 요청을 수신
        self.confirmation_request_sub = self.create_subscription(
            Confirmation, '/rosa/confirmation_request',
            self.confirmation_request_callback, 10
        )
        # TaskProcessor는 응답을 수신
        self.confirmation_response_sub = self.create_subscription(
            Confirmation, '/rosa/confirmation_response',
            self.confirmation_response_callback, 10
        )
        
        # 🆕 로그 토픽 추가 (프로세스 간 통신용)
        self.log_pub = self.create_publisher(String, '/rosa/log_messages', 10)
        self.log_sub = self.create_subscription(
            String, '/rosa/log_messages',
            self.log_callback, 10
        )
        
        # 🆕 작업 대기열 동기화 토픽 추가
        self.task_queue_pub = self.create_publisher(String, '/rosa/task_queue', 10)
        self.task_queue_sub = self.create_subscription(
            String, '/rosa/task_queue',
            self.task_queue_callback, 10
        )
        
        self.setup_robot_connections()
        
        # [추가] 웨이포인트 파일 로드 및 네비게이터 초기화
        self.waypoints = {}
        try:
            # 'pinky_navigation' 패키지의 공유 폴더 경로를 찾습니다.
            waypoint_file_path = '/home/addinedu/jeong/multi_robot_project/0808_system/waypoints.yaml'

            with open(waypoint_file_path, 'r') as f:
                self.waypoints = yaml.safe_load(f)

            self.log_message(f"✅ 웨이포인트 파일 로드 완료: {waypoint_file_path}", "SUCCESS", is_system=True)

        except Exception as e:
            self.log_message(f"❌ 웨이포인트 파일 로드 실패: {e}", "ERROR", is_system=True)
            self.log_message(f"💡 [해결 방법] 'pinky_navigation' 패키지 안의 'params' 폴더에 'waypoints.yaml' 파일이 있는지 확인하세요.", "INFO", is_system=True)

        self.navigators: Dict[str, BasicNavigator] = {}
        for robot_name in self.robot_names:
            # 네임스페이스를 지정하여 각 로봇의 네비게이터를 생성
            self.navigators[robot_name] = BasicNavigator(namespace=f'/{robot_name}')
        
        # 타이머 설정
        self.create_timer(2.0, self.check_robot_status)
        self.create_timer(30.0, self.debug_robot_status)
        self.create_timer(1.0, self.publish_robot_status)
        
        self.log_message("🤖 ROSA 로봇 관리자 시작!", is_system=True)
        self.log_message("🔗 로봇 연결 및 상태 모니터링 활성화", is_system=True)
    
    # [신규] follow_waypoints 함수
    def follow_waypoints(self, robot_name: str, path_name: str) -> bool:
        """지정된 웨이포인트 경로를 따라가도록 명령합니다."""
        if path_name not in self.waypoints:
            self.log_message(f"❌ 알 수 없는 웨이포인트 경로: {path_name}", "ERROR", is_system=True)
            return False

        robot = self.robots[robot_name]
        navigator = self.navigators[robot_name]
        
        # 이미 다른 작업을 하고 있다면 중단 (필요 시)
        if not navigator.isTaskComplete():
            navigator.cancelTask()

        goal_poses = []
        for point in self.waypoints[path_name]:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(point['pose']['position']['x'])
            pose.pose.position.y = float(point['pose']['position']['y'])
            pose.pose.orientation.w = 1.0
            goal_poses.append(pose)

        if not goal_poses:
            self.log_message(f"❌ 웨이포인트 경로 '{path_name}'에 지점이 없습니다.", "ERROR", is_system=True)
            return False

        self.log_message(f"▶️ {robot_name}가 경로 '{path_name}' 주행을 시작합니다.", is_system=True)
        robot.state = RobotState.MOVING
        navigator.followWaypoints(goal_poses)
        
        return True

    def set_task_processor(self, task_processor):
        """작업 처리기 참조 설정"""
        self.task_processor = task_processor
    
    def set_gui(self, gui):
        """GUI 참조 설정"""
        self.gui = gui
    
    def log_callback(self, msg: String):
        """다른 프로세스에서 온 로그 메시지 수신"""
        if self.status_only:
            # 메시지 형식: "LEVEL|LOG_TYPE|MESSAGE"
            parts = msg.data.split('|', 2)
            if len(parts) == 3:
                level, log_type, message = parts
                if log_type == "COMMAND":
                    # 명령 결과는 터미널에 출력
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] {message}")
                elif log_type == "SYSTEM" and self.gui:
                    # 시스템 로그는 GUI에 출력 (필터링 포함)
                    self.gui.add_log(message, level)
            else:
                # 호환성을 위한 기본 처리
                if self.gui:
                    self.gui.add_log(msg.data, "INFO")
    
    def task_queue_callback(self, msg: String):
        """다른 프로세스에서 온 작업 대기열 정보 수신"""
        if self.status_only and self.task_processor:
            # JSON 형식으로 작업 대기열 정보를 받아서 동기화
            import json
            try:
                queue_data = json.loads(msg.data)
                # task_processor의 global_task_queue를 업데이트
                self.task_processor.sync_task_queue_from_data(queue_data)
            except Exception as e:
                self.log_message(f"❌ 작업 대기열 동기화 오류: {str(e)}", "ERROR", is_system=True)
    
    def publish_log_message(self, message: str, level: str = "INFO", log_type: str = "SYSTEM"):
        """로그 메시지를 ROS 토픽으로 발행"""
        if self.input_only:  # input-only 모드에서만 다른 프로세스로 전송
            msg = String()
            msg.data = f"{level}|{log_type}|{message}"
            self.log_pub.publish(msg)
    
    def publish_task_queue(self, queue_data: dict):
        """작업 대기열 정보를 ROS 토픽으로 발행"""
        if self.input_only:  # input-only 모드에서만 다른 프로세스로 전송
            import json
            msg = String()
            msg.data = json.dumps(queue_data, ensure_ascii=False)
            self.task_queue_pub.publish(msg)
    
    def log_message(self, message: str, level: str = "INFO", is_system: bool = True):
        """로그 메시지 출력 (시스템 로그인지 구분)"""
        if self.status_only and self.gui and is_system:
            # status-only 모드: 시스템 로그만 GUI로 (필터링 포함)
            self.gui.add_log(message, level)
        elif self.input_only:
            # input-only 모드: 로컬 출력 안하고 ROS 토픽으로만 전송
            log_type = "SYSTEM" if is_system else "COMMAND"
            self.publish_log_message(message, level, log_type)
        else:
            # 통합 모드: 로컬 터미널 출력
            print(f"[{datetime.now().strftime('%H:%M:%S')}] {message}")
    
    def setup_robot_connections(self):
        """로봇 ROS 토픽 연결"""
        self.goal_pubs = {}
        self.cmd_vel_subs = {}
        self.pose_subs = {}
        self.battery_subs = {}
        
        for robot_name in self.robot_names:
            self.log_message(f"🔗 {robot_name} 연결 설정 중...", is_system=True)
            
            self.goal_pubs[robot_name] = self.create_publisher(
                PoseStamped, f'{robot_name}/goal_pose', 10
            )
            
            self.cmd_vel_subs[robot_name] = self.create_subscription(
                Twist, f'{robot_name}/cmd_vel', 
                lambda msg, name=robot_name: self.cmd_vel_callback(msg, name), 10
            )
            
            self.pose_subs[robot_name] = self.create_subscription(
                PoseWithCovarianceStamped, f'{robot_name}/amcl_pose',
                lambda msg, name=robot_name: self.pose_callback(msg, name), 10
            )
            
            self.battery_subs[robot_name] = self.create_subscription(
                Float32, f'{robot_name}/battery_present',
                lambda msg, name=robot_name: self.battery_callback(msg, name), 10
            )
            
            self.log_message(f"✅ {robot_name} 토픽 구독 완료", "SUCCESS", is_system=True)
    
    def battery_callback(self, msg, robot_name):
        """배터리 콜백"""
        robot = self.robots[robot_name]
        robot.battery_level = msg.data
        
        if robot.battery_level < self.battery_threshold:
            if robot.state != RobotState.LOW_BATTERY:
                robot.state = RobotState.LOW_BATTERY
                self.log_message(f"🔋 {robot_name} 배터리 부족! ({robot.battery_level:.1f}%) 업무 불가능", "WARNING", is_system=True)
                
                self.location_manager.leave_queue('픽업대', robot_name)
                robot.queue_position = -1
                
                if robot.current_task and self.task_processor:
                    self.log_message(f"⚠️ {robot_name} 현재 작업({robot.current_task.id}) 일시 중단", "WARNING", is_system=True)
                    self.task_processor.suspend_robot_task(robot_name)
                    
        elif robot.state == RobotState.LOW_BATTERY and robot.battery_level >= self.battery_threshold + 5:
            robot.state = RobotState.IDLE
            self.log_message(f"🔋 {robot_name} 배터리 회복! ({robot.battery_level:.1f}%) 업무 가능", "SUCCESS", is_system=True)
    
    def cmd_vel_callback(self, msg, robot_name):
        """속도 명령 콜백"""
        robot = self.robots[robot_name]
        linear_speed = abs(msg.linear.x)
        angular_speed = abs(msg.angular.z)
        
        if linear_speed > 0.01 or angular_speed > 0.01:
            robot.is_moving = True
            if robot.state in [RobotState.IDLE, RobotState.WAITING_IN_QUEUE]:
                robot.state = RobotState.MOVING
        else:
            if time.time() - robot.last_goal_time > 3.0:
                robot.is_moving = False
                # 로봇이 멈췄을 때 상태 업데이트
                if robot.state == RobotState.MOVING and not robot.current_task:
                    robot.state = RobotState.IDLE
    
    def pose_callback(self, msg, robot_name):
        """위치 콜백"""
        robot = self.robots[robot_name]
        old_pose_exists = robot.current_pose is not None
        
        robot.current_pose = msg.pose.pose
        robot.last_pose_time = time.time()
        
        if not old_pose_exists:
            x = robot.current_pose.position.x
            y = robot.current_pose.position.y
            self.log_message(f"📍 {robot_name} 연결됨: ({x:.3f}, {y:.3f})", "SUCCESS", is_system=True)

    def confirmation_request_callback(self, msg: Confirmation):
        """TaskProcessor에서 보낸 확인 요청을 GUI로 전달"""
        self.log_message(f"🔍 확인 요청 수신: {msg.robot_name} - {msg.prompt} (GUI 연결: {self.gui is not None})", "INFO", is_system=True)
        if self.gui:
            self.gui.show_confirmation_request(msg.robot_name, msg.prompt)
            self.log_message(f"✅ GUI에 확인 요청 전달 완료: {msg.robot_name} - {msg.prompt}", "INFO", is_system=True)
        else:
            self.log_message(f"❌ GUI가 연결되지 않아 확인 요청을 표시할 수 없습니다", "ERROR", is_system=True)

    def confirmation_response_callback(self, msg: Confirmation):
        """GUI에서 보낸 확인 응답을 ConfirmationManager로 전달"""
        if self.task_processor:
            self.task_processor.confirmation_manager.handle_confirmation_from_ros(msg)
    
    def move_robot_to_location(self, robot_name: str, location: str):
        """로봇을 지정 위치로 이동"""
        coordinates = self.location_manager.get_location_coordinates(location)
        if not coordinates:
            self.log_message(f"❌ 알 수 없는 위치: {location}", "ERROR", is_system=True)
            return False
        
        return self.move_robot_to_coordinates(robot_name, coordinates, location)
    
    def move_robot_to_coordinates(self, robot_name: str, coordinates: Tuple[float, float], location_name: str):
        """로봇을 지정 좌표로 이동"""
        robot = self.robots[robot_name]
        x, y = coordinates
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        
        self.goal_pubs[robot_name].publish(goal_pose)
        robot.last_goal_time = time.time()
        robot.last_goal_pos = (x, y)
        
        # 복귀 명령인지 확인해서 상태 설정
        if robot.current_task and hasattr(robot.current_task, 'is_return_command') and robot.current_task.is_return_command:
            robot.state = RobotState.RETURNING
        else:
            robot.state = RobotState.MOVING
        
        self.log_message(f"🎯 {robot_name} → {location_name} ({x:.3f}, {y:.3f})", is_system=True)
        return True
    
    def robot_has_arrived_at(self, robot_name: str, location: str) -> bool:
        """로봇이 지정 위치에 도착했는지 확인"""
        robot = self.robots[robot_name]
        coordinates = self.location_manager.get_location_coordinates(location)
        
        if not robot.current_pose or not coordinates:
            return False
        
        target_x, target_y = coordinates
        current_x = robot.current_pose.position.x
        current_y = robot.current_pose.position.y
        
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        # 디버그: 도착 조건 체크
        if distance < 0.25:  # 0.15m → 0.25m로 완화 (임시)
            self.log_message(f"🎯 {robot_name} → {location}: 거리 {distance:.3f}m, 이동중: {robot.is_moving}", "INFO", is_system=True)
        
        if distance < 0.25 and not robot.is_moving:  # 0.15m → 0.25m로 완화
            self.log_message(f"📍 {robot_name} → {location} 도착", "SUCCESS", is_system=True)
            robot.last_goal_time = 0
            return True
        
        return False
    
    def get_current_location_name(self, robot_name: str) -> str:
        """현재 위치 이름 반환"""
        robot = self.robots[robot_name]
        if not robot.current_pose:
            return "알 수 없음"
        
        x = robot.current_pose.position.x
        y = robot.current_pose.position.y
        
        closest_location = None
        min_distance = float('inf')
        
        for name, (lx, ly) in self.location_manager.all_locations.items():
            distance = math.sqrt((x - lx)**2 + (y - ly)**2)
            if distance < min_distance:
                min_distance = distance
                closest_location = name
        
        if min_distance < 0.15:  # 0.3m → 0.15m로 더 정확하게
            return f"{closest_location}"
        else:
            return f"({x:.2f}, {y:.2f})"
    
    def get_detailed_progress_description(self, robot: RobotInfo) -> str:
        """상세한 진행 상황 설명 반환 (GUI 상태 테이블용)"""
        if robot.current_task:
            task = robot.current_task
            if task.type == TaskType.DELIVERY:
                if robot.task_step == 0.5:
                    if robot.queue_position >= 0:
                        return f"픽업대에서 대기중 ({robot.queue_position + 1}번째)"
                    else:
                        return "픽업대에서 대기중"
                elif robot.task_step == 1:
                    return "픽업대로 이동중"
                elif robot.task_step == 1.5:
                    return f"픽업대에서 {task.item} 적재 대기중"
                elif robot.task_step == 2:
                    return f"{task.destination}에 {task.item} 배달중"
                elif robot.task_step == 2.5:
                    return f"{task.destination}에서 {task.item} 전달 완료 대기중"
                else:
                    return f"{task.destination}에 {task.item} 배달중"
            elif task.type == TaskType.RETURN_TO_CHARGE:
                if robot.is_moving:
                    return f"{task.destination}로 복귀중"
                else:
                    return f"{task.destination}에서 대기중 (업무 할당 가능)"
            else:
                # SIMPLE_MOVE
                if robot.is_moving:
                    return f"{task.destination}로 이동중"
                else:
                    # 충전소 명령인지 확인
                    if hasattr(task, 'is_charge_station_command') and task.is_charge_station_command:
                        return f"{task.destination}에서 대기중"
                    elif robot.is_permanently_stationed:
                        return f"{task.destination}에서 대기중 (고정 배치)"
                    else:
                        return f"{task.destination}에서 대기중"
        elif robot.state == RobotState.LOW_BATTERY:
            return "배터리 부족 - 충전소에서 대기중"
        elif robot.state == RobotState.RETURNING:
            charge_station = self.get_charge_station_for_robot(robot.name)
            return f"{charge_station}로 복귀중 (업무 할당 가능)"
        elif robot.is_moving and robot.state == RobotState.MOVING:
            charge_station = self.get_charge_station_for_robot(robot.name)
            return f"{charge_station}로 귀환중"
        elif robot.state == RobotState.CHARGING:
            return "충전소에서 대기중"
        else:
            return "대기중"
    
    def get_charge_station_for_robot(self, robot_name: str) -> str:
        """로봇에 할당된 충전소 반환"""
        return self.robot_charge_station_map.get(robot_name, '3번 충전소')
    
    def check_robot_status(self):
        """로봇 상태 체크"""
        for robot_name, navigator in self.navigators.items():
            robot = self.robots[robot_name]
            if robot.state == RobotState.MOVING and navigator.isTaskComplete():
                self.log_message(f"✅ {robot_name} 경로 주행 완료.", "SUCCESS", is_system=True)
                robot.state = RobotState.IDLE
                robot.last_goal_time = 0
    
    def debug_robot_status(self):
        """주기적 로봇 상태 체크"""
        current_time = time.time()
        
        for robot_name, robot in self.robots.items():
            time_since_pose = current_time - robot.last_pose_time if robot.last_pose_time > 0 else 999
            if robot.current_pose and time_since_pose > 15.0:  # 10초 → 15초로 완화
                if not hasattr(robot, 'connection_warning_shown'):
                    self.log_message(f"⚠️ {robot_name} 위치 데이터 연결 불안정 (15초 이상 미수신)", "WARNING", is_system=True)
                    robot.connection_warning_shown = True
            elif robot.current_pose and time_since_pose < 8.0:  # 5초 → 8초로 완화
                if hasattr(robot, 'connection_warning_shown'):
                    self.log_message(f"✅ {robot_name} 위치 데이터 연결 복구됨", "SUCCESS", is_system=True)
                    delattr(robot, 'connection_warning_shown')
    
    def publish_robot_status(self):
        """로봇 상태 발행"""
        for robot_name, robot in self.robots.items():
            msg = RobotStatus()
            msg.robot_name = robot_name
            msg.state = robot.state.value
            msg.progress = self.get_detailed_progress_description(robot)
            msg.position_x = robot.current_pose.position.x if robot.current_pose else 0.0
            msg.position_y = robot.current_pose.position.y if robot.current_pose else 0.0
            self.status_pub.publish(msg)