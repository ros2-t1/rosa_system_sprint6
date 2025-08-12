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
        try:
            if path_name not in self.waypoints:
                self.log_message(f"❌ 알 수 없는 웨이포인트 경로: {path_name}", "ERROR", is_system=True)
                self.log_message(f"💡 사용 가능한 경로: {list(self.waypoints.keys())}", "INFO", is_system=True)
                return False

            if robot_name not in self.robots:
                self.log_message(f"❌ 알 수 없는 로봇: {robot_name}", "ERROR", is_system=True)
                return False

            robot = self.robots[robot_name]
            navigator = self.navigators[robot_name]
            
            # 네비게이터가 준비되었는지 확인
            if not navigator:
                self.log_message(f"❌ {robot_name} 네비게이터가 초기화되지 않았습니다.", "ERROR", is_system=True)
                return False
            
            # 이미 다른 작업을 하고 있다면 중단 후 잠시 대기
            try:
                if not navigator.isTaskComplete():
                    self.log_message(f"⚠️ {robot_name}의 진행 중인 작업을 취소합니다.", "INFO", is_system=True)
                    navigator.cancelTask()
                    
                    # 취소 처리 완료까지 잠시 대기
                    import time
                    time.sleep(1.0)
                    
                    # 강제로 상태 리셋
                    self.log_message(f"🔄 {robot_name} navigator 상태 리셋", "INFO", is_system=True)
                    
            except Exception as e:
                self.log_message(f"⚠️ 작업 취소 중 오류 (무시): {e}", "WARNING", is_system=True)

            goal_poses = []
            for i, point in enumerate(self.waypoints[path_name]):
                try:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = float(point['pose']['position']['x'])
                    pose.pose.position.y = float(point['pose']['position']['y'])
                    pose.pose.position.z = float(point['pose']['position'].get('z', 0.0))
                    
                    # orientation 처리
                    orientation = point['pose']['orientation']
                    pose.pose.orientation.x = float(orientation.get('x', 0.0))
                    pose.pose.orientation.y = float(orientation.get('y', 0.0))
                    pose.pose.orientation.z = float(orientation.get('z', 0.0))
                    pose.pose.orientation.w = float(orientation.get('w', 1.0))
                    
                    goal_poses.append(pose)
                    self.log_message(f"📍 웨이포인트 {i+1}: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})", "INFO", is_system=True)
                except Exception as e:
                    self.log_message(f"❌ 웨이포인트 {i+1} 파싱 오류: {e}", "ERROR", is_system=True)
                    return False

            if not goal_poses:
                self.log_message(f"❌ 웨이포인트 경로 '{path_name}'에 지점이 없습니다.", "ERROR", is_system=True)
                return False

            self.log_message(f"▶️ {robot_name}가 경로 '{path_name}' 주행을 시작합니다. ({len(goal_poses)}개 지점)", "SUCCESS", is_system=True)
            robot.state = RobotState.MOVING
            
            # followWaypoints 호출 및 오류 처리
            try:
                navigator.followWaypoints(goal_poses)
                self.log_message(f"✅ {robot_name} waypoint 명령 전송 완료", "SUCCESS", is_system=True)
                return True
            except Exception as e:
                self.log_message(f"❌ {robot_name} waypoint 명령 전송 실패: {e}", "ERROR", is_system=True)
                robot.state = RobotState.IDLE
                return False
                
        except Exception as e:
            self.log_message(f"❌ follow_waypoints 전체 오류: {e}", "ERROR", is_system=True)
            return False

    def plan_path_with_waypoints(self, robot_name: str, destination_name: str) -> bool:
        """A* path planning using waypoints as intermediate routes"""
        try:
            if robot_name not in self.robots:
                self.log_message(f"❌ 알 수 없는 로봇: {robot_name}", "ERROR", is_system=True)
                return False
                
            # 목적지 찾기
            destination = None
            for dest in self.waypoints.get('destinations', []):
                if dest['name'] == destination_name:
                    destination = dest
                    break
                    
            if not destination:
                self.log_message(f"❌ 알 수 없는 목적지: {destination_name}", "ERROR", is_system=True)
                return False
            
            robot = self.robots[robot_name]
            current_pos = robot.position
            dest_pos = destination['pose']['position']
            
            self.log_message(f"🗺️ {robot_name}: 현재위치 ({current_pos.x:.2f}, {current_pos.y:.2f}) -> {destination_name} ({dest_pos['x']:.2f}, {dest_pos['y']:.2f})", "INFO", is_system=True)
            
            # 상행선/하행선 선택 로직
            highway_path = self._select_highway_path(current_pos, dest_pos)
            
            if not highway_path:
                self.log_message(f"⚠️ 직접 경로로 이동: {destination_name}", "INFO", is_system=True)
                coordinates = self.location_manager.get_location_coordinates(destination_name)
                if coordinates:
                    return self.move_robot_to_coordinates(robot_name, coordinates, destination_name)
                else:
                    self.log_message(f"❌ 직접 이동 실패: 알 수 없는 위치 {destination_name}", "ERROR", is_system=True)
                    return False
            
            # A* 경로 생성: 현재위치 -> highway waypoints -> 목적지
            full_path = self._create_full_path(current_pos, highway_path, dest_pos)
            
            if not full_path:
                self.log_message(f"❌ 경로 생성 실패", "ERROR", is_system=True)
                return False
                
            self.log_message(f"✅ A* 경로 계획 완료: {len(full_path)}개 지점", "SUCCESS", is_system=True)
            
            # 경로 실행
            return self._execute_planned_path(robot_name, full_path, destination_name)
            
        except Exception as e:
            self.log_message(f"❌ path planning 오류: {e}", "ERROR", is_system=True)
            return False
    
    def _select_highway_path(self, current_pos, dest_pos):
        """현재 위치와 목적지를 기반으로 상행선/하행선 선택"""
        try:
            current_y = current_pos.y
            dest_y = dest_pos['y']
            
            self.log_message(f"🗺️ 경로 선택: 현재 Y={current_y:.2f}, 목적지 Y={dest_y:.2f}", "INFO", is_system=True)
            
            # Highway waypoint의 Y 범위 확인
            # highway_up: -0.8 ~ 0.9
            # highway_down: -0.8 ~ 0.9  
            highway_min_y = -0.8
            highway_max_y = 0.9
            
            # 목적지가 highway 범위를 벗어나면 직접 이동
            if dest_y > highway_max_y + 0.1 or dest_y < highway_min_y - 0.1:
                self.log_message(f"➡️ 직접 경로 (highway 범위 밖: {dest_y:.2f})", "INFO", is_system=True)
                return None
            
            # 현재 위치가 highway 범위를 벗어나면 직접 이동
            if current_y > highway_max_y + 0.1 or current_y < highway_min_y - 0.1:
                self.log_message(f"➡️ 직접 경로 (현재 위치가 highway 범위 밖: {current_y:.2f})", "INFO", is_system=True)
                return None
            
            # Y 좌표 기준으로 상행선/하행선 결정 (임계값 낮춤)
            y_diff = dest_y - current_y
            if y_diff > 0.05:  # 위쪽으로 가는 경우 (임계값 0.1 -> 0.05)
                self.log_message(f"🟢 상행선 경로 선택 (위로 {y_diff:.2f}m)", "SUCCESS", is_system=True)
                return 'highway_up'
            elif y_diff < -0.05:  # 아래쪽으로 가는 경우 (임계값 0.1 -> 0.05)
                self.log_message(f"🔵 하행선 경로 선택 (아래로 {y_diff:.2f}m)", "SUCCESS", is_system=True)
                return 'highway_down'
            else:  # 매우 비슷한 Y 좌표라면 X 좌표도 고려
                current_x = current_pos.x
                dest_x = dest_pos['x']
                
                # X 좌표 차이가 크면 highway 사용
                x_diff = abs(dest_x - current_x)
                if x_diff > 0.3:  # X 거리가 0.3m 이상이면 highway 사용
                    if current_y < 0:  # 현재가 아래쪽이면 상행선
                        self.log_message(f"🟢 상행선 경로 선택 (X거리 {x_diff:.2f}m, 아래에서 출발)", "SUCCESS", is_system=True)
                        return 'highway_up'
                    else:  # 현재가 위쪽이면 하행선
                        self.log_message(f"🔵 하행선 경로 선택 (X거리 {x_diff:.2f}m, 위에서 출발)", "SUCCESS", is_system=True)
                        return 'highway_down'
                
                self.log_message(f"➡️ 직접 경로 (가까운 거리: Y차이={y_diff:.2f}m, X차이={x_diff:.2f}m)", "INFO", is_system=True)
                return None
                
        except Exception as e:
            self.log_message(f"❌ highway 선택 오류: {e}", "ERROR", is_system=True)
            return None
    
    def _create_full_path(self, current_pos, highway_name, dest_pos):
        """전체 경로 생성: 시작점 -> highway waypoints -> 목적지"""
        try:
            full_path = []
            
            # 1. 현재 위치 추가
            start_pose = {
                'pose': {
                    'position': {'x': current_pos.x, 'y': current_pos.y, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            }
            full_path.append(start_pose)
            
            # 2. Highway waypoints 추가
            highway_points = self.waypoints.get(highway_name, [])
            
            # 현재 위치에서 가장 가까운 highway 진입점 찾기
            if highway_points:
                closest_idx = self._find_closest_waypoint_index(current_pos, highway_points)
                
                if highway_name == 'highway_up':
                    # 상행선: closest_idx부터 끝까지
                    selected_points = highway_points[closest_idx:]
                else:  # highway_down
                    # 하행선: closest_idx부터 끝까지
                    selected_points = highway_points[closest_idx:]
                
                full_path.extend(selected_points)
            
            # 3. 최종 목적지 추가
            dest_pose = {
                'pose': {
                    'position': dest_pos,
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            }
            full_path.append(dest_pose)
            
            # 로그 출력
            for i, point in enumerate(full_path):
                pos = point['pose']['position']
                self.log_message(f"📍 경로 {i+1}: ({pos['x']:.2f}, {pos['y']:.2f})", "INFO", is_system=True)
            
            return full_path
            
        except Exception as e:
            self.log_message(f"❌ 전체 경로 생성 오류: {e}", "ERROR", is_system=True)
            return []
    
    def _find_closest_waypoint_index(self, current_pos, waypoints):
        """현재 위치에서 가장 가까운 waypoint index 찾기"""
        try:
            min_distance = float('inf')
            closest_idx = 0
            
            for i, point in enumerate(waypoints):
                wp_pos = point['pose']['position']
                distance = ((current_pos.x - wp_pos['x'])**2 + (current_pos.y - wp_pos['y'])**2)**0.5
                
                if distance < min_distance:
                    min_distance = distance
                    closest_idx = i
            
            self.log_message(f"🎯 가장 가까운 waypoint: index {closest_idx}, 거리 {min_distance:.2f}m", "INFO", is_system=True)
            return closest_idx
            
        except Exception as e:
            self.log_message(f"❌ closest waypoint 찾기 오류: {e}", "ERROR", is_system=True)
            return 0
    
    def _execute_planned_path(self, robot_name: str, path_points: list, destination_name: str) -> bool:
        """계획된 경로 실행"""
        try:
            robot = self.robots[robot_name]
            navigator = self.navigators[robot_name]
            
            if not navigator:
                self.log_message(f"❌ {robot_name} 네비게이터 없음", "ERROR", is_system=True)
                return False
            
            # 기존 작업 취소
            if not navigator.isTaskComplete():
                navigator.cancelTask()
            
            goal_poses = []
            for i, point in enumerate(path_points):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                
                pos = point['pose']['position']
                ori = point['pose']['orientation']
                
                pose.pose.position.x = float(pos['x'])
                pose.pose.position.y = float(pos['y'])
                pose.pose.position.z = float(pos.get('z', 0.0))
                
                pose.pose.orientation.x = float(ori.get('x', 0.0))
                pose.pose.orientation.y = float(ori.get('y', 0.0))
                pose.pose.orientation.z = float(ori.get('z', 0.0))
                pose.pose.orientation.w = float(ori.get('w', 1.0))
                
                goal_poses.append(pose)
            
            self.log_message(f"🚀 {robot_name} A* 경로 실행 시작: {destination_name}", "SUCCESS", is_system=True)
            robot.state = RobotState.MOVING
            robot.task_type = f"이동: {destination_name}"
            
            navigator.followWaypoints(goal_poses)
            return True
            
        except Exception as e:
            self.log_message(f"❌ 경로 실행 오류: {e}", "ERROR", is_system=True)
            return False

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
        """로봇을 지정 위치로 이동 (A* path planning 사용)"""
        # 먼저 A* path planning 시도
        if self.plan_path_with_waypoints(robot_name, location):
            return True
        
        # A* 실패 시 기존 방식 사용
        self.log_message(f"⚠️ A* 실패, 기존 방식으로 이동: {location}", "WARNING", is_system=True)
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