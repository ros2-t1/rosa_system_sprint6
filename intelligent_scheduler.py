#!/usr/bin/env python3

import math
import time
from typing import Tuple, Optional
from robot_manager import RobotState, Task, TaskType

class IntelligentTaskScheduler:
    def __init__(self, robot_manager, input_only: bool = False):
        self.robot_manager = robot_manager
        self.input_only = input_only
        self.task_completion_history = []
        
    def calculate_distance(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def estimate_task_duration(self, robot, task: Task) -> float:
        if not robot.current_pose or task.destination not in self.robot_manager.location_manager.all_locations:
            return 300.0
        current_pos = (robot.current_pose.position.x, robot.current_pose.position.y)
        dest_pos = self.robot_manager.location_manager.get_location_coordinates(task.destination)
        if task.type == TaskType.DELIVERY:
            pickup_pos = self.robot_manager.location_manager.get_location_coordinates("픽업대")
            pickup_distance = self.calculate_distance(current_pos, pickup_pos)
            delivery_distance = self.calculate_distance(pickup_pos, dest_pos)
            total_distance = pickup_distance + delivery_distance
            travel_time = total_distance / robot.average_speed
            handling_time = 30.0
            queue_length = self.robot_manager.location_manager.get_queue_length('픽업대')
            queue_wait_time = queue_length * 60.0
            return travel_time + handling_time + queue_wait_time
        else:
            distance = self.calculate_distance(current_pos, dest_pos)
            return distance / robot.average_speed
    
    def calculate_robot_availability_time(self, robot) -> float:
        current_time = time.time()
        if robot.state in [RobotState.IDLE, RobotState.RETURNING]:  # 복귀중인 로봇도 업무 할당 가능
            return current_time
        elif robot.current_task:
            remaining_time = self.estimate_task_duration(robot, robot.current_task)
            return current_time + remaining_time
        else:
            return current_time + 60.0
    
    def calculate_priority_score(self, task: Task) -> float:
        base_score = task.priority.value * 100
        current_time = time.time()
        age_minutes = (current_time - task.created_time) / 60.0
        urgency_bonus = age_minutes * 2
        deadline_penalty = 0
        if task.deadline:
            time_until_deadline = (task.deadline - current_time) / 60.0
            if time_until_deadline < 30:
                deadline_penalty = 100
            elif time_until_deadline < 60:
                deadline_penalty = 50
        return base_score + urgency_bonus + deadline_penalty
    
    def find_optimal_robot(self, task: Task, log_message: callable) -> Optional[str]:
        log_message(f"🧠 AI 분석 중: {task.destination}에 {task.item if task.item else '이동'} 작업", is_command=True)
        if task.type == TaskType.DELIVERY:
            queue_length = self.robot_manager.location_manager.get_queue_length('픽업대')
            is_busy = self.robot_manager.location_manager.is_location_busy('픽업대')
            occupant = self.robot_manager.location_manager.location_occupancy.get('픽업대', '없음')
            log_message(f"📦 픽업대 현황: {'사용중' if is_busy else '비어있음'} (사용자: {occupant}), 대기: {queue_length}명", is_command=True)
        
        # 1단계: 충전소에서 대기 중인 로봇들 우선 선택
        charging_robots = []
        other_robots = []
        
        for robot_name, robot in self.robot_manager.robots.items():
            if robot.state == RobotState.LOW_BATTERY:
                log_message(f"❌ {robot_name}: 배터리 부족 ({robot.battery_level:.1f}%)", is_command=True)
                continue
            if robot.state == RobotState.WAITING_FOR_CONFIRMATION:
                log_message(f"❌ {robot_name}: 응답 대기 중 (작업 할당 불가)", is_command=True)
                continue
            if robot.is_permanently_stationed:
                log_message(f"❌ {robot_name}: 영구 대기 중 (명시적 지정 필요)", is_command=True)
                continue
            if not robot.current_pose:
                current_time = time.time()
                time_since_last_pose = current_time - robot.last_pose_time
                log_message(f"❌ {robot_name}: 위치 정보 없음 (마지막 업데이트: {time_since_last_pose:.1f}초 전)", is_command=True)
                continue
            
            # 충전소에 있는지 확인
            current_location = self.robot_manager.get_current_location_name(robot_name)
            charge_station = self.robot_manager.get_charge_station_for_robot(robot_name)
            is_at_charging_station = (charge_station in current_location and 
                                    robot.state in [RobotState.IDLE, RobotState.CHARGING])
            
            if is_at_charging_station and not robot.current_task:
                charging_robots.append((robot_name, robot))
                log_message(f"🔋 {robot_name}: 충전소에서 대기 중 (배터리: {robot.battery_level:.1f}%)", is_command=True)
            else:
                other_robots.append((robot_name, robot))
        
        # 충전소 로봇이 있으면 배터리 많은 순으로 선택
        if charging_robots:
            charging_robots.sort(key=lambda x: x[1].battery_level, reverse=True)
            selected_robot = charging_robots[0][0]
            log_message(f"✅ 충전소 로봇 선택: {selected_robot} (배터리: {charging_robots[0][1].battery_level:.1f}%)", is_command=True)
            return selected_robot
        
        # 충전소 로봇이 없으면 기존 로직으로 처리
        candidates = []
        for robot_name, robot in other_robots:
            
            available_time = self.calculate_robot_availability_time(robot)
            current_time = time.time()
            wait_time = max(0, available_time - current_time)
            task_duration = self.estimate_task_duration(robot, task)
            total_completion_time = wait_time + task_duration
            current_pos = (robot.current_pose.position.x, robot.current_pose.position.y)
            if task.destination in self.robot_manager.location_manager.all_locations:
                dest_pos = self.robot_manager.location_manager.get_location_coordinates(task.destination)
                distance = self.calculate_distance(current_pos, dest_pos)
                distance_score = max(0, 100 - distance * 50)
            else:
                distance_score = 0
            efficiency_score = robot.efficiency_score * 50
            battery_score = (robot.battery_level - 40) * 2
            wait_penalty = wait_time * 2
            queue_penalty = 0
            if task.type == TaskType.DELIVERY:
                queue_length = self.robot_manager.location_manager.get_queue_length('픽업대')
                queue_penalty = queue_length * 20
            
            # 복귀중인 로봇에게 보너스 점수 (업무 할당 가능하므로)
            returning_bonus = 20 if robot.state == RobotState.RETURNING else 0
            
            total_score = distance_score + efficiency_score + battery_score - wait_penalty - queue_penalty + returning_bonus
            candidates.append({
                'robot_name': robot_name,
                'total_score': total_score,
                'distance': distance if 'distance' in locals() else 999,
                'wait_time': wait_time,
                'completion_time': total_completion_time,
                'battery': robot.battery_level,
                'efficiency': robot.efficiency_score,
                'state': robot.state.value,
                'queue_penalty': queue_penalty,
                'returning_bonus': returning_bonus
            })
            log_message(f"🤖 {robot_name}: 점수 {total_score:.1f} (거리:{distance:.2f}m, 대기:{wait_time:.0f}s, 배터리:{robot.battery_level:.1f}%, 복귀보너스:{returning_bonus})", is_command=True)
        
        if not candidates:
            log_message("❌ 사용 가능한 로봇이 없습니다!", is_command=True)
            return None
        best_robot = max(candidates, key=lambda x: x['total_score'])
        log_message(f"🎯 최적 선택: {best_robot['robot_name']} (점수: {best_robot['total_score']:.1f})", is_command=True)
        log_message(f"└─ 예상 완료 시간: {best_robot['completion_time']:.1f}초 후", is_command=True)
        return best_robot['robot_name']
    
    def update_robot_efficiency(self, robot_name: str, task: Task, actual_duration: float, log_message: callable):
        robot = self.robot_manager.robots[robot_name]
        estimated_duration = task.estimated_duration
        if estimated_duration > 0:
            accuracy = min(estimated_duration / actual_duration, 2.0)
            alpha = 0.3
            robot.efficiency_score = (1 - alpha) * robot.efficiency_score + alpha * accuracy
            log_message(f"📊 {robot_name} 효율성 업데이트: {robot.efficiency_score:.2f}", is_command=True)
