#!/usr/bin/env python3

import time
from datetime import datetime
from typing import Optional
from robot_manager import Task, TaskType, Priority, RobotState

class CommandParser:
    def __init__(self, robot_manager, task_processor):
        self.robot_manager = robot_manager
        self.task_processor = task_processor
        self.log_message = task_processor.log_message # 로깅 함수 공유

    def parse_intelligent_command(self, cmd: str):
        """AI 기반 자연어 명령 처리"""
        if self.task_processor.status_only:
            return
        
        self.log_message(f"🧠 AI 명령 분석: '{cmd}'", is_command=True)
        
        # 🔄 새로고침/깨우기 명령 감지
        if any(word in cmd.lower() for word in ['refresh', '새로고침', '깨워', 'wake', 'reset']):
            self._handle_refresh_command()
            return
        
        # 🔍 로봇 상태 질문 감지
        if any(word in cmd for word in ['뭐해', '어디야', '상태', 'status']) and any(num in cmd for num in ['3번', '8번', '9번']):
            self._handle_robot_status_query(cmd)
            return
        
        # 우선순위 감지
        priority = Priority.NORMAL
        if any(word in cmd.lower() for word in ['급', '빨리', 'urgent', '긴급']):
            priority = Priority.HIGH
            self.log_message("⚡ 높은 우선순위로 설정", is_command=True)
        elif any(word in cmd.lower() for word in ['천천히', '나중에', 'low']):
            priority = Priority.LOW
            self.log_message("🐌 낮은 우선순위로 설정", is_command=True)
        
        # 로봇 지정 감지
        robot_name = None
        for i, rname in enumerate(['3번', '8번', '9번']):
            if rname in cmd:
                robot_name = self.robot_manager.robot_names[i]
                cmd = cmd.replace(rname, '').strip()
                self.log_message(f"🎯 수동 로봇 지정: {robot_name}", is_command=True)
                break
        
        # 복귀 명령 감지
        is_return_command = False
        is_charge_station_command = False
        
        if any(word in cmd.lower() for word in ['복귀', '돌아가', '돌아와', '귀환']):
            is_return_command = True
            if not robot_name:
                self.log_message("❌ '복귀' 명령은 로봇을 지정해야 합니다. (예: '3번 로봇 복귀해')", "ERROR", is_command=True)
                return
            destination = self.robot_manager.get_charge_station_for_robot(robot_name)
            self.log_message(f"🏠 '{destination}'로 복귀 명령으로 해석 (업무 할당 가능)", is_command=True)
        else:
            # 목적지 감지
            destination = None
            for location in self.robot_manager.location_manager.main_locations.keys():
                if location in cmd:
                    destination = location
                    break
            
            # 충전소 명령 특별 처리
            if not destination and "충전소" in cmd:
                if not robot_name:
                    self.log_message("❌ '충전소' 명령은 로봇을 지정해야 합니다. (예: '3번 로봇 충전소 가')", "ERROR", is_command=True)
                    return
                destination = self.robot_manager.get_charge_station_for_robot(robot_name)
                is_charge_station_command = True
                self.log_message(f"🏠 '{destination}'로 복귀 명령으로 해석 (업무 할당 가능)", is_command=True)
            
            if not destination:
                self.log_message("❌ 목적지를 찾을 수 없습니다.", "ERROR", is_command=True)
                self.log_message(f"💡 사용 가능한 위치: {', '.join(self.robot_manager.location_manager.main_locations.keys())}", is_command=True)
                return
        
        # 작업 유형 및 물품 감지
        if is_return_command:
            task_type = TaskType.RETURN_TO_CHARGE
            item = ""
            self.log_message("🏠 복귀 작업으로 분류 (업무 할당 가능)", is_command=True)
        elif is_charge_station_command:
            task_type = TaskType.RETURN_TO_CHARGE
            item = ""
            self.log_message("🏠 복귀 작업으로 분류 (업무 할당 가능)", is_command=True)
        else:
            task_type = TaskType.SIMPLE_MOVE
            item = ""
            
            if any(word in cmd for word in ['배달', '갖다', '전달', '운반', '가져가']):
                task_type = TaskType.DELIVERY
                self.log_message("📦 배송 작업으로 분류", is_command=True)
                
                if "물" in cmd:
                    item = "물"
                elif "서류" in cmd:
                    item = "서류"
                elif "상자" in cmd:
                    item = "상자"
                elif "음식" in cmd:
                    item = "음식"
                elif "영양제" in cmd:
                    item = "영양제"
                elif "도시락" in cmd:
                    item = "도시락"
                else:
                    item = "물품"
            else:
                self.log_message("🚶 이동 작업으로 분류", is_command=True)
        
        # 작업 생성 - 고유한 Task ID 생성 (타임스탬프 기반)
        import time
        task_counter = getattr(self.task_processor, '_task_counter', 0) + 1
        self.task_processor._task_counter = task_counter
        task_id = f"AI{task_counter:03d}"
        task = Task(
            id=task_id,
            type=task_type,
            destination=destination,
            item=item,
            priority=priority,
            created_time=time.time()
        )
        
        if is_charge_station_command:
            task.is_charge_station_command = True
        elif is_return_command:
            task.is_return_command = True
        
        self.task_processor.add_task_to_queue(task, robot_name, is_return_command, is_charge_station_command)
    
    def _handle_refresh_command(self):
        """새로고침/깨우기 명령 처리"""
        self.log_message("🔄 시스템 새로고침 실행", is_command=True)
        
        # 멈춰있는 로봇들 깨우기
        woken_robots = []
        for robot_name, robot in self.robot_manager.robots.items():
            if robot.current_task and not robot.is_moving:
                # 현재 작업이 있지만 움직이지 않는 로봇 발견
                task_time = robot.current_task.created_time if robot.current_task else 0
                current_time = time.time()
                stuck_duration = current_time - task_time
                
                if stuck_duration > 10:  # 10초 이상 멈춘 로봇
                    self.log_message(f"⚡ {robot_name} 로봇 깨우기 시도 (멈춘 시간: {stuck_duration:.0f}초)", is_command=True)
                    
                    # 로봇 상태 강제 리셋
                    robot.state = RobotState.WORKING if robot.current_task else RobotState.IDLE
                    robot.waiting_for_confirmation = False
                    robot.confirmation_prompt = ""
                    
                    # 작업 재시작 시도
                    if robot.current_task:
                        if robot.task_step == 1.5:  # 픽업 중
                            self.log_message(f"🔄 {robot_name} 픽업 작업 재시작", is_command=True)
                        elif robot.task_step == 2.5:  # 배달 확인 중
                            self.log_message(f"🔄 {robot_name} 배달 확인 재시작", is_command=True)
                    
                    woken_robots.append(robot_name)
        
        if woken_robots:
            self.log_message(f"⚡ 깨운 로봇: {', '.join(woken_robots)}", is_command=True)
        else:
            self.log_message("✅ 멈춘 로봇이 없습니다. 모든 로봇이 정상 작동 중!", is_command=True)
        
        # 확인 요청 상태 초기화
        self.task_processor.waiting_for_global_confirmation = False
        self.task_processor.global_confirmation_prompt = ""
        self.task_processor.confirmation_robot = ""
        
        # 큐 처리 강제 실행
        self.task_processor.process_location_queues()
        
        self.log_message("🔄 시스템 새로고침 완료!", is_command=True)
    
    def _handle_robot_status_query(self, cmd: str):
        """로봇 상태 질문 처리"""
        # 로봇 번호 추출
        robot_name = None
        if '3번' in cmd:
            robot_name = 'DP_03'
        elif '8번' in cmd:
            robot_name = 'DP_08' 
        elif '9번' in cmd:
            robot_name = 'DP_09'
        
        if not robot_name:
            self.log_message("❌ 로봇 번호를 찾을 수 없습니다", is_command=True)
            return
        
        robot = self.robot_manager.robots.get(robot_name)
        if not robot:
            self.log_message(f"❌ {robot_name} 로봇을 찾을 수 없습니다", is_command=True)
            return
        
        # 현재 위치
        current_location = self.robot_manager.get_current_location_name(robot_name)
        
        # 현재 상태 분석
        if not robot.current_pose:
            status_msg = f"📡 {robot_name}: 연결되지 않음"
        elif not robot.current_task:
            status_msg = f"😴 {robot_name}: {current_location}에서 대기 중 (배터리: {robot.battery_level:.1f}%)"
        else:
            task = robot.current_task
            task_id = task.id
            
            if robot.state == RobotState.WAITING_FOR_CONFIRMATION:
                status_msg = f"❓ {robot_name}: [{task_id}] 확인 응답 대기 중 @ {current_location}"
            elif task.type.name == "DELIVERY":
                if robot.task_step == 1:
                    status_msg = f"🚚 {robot_name}: [{task_id}] 픽업대로 이동 중 @ {current_location}"
                elif robot.task_step == 1.5:
                    status_msg = f"📦 {robot_name}: [{task_id}] 픽업대에서 {task.item} 적재 중 @ {current_location}"
                elif robot.task_step == 2:
                    status_msg = f"🚛 {robot_name}: [{task_id}] {task.destination}에 {task.item} 배달 중 @ {current_location}"
                elif robot.task_step == 2.5:
                    status_msg = f"📍 {robot_name}: [{task_id}] {task.destination}에서 {task.item} 전달 완료 대기 @ {current_location}"
                else:
                    status_msg = f"🚀 {robot_name}: [{task_id}] 배달 작업 진행 중 @ {current_location}"
            elif task.type.name == "RETURN_TO_CHARGE":
                status_msg = f"🏠 {robot_name}: [{task_id}] {task.destination}로 복귀 중 @ {current_location}"
            else:
                status_msg = f"🚶 {robot_name}: [{task_id}] {task.destination}로 이동 중 @ {current_location}"
        
        # 배터리 상태 추가
        if robot.current_pose:
            battery_status = "🔋" if robot.battery_level > 60 else "🪫" if robot.battery_level > 20 else "🚨"
            status_msg += f" {battery_status}{robot.battery_level:.1f}%"
        
        # 멈춰있는 시간 확인
        if robot.current_task and not robot.is_moving:
            task_time = robot.current_task.created_time if robot.current_task else 0
            stuck_duration = time.time() - task_time
            if stuck_duration > 30:
                status_msg += f" ⚠️ (멈춘 지 {stuck_duration:.0f}초)"
        
        self.log_message(status_msg, is_command=True)