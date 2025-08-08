#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, scrolledtext
from tkinter import font as tkFont
import time
import subprocess
import os
import threading
import cv2
from PIL import Image, ImageTk
from datetime import datetime
from robot_manager import RobotState, TaskType
from pinky_interfaces.msg import Confirmation

class StatusDisplayGUI:
    """향상된 상태 표시 GUI"""
    
    def __init__(self, robot_manager, task_processor):
        self.robot_manager = robot_manager
        self.task_processor = task_processor
        self.root = tk.Tk()
        self.root.title("🤖 ROSA 로봇 관제 시스템 - 상태 모니터")
        self.root.geometry("1500x900")
        self.root.configure(bg='#2b2b2b')
        
        # 폰트 설정
        self.font_large = tkFont.Font(family="맑은 고딕", size=12, weight="bold")
        self.font_medium = tkFont.Font(family="맑은 고딕", size=10)
        self.font_small = tkFont.Font(family="맑은 고딕", size=9)
        
        self.setup_widgets()
        self.confirmation_active = False
        self.current_confirmation = None
        self.last_queue_length = 0
        
        # 작업 상태 전환 추적용
        self.completed_tasks = {}  # {task_id: timestamp}
        self.task_status_cache = {}  # {task_id: current_status}
        self.status_transitions = {}  # {task_id: transition_info}
        
        # 글로벌 카메라 관련
        self.camera_available = False
        self.camera_capture = None
        self.camera_thread = None
        self.camera_running = False
        self.current_camera_frame = None
        self._last_camera_error_time = 0
        self._last_capture_error_time = 0
        
        
    def setup_widgets(self):
        # 메인 프레임
        main_frame = tk.Frame(self.root, bg='#2b2b2b')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 상단: 제목 및 시간
        header_frame = tk.Frame(main_frame, bg='#2b2b2b')
        header_frame.pack(fill=tk.X, pady=(0, 10))
        
        title_label = tk.Label(header_frame, text="🤖 ROSA 로봇 관제 시스템 - 상태 모니터", 
                              font=self.font_large, fg='#00ff88', bg='#2b2b2b')
        title_label.pack(side=tk.LEFT)
        
        self.time_label = tk.Label(header_frame, text="", 
                                  font=self.font_medium, fg='#ffffff', bg='#2b2b2b')
        self.time_label.pack(side=tk.RIGHT)
        
        # 좌측: 로봇 상태
        left_frame = tk.Frame(main_frame, bg='#2b2b2b')
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # 글로벌 카메라 화면
        camera_label = tk.Label(left_frame, text="📷 글로벌 카메라", 
                               font=self.font_large, fg='#ffaa00', bg='#2b2b2b')
        camera_label.pack(anchor='w')
        
        self.camera_frame = tk.Frame(left_frame, bg='#3b3b3b', relief=tk.RAISED, bd=1, height=200)
        self.camera_frame.pack(fill=tk.X, pady=(5, 10))
        self.camera_frame.pack_propagate(False)  # 고정 높이 유지
        
        # 카메라 화면을 표시할 라벨 (이미지용)
        self.camera_display = tk.Label(self.camera_frame, bg='#3b3b3b')
        self.camera_display.pack(expand=True, fill=tk.BOTH)
        
        # 상태 표시용 라벨 (카메라 화면 위에 오버레이)
        self.camera_status_label = tk.Label(self.camera_frame, text="📡 카메라 연결 확인 중...", 
                                           font=self.font_medium, fg='#ffffff', bg='#3b3b3b')
        self.camera_status_label.place(relx=0.5, rely=0.5, anchor='center')
        
        # 로봇 상태 라벨
        robot_label = tk.Label(left_frame, text="🤖 로봇 상태", 
                              font=self.font_large, fg='#ffaa00', bg='#2b2b2b')
        robot_label.pack(anchor='w', pady=(10, 0))
        
        # 로봇 상태 트리뷰
        robot_frame = tk.Frame(left_frame, bg='#3b3b3b', relief=tk.RAISED, bd=1)
        robot_frame.pack(fill=tk.X, pady=(5, 10))
        
        robot_columns = ('로봇', '상태', '배터리', '진행상황', '현재위치')
        self.robot_tree = ttk.Treeview(robot_frame, columns=robot_columns, show='headings', height=4)
        
        # 컬럼 설정
        self.robot_tree.heading('로봇', text='로봇')
        self.robot_tree.heading('상태', text='상태')
        self.robot_tree.heading('배터리', text='배터리')
        self.robot_tree.heading('진행상황', text='진행상황')
        self.robot_tree.heading('현재위치', text='현재위치')
        
        self.robot_tree.column('로봇', width=80, anchor='center')
        self.robot_tree.column('상태', width=100, anchor='center')
        self.robot_tree.column('배터리', width=90, anchor='center')
        self.robot_tree.column('진행상황', width=280, anchor='center')
        self.robot_tree.column('현재위치', width=120, anchor='center')
        
        # 스크롤바
        robot_scrollbar = ttk.Scrollbar(robot_frame, orient=tk.VERTICAL, command=self.robot_tree.yview)
        self.robot_tree.configure(yscrollcommand=robot_scrollbar.set)
        
        self.robot_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        robot_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 작업 대기열
        queue_label = tk.Label(left_frame, text="📝 작업 대기열", 
                              font=self.font_large, fg='#ffaa00', bg='#2b2b2b')
        queue_label.pack(anchor='w', pady=(10, 0))
        
        queue_frame = tk.Frame(left_frame, bg='#3b3b3b', relief=tk.RAISED, bd=1)
        queue_frame.pack(fill=tk.X, pady=(5, 10))
        
        queue_columns = ('우선순위', '작업ID', '로봇', '목적지', '물품', '예상시간')
        self.queue_tree = ttk.Treeview(queue_frame, columns=queue_columns, show='headings', height=16)
        
        for col in queue_columns:
            self.queue_tree.heading(col, text=col)
            self.queue_tree.column(col, width=100, anchor='center')
        
        queue_scrollbar = ttk.Scrollbar(queue_frame, orient=tk.VERTICAL, command=self.queue_tree.yview)
        self.queue_tree.configure(yscrollcommand=queue_scrollbar.set)
        
        self.queue_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        queue_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 확인 요청 프레임 (작업대기열 바로 아래) - 항상 표시하되 내용만 변경
        self.confirmation_frame = tk.Frame(left_frame, bg='#ff6600', relief=tk.RAISED, bd=3)
        self.confirmation_frame.pack(fill=tk.X, pady=(5, 0))  # 항상 표시
        
        conf_title = tk.Label(self.confirmation_frame, text="❓ 확인 요청", 
                             font=self.font_large, fg='#ffffff', bg='#ff6600')
        conf_title.pack(pady=(10, 5))
        
        self.confirmation_text = tk.Label(self.confirmation_frame, text="확인 요청이 없습니다.", 
                                         font=self.font_medium, fg='#ffffff', bg='#ff6600',
                                         wraplength=700)
        self.confirmation_text.pack(pady=5)
        
        button_frame = tk.Frame(self.confirmation_frame, bg='#ff6600')
        button_frame.pack(pady=(10, 15))
        
        self.yes_button = tk.Button(button_frame, text="✅ YES", 
                                   font=self.font_medium, bg='#00aa00', fg='white',
                                   command=self.on_yes_click, width=12, height=2, state='disabled')
        self.yes_button.pack(side=tk.LEFT, padx=20)
        
        self.no_button = tk.Button(button_frame, text="❌ NO", 
                                  font=self.font_medium, bg='#aa0000', fg='white',
                                  command=self.on_no_click, width=12, height=2, state='disabled')
        self.no_button.pack(side=tk.LEFT, padx=20)
        
        # 우측: 시스템 로그 (연결 상태와 확인 요청만)
        right_frame = tk.Frame(main_frame, bg='#2b2b2b')
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        log_label = tk.Label(right_frame, text="📄 시스템 로그 (연결 상태 & 확인 요청)", 
                            font=self.font_large, fg='#ffaa00', bg='#2b2b2b')
        log_label.pack(anchor='w')
        
        log_frame = tk.Frame(right_frame, bg='#3b3b3b', relief=tk.RAISED, bd=1)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(5, 0))
        
        self.log_text = scrolledtext.ScrolledText(log_frame, 
                                                 bg='#1e1e1e', fg='#00ff00',
                                                 font=self.font_small,
                                                 wrap=tk.WORD, height=25)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 카메라 상태 확인 시작
        self.start_camera_monitoring()
        
    def add_log(self, message: str, level: str = "INFO"):
        """로그 메시지 추가 (시스템 로그만 - 연결 상태, 확인 요청 관련만)"""
        # 시스템 로그에 표시할 메시지만 필터링
        system_keywords = ["연결", "확인", "응답", "GUI", "TF", "배터리", "복구", "중단", "불안정"]
        
        if any(keyword in message for keyword in system_keywords):
            timestamp = datetime.now().strftime("%H:%M:%S")
            
            # 색상 설정
            colors = {
                "INFO": "#00ff00",
                "WARNING": "#ffaa00", 
                "ERROR": "#ff0000",
                "SUCCESS": "#00ffaa"
            }
            color = colors.get(level, "#ffffff")
            
            # 로그 추가
            formatted_msg = f"[{timestamp}] {message}\n"
            
            self.log_text.insert(tk.END, formatted_msg)
            self.log_text.see(tk.END)
            
            # 최대 라인 수 제한 (성능 향상)
            lines = self.log_text.get("1.0", tk.END).split('\n')
            if len(lines) > 1000:
                # 처음 500줄 삭제
                self.log_text.delete("1.0", "500.0")
    
    def update_display(self):
        """화면 업데이트"""
        # 시간 업데이트
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.time_label.config(text=current_time)
        
        # 로봇 상태 업데이트
        self.update_robot_status()
        
        # 작업 대기열 업데이트
        self.update_queue_status()
        
        # 200ms 후 다시 업데이트
        self.root.after(1000, self.update_display)  # 200ms → 1000ms로 최적화
    
    def update_robot_status(self):
        """로봇 상태 트리뷰 업데이트"""
        # 기존 항목 삭제
        for item in self.robot_tree.get_children():
            self.robot_tree.delete(item)
        
        # 로봇 상태 추가
        for robot_name, robot in self.robot_manager.robots.items():
            # 상태 결정 (요구사항에 맞게 수정)
            if robot.current_pose is None:
                state_text = "연결안됨"
            elif robot.state == RobotState.LOW_BATTERY:
                state_text = "충전중"
            elif robot.state == RobotState.WAITING_FOR_CONFIRMATION:
                state_text = "응답대기중"
            elif robot.state == RobotState.RETURNING:
                state_text = "귀환중"
            elif robot.current_task and robot.current_task.type == TaskType.DELIVERY:
                state_text = "배달중"
            elif robot.current_task:
                state_text = "이동중"
            else:
                state_text = "충전중" if robot.state == RobotState.CHARGING else "대기중"
            
            # 배터리 표시
            if robot.current_pose is None:
                battery_text = "알 수 없음"
            else:
                battery_icon = "🔋" if robot.battery_level > 60 else "🪫" if robot.battery_level > 20 else "🚨"
                battery_text = f"{battery_icon} {robot.battery_level:.1f}%"
            
            # 진행상황 결정 (작업ID 포함 표시)
            if robot.current_pose is None:
                progress = "연결되지 않음"
            elif robot.current_task:
                task = robot.current_task
                task_id_prefix = f"[{task.id}] "
                
                if task.type == TaskType.DELIVERY:
                    if robot.task_step == 0.5:
                        if robot.queue_position >= 0:
                            progress = f"{task_id_prefix}픽업대에서 대기중 ({robot.queue_position + 1}번째)"
                        else:
                            progress = f"{task_id_prefix}픽업대에서 대기중"
                    elif robot.task_step == 1:
                        progress = f"{task_id_prefix}픽업대로 이동중"
                    elif robot.task_step == 1.5:
                        progress = f"{task_id_prefix}픽업대에서 {task.item} 적재 대기중"
                    elif robot.task_step == 2:
                        progress = f"{task_id_prefix}{task.destination}에 {task.item} 배달중"
                    elif robot.task_step == 2.5:
                        progress = f"{task_id_prefix}{task.destination}에서 {task.item} 전달 완료 대기중"
                    else:
                        progress = f"{task_id_prefix}{task.destination}에 {task.item} 배달중"
                else:
                    # SIMPLE_MOVE
                    if robot.is_moving:
                        progress = f"{task_id_prefix}{task.destination}로 이동중"
                    else:
                        progress = f"{task_id_prefix}{task.destination}에서 대기중"
            elif robot.state == RobotState.LOW_BATTERY:
                progress = "배터리 부족 - 충전소에서 대기중"
            elif robot.is_moving and robot.state == RobotState.MOVING:
                charge_station = self.robot_manager.get_charge_station_for_robot(robot_name)
                progress = f"{charge_station}로 귀환중"
            elif robot.state == RobotState.CHARGING:
                progress = "충전소에서 대기중"
            else:
                progress = "대기중"
            
            # 위치
            location = self.robot_manager.get_current_location_name(robot_name)
            
            # 연결 상태에 따른 색상 태그
            if robot.current_pose is None:
                tag = "disconnected"
            elif robot.battery_level < 40:
                tag = "low_battery"
            elif robot.current_task:
                tag = "working"
            else:
                tag = "normal"
            
            # 행 추가
            item_id = self.robot_tree.insert('', 'end', values=(
                robot_name, state_text, battery_text, progress, location
            ), tags=(tag,))
        
        # 완료된 작업 체크 및 기록
        self._check_for_completed_tasks()
        
        # 태그 색상 설정
        self.robot_tree.tag_configure("disconnected", foreground="gray")
        self.robot_tree.tag_configure("low_battery", foreground="red")
        self.robot_tree.tag_configure("working", foreground="blue")
        self.robot_tree.tag_configure("normal", foreground="black")
    
    def update_queue_status(self):
        """작업 대기열 트리뷰 업데이트 - 수정된 버전"""
        # 완료된 작업 정리 (3초 후)
        current_time = time.time()
        expired_tasks = []
        for task_id, completion_time in self.completed_tasks.items():
            if current_time - completion_time > 3.0:
                expired_tasks.append(task_id)
        
        for task_id in expired_tasks:
            del self.completed_tasks[task_id]
            if task_id in self.task_status_cache:
                del self.task_status_cache[task_id]
        
        # 기존 항목 삭제
        for item in self.queue_tree.get_children():
            self.queue_tree.delete(item)
        
        # task_processor 확인
        if not self.task_processor:
            self.add_log("⚠️ GUI: task_processor가 None입니다", "WARNING")
            return
        
        if not hasattr(self.task_processor, 'global_task_queue'):
            self.add_log("⚠️ GUI: global_task_queue 속성이 없습니다", "WARNING")
            return
        
        try:
            # 진행 중인 작업 먼저 추가
            working_count = 0
            for robot_name, robot in self.robot_manager.robots.items():
                if robot.current_task:
                    task = robot.current_task
                    priority_icons = {
                        "LOW": "🔵",
                        "NORMAL": "🟡", 
                        "HIGH": "🔴",
                        "URGENT": "⚡"
                    }
                    
                    priority_text = f"{priority_icons.get(task.priority.name, '⚪')} {task.priority.name}"
                    item_text = task.item if task.item else "-"
                    estimated_time = f"{task.estimated_duration:.0f}초"
                    
                    # 작업 상태 전환 추적
                    task_id = task.id
                    previous_status = self.task_status_cache.get(task_id, "대기중")
                    
                    # 진행 상태 상세 표시 (상태 전환 포함)
                    if robot.state.name == "WAITING_FOR_CONFIRMATION":
                        status_icon = "❓"
                        status_text = "[확인 대기]"
                        tag = "waiting_confirmation"
                        current_status = "확인 대기"
                    elif task.type.name == "DELIVERY":
                        if robot.task_step == 1:
                            status_icon = "🚚"
                            status_text = "[픽업대로 이동]"
                            current_status = "작업중"
                        elif robot.task_step == 1.5:
                            status_icon = "📦"
                            status_text = "[픽업 대기]"
                            current_status = "작업중"
                        elif robot.task_step == 2:
                            status_icon = "🚛"
                            status_text = "[배달중]"
                            current_status = "작업중"
                        elif robot.task_step == 2.5:
                            status_icon = "📍"
                            status_text = "[전달 대기]"
                            current_status = "작업중"
                        else:
                            status_icon = "🚀"
                            status_text = "[진행중]"
                            current_status = "작업중"
                        tag = "working"
                    else:
                        status_icon = "🚀"
                        status_text = "[진행중]"
                        current_status = "작업중"
                        tag = "working"
                    
                    # 상태 변화 감지 및 캐시 업데이트 (상세 전환 추적)
                    if previous_status != current_status:
                        self.task_status_cache[task_id] = current_status
                        transition_time = time.time()
                        
                        # 상태 전환 기록
                        if previous_status == "대기중" and current_status == "작업중":
                            self.status_transitions[task_id] = {
                                'from': '대기중',
                                'to': '작업중',
                                'time': transition_time,
                                'display_until': transition_time + 3.0
                            }
                            self.add_log(f"📋 작업 상태 변화: {task_id} - 대기중 → 작업중", "INFO")
                    
                    self.queue_tree.insert('', 'end', values=(
                        f"{status_icon} {priority_text}", f"{status_text} {task.id}", robot_name, 
                        task.destination, item_text, estimated_time
                    ), tags=(tag,))
                    working_count += 1
            
            # 대기 중인 작업 추가
            waiting_count = 0
            queue_length = len(self.task_processor.global_task_queue)
            
            # 대기열 길이에 변화가 있을 때만 로그 기록
            if queue_length != self.last_queue_length:
                if queue_length > 0:
                    self.add_log(f"📋 GUI: {queue_length}개 작업 대기 중", "INFO")
                else:
                    self.add_log(f"📋 GUI: 대기열 비어있음", "INFO")
                self.last_queue_length = queue_length
            
            for i, task in enumerate(self.task_processor.global_task_queue, 1):
                # 이미 진행중인 작업은 대기열에서 제외 (중복 방지)
                is_currently_working = False
                for robot_name, robot in self.robot_manager.robots.items():
                    if robot.current_task and robot.current_task.id == task.id:
                        is_currently_working = True
                        break
                
                if is_currently_working:
                    continue  # 이미 진행중인 작업은 건너뛰기
                
                priority_icons = {
                    "LOW": "🔵",
                    "NORMAL": "🟡", 
                    "HIGH": "🔴",
                    "URGENT": "⚡"
                }
                
                priority_text = f"{priority_icons.get(task.priority.name, '⚪')} {task.priority.name}"
                item_text = task.item if task.item else "-"
                estimated_time = f"{task.estimated_duration:.0f}초"
                
                robot_text = task.robot_name if task.robot_name else "미할당"
                
                # 작업 상태 상세 표시 (상태 전환 추적 포함)
                task_id = task.id
                previous_status = self.task_status_cache.get(task_id, None)
                current_status = "대기중"
                
                status_icon = "⏳"
                status_text = f"[대기{waiting_count + 1}]"
                
                if task.robot_name and task.robot_name in self.robot_manager.robots:
                    robot = self.robot_manager.robots[task.robot_name]
                    if not robot.current_pose:
                        status_icon = "📡"
                        status_text = "[로봇 연결 대기]"
                    elif robot.state.name == "LOW_BATTERY":
                        status_icon = "🔋"
                        status_text = "[배터리 대기]"
                
                # 상태 변화 감지 및 캐시 업데이트
                if previous_status is None:
                    self.task_status_cache[task_id] = current_status
                    # 초기 상태 전환 기록
                    self.status_transitions[task_id] = {
                        'from': None,
                        'to': '대기중',
                        'time': time.time(),
                        'display_until': time.time() + 3.0
                    }
                
                self.queue_tree.insert('', 'end', values=(
                    f"{status_icon} {priority_text}", f"{status_text} {task.id}", robot_text, 
                    task.destination, item_text, estimated_time
                ), tags=("waiting" if task.robot_name else "unassigned",))
                waiting_count += 1
            
        except Exception as e:
            self.add_log(f"❌ GUI 업데이트 오류: {str(e)}", "ERROR")
        
        # 상태 전환 표시 (대기중 → 작업중, 3초간)
        current_time = time.time()
        expired_transitions = []
        for task_id, transition in self.status_transitions.items():
            if current_time < transition['display_until']:
                time_remaining = transition['display_until'] - current_time
                from_status = transition['from'] or '신규'
                to_status = transition['to']
                
                if from_status == '대기중' and to_status == '작업중':
                    self.queue_tree.insert('', 'end', values=(
                        "🔄 전환", f"[대기중→작업중] {task_id}", "-", "-", "-", f"{time_remaining:.1f}초 후 사라짐"
                    ), tags=("transition",))
                elif from_status == '신규' and to_status == '대기중':
                    self.queue_tree.insert('', 'end', values=(
                        "🆕 신규", f"[신규→대기중] {task_id}", "-", "-", "-", f"{time_remaining:.1f}초 후 사라짐"
                    ), tags=("new_task",))
            else:
                expired_transitions.append(task_id)
        
        # 만료된 전환 정보 삭제
        for task_id in expired_transitions:
            del self.status_transitions[task_id]
        
        # 완료된 작업 3초간 표시 추가
        for task_id, completion_time in self.completed_tasks.items():
            time_remaining = 3.0 - (current_time - completion_time)
            if time_remaining > 0:
                self.queue_tree.insert('', 'end', values=(
                    "✅ 완료", f"[작업완료] {task_id}", "-", "-", "-", f"{time_remaining:.1f}초 후 사라짐"
                ), tags=("completed",))
        
        # 태그 색상 설정
        self.queue_tree.tag_configure("working", foreground="blue", background="lightblue")
        self.queue_tree.tag_configure("waiting_confirmation", foreground="orange", background="lightyellow")
        self.queue_tree.tag_configure("waiting", foreground="black")
        self.queue_tree.tag_configure("unassigned", foreground="red", background="lightyellow")
        self.queue_tree.tag_configure("completed", foreground="green", background="lightgreen")
        self.queue_tree.tag_configure("transition", foreground="purple", background="lavender")
        self.queue_tree.tag_configure("new_task", foreground="darkblue", background="lightcyan")
    
    def show_confirmation_request(self, robot_name: str, prompt: str):
        """확인 요청 표시"""
        
        prompt_texts = {
            "item_loaded": f"❓ {robot_name} 로봇팔이 물품을 픽업했나요?",
            "manual_pickup": f"❓ {robot_name} 로봇팔 픽업이 실패했습니다. 수동으로 적재하시겠습니까?",
            "item_delivered": f"❓ {robot_name} 로봇이 물품을 전달했나요?",
            "stay_or_return": f"❓ {robot_name} 로봇이 현재 위치에서 대기할까요?\n(YES: 현재 위치 대기, NO: 충전소 복귀)",
            "add_to_queue": f"❓ {robot_name} 로봇의 진행중인 업무를 대기열에 추가할까요?\n(YES: 대기열에 추가, NO: 업무 완료 처리)"
        }
        
        display_text = prompt_texts.get(prompt, f"❓ {robot_name}: {prompt}")
        
        # 텍스트 업데이트
        self.confirmation_text.config(text=display_text)
        
        # 현재 확인 요청 정보 저장
        self.current_confirmation = {
            'robot_name': robot_name,
            'prompt': prompt
        }
        self.confirmation_active = True
        
        # 버튼 활성화
        self.yes_button.config(state='normal')
        self.no_button.config(state='normal')
        
        # 로그에도 메시지 추가
        self.add_log(display_text, "WARNING")
        
        # 창을 맨 앞으로 가져오기
        self.root.lift()
        self.root.attributes('-topmost', True)
        self.root.after(100, lambda: self.root.attributes('-topmost', False))
        
        # 화면 업데이트 강제
        self.root.update_idletasks()
    
    def on_yes_click(self):
        """YES 버튼 클릭"""
        self.send_confirmation_response(True)
        
    def on_no_click(self):
        """NO 버튼 클릭"""
        self.send_confirmation_response(False)
    
    def send_confirmation_response(self, confirmed: bool):
        """확인 응답 전송 - 수정된 버전"""
        if not self.confirmation_active or not self.current_confirmation:
            return
        
        robot_name = self.current_confirmation['robot_name']
        prompt = self.current_confirmation['prompt']
        
        # ROS 메시지 전송
        msg = Confirmation()
        msg.robot_name = robot_name
        msg.prompt = prompt
        msg.confirmed = confirmed  # 실제 응답값
        
        self.robot_manager.confirmation_response_pub.publish(msg)
        
        # 응답 로그 추가
        response_text = "YES" if confirmed else "NO"
        self.add_log(f"✅ 확인 응답: {robot_name} - {response_text}", "SUCCESS")
        
        # 확인 상태 초기화 (프레임은 숨기지 않고 텍스트만 변경)
        self.confirmation_text.config(text="확인 요청이 없습니다.")
        self.confirmation_active = False
        self.current_confirmation = None
        
        # 버튼 비활성화
        self.yes_button.config(state='disabled')
        self.no_button.config(state='disabled')
        
        # task_processor의 확인 상태도 초기화
        if self.task_processor:
            self.task_processor.waiting_for_global_confirmation = False
            self.task_processor.global_confirmation_prompt = ""
            self.task_processor.confirmation_robot = ""
    
    def run(self):
        """GUI 실행"""
        self.update_display()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()
    
    def _check_for_completed_tasks(self):
        """완료된 작업 감지 및 기록"""
        # 현재 진행중인 작업들 확인
        current_task_ids = set()
        for robot_name, robot in self.robot_manager.robots.items():
            if robot.current_task:
                current_task_ids.add(robot.current_task.id)
        
        # 캐시된 작업 중에서 더 이상 진행되지 않는 작업 찾기
        for task_id, status in list(self.task_status_cache.items()):
            if status == "작업중" and task_id not in current_task_ids:
                # 작업이 완료된 것으로 판단
                if task_id not in self.completed_tasks:
                    self.completed_tasks[task_id] = time.time()
                    self.add_log(f"📋 작업 완료: {task_id} - 작업중 → 작업완료", "SUCCESS")
    
    def start_camera_monitoring(self):
        """카메라 모니터링 시작"""
        self.camera_thread = threading.Thread(target=self._monitor_and_capture_camera, daemon=True)
        self.camera_thread.start()
    
    def _monitor_and_capture_camera(self):
        """카메라 상태 모니터링 및 화면 캡처 (별도 스레드)"""
        last_device_check = 0
        device_check_interval = 5.0  # 5초마다만 디바이스 체크
        
        while True:
            try:
                current_time = time.time()
                
                # 카메라가 실행 중이면 프레임 캡처 (우선순위)
                if self.camera_running and self.camera_capture:
                    self._capture_frame()
                    time.sleep(0.066)  # ~15fps (66ms)
                    continue
                
                # 디바이스 체크 (5초마다만)
                if current_time - last_device_check >= device_check_interval:
                    last_device_check = current_time
                    camera_found = self._check_video_devices()
                    
                    # 카메라 상태 변화가 있을 때만 처리
                    if camera_found != self.camera_available:
                        self.camera_available = camera_found
                        if camera_found:
                            # 카메라 발견 시 시작 시도
                            self._try_start_camera()
                        else:
                            # 카메라 없음 - 중지
                            self._stop_camera_capture()
                
                time.sleep(1.0)  # 카메라가 없으면 1초 대기
                
            except Exception as e:
                self.add_log(f"❌ 카메라 모니터링 오류: {str(e)}", "ERROR")
                time.sleep(5.0)  # 오류 시 5초 대기
    
    def _check_video_devices(self):
        """비디오 디바이스 존재 여부만 체크 (OpenCV 사용 안함)"""
        try:
            for i in range(4):  # video0~3만 체크 (범위 축소)
                if os.path.exists(f'/dev/video{i}'):
                    return True
            return False
        except:
            return False
    
    def _try_start_camera(self):
        """카메라 시작 시도 (안전한 방식)"""
        if self.camera_running:
            return
            
        for i in range(4):  # video0~3만 시도
            if os.path.exists(f'/dev/video{i}'):
                try:
                    # OpenCV 타임아웃 설정
                    test_cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
                    test_cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 1000)  # 1초 타임아웃
                    
                    if test_cap.isOpened():
                        # 간단한 읽기 테스트
                        ret, frame = test_cap.read()
                        test_cap.release()
                        
                        if ret and frame is not None:
                            # 성공적으로 읽음 - 이 카메라로 시작
                            self._start_camera_capture(i)
                            return
                    else:
                        test_cap.release()
                        
                except Exception as e:
                    # 이 카메라는 사용 불가 - 다음 카메라 시도
                    continue
    
    def _start_camera_capture(self, camera_index):
        """카메라 캡처 시작 (안전한 방식)"""
        try:
            # 기존 카메라가 있으면 정리
            if self.camera_capture:
                self.camera_capture.release()
                self.camera_capture = None
            
            # 새 카메라 생성 (V4L2 백엔드 명시적 사용)
            self.camera_capture = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
            
            # 타임아웃 설정
            self.camera_capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 2000)
            
            if self.camera_capture.isOpened():
                # 해상도 및 FPS 설정
                self.camera_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                self.camera_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
                self.camera_capture.set(cv2.CAP_PROP_FPS, 15)
                self.camera_capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 버퍼 크기 최소화
                
                # 첫 프레임 테스트
                ret, test_frame = self.camera_capture.read()
                if ret and test_frame is not None:
                    self.camera_running = True
                    # 상태 라벨 숨기기
                    self.root.after(0, lambda: self.camera_status_label.config(text=""))
                    self.add_log(f"📹 글로벌 카메라 시작됨 (video{camera_index})", "SUCCESS")
                else:
                    raise Exception(f"카메라 {camera_index} 프레임 읽기 실패")
            else:
                raise Exception(f"카메라 {camera_index} 열기 실패")
                
        except Exception as e:
            # 오류 발생 시 안전하게 정리
            if self.camera_capture:
                try:
                    self.camera_capture.release()
                except:
                    pass
                self.camera_capture = None
            self.camera_running = False
            # 로그는 너무 자주 안 찍히도록 (5초에 한 번만)
            current_time = time.time()
            if current_time - self._last_camera_error_time > 5.0:
                self.add_log(f"❌ 카메라 시작 실패: video{camera_index}", "ERROR")
                self._last_camera_error_time = current_time
    
    def _stop_camera_capture(self):
        """카메라 캡처 중지"""
        self.camera_running = False
        if self.camera_capture:
            self.camera_capture.release()
            self.camera_capture = None
        
        # 상태 라벨 표시
        self.root.after(0, lambda: self.camera_status_label.config(text="📡 카메라 연결 없음", fg='#ff6666'))
        self.root.after(0, lambda: self.camera_display.config(image=""))
        self.add_log("📷 글로벌 카메라가 중지되었습니다", "WARNING")
    
    def _capture_frame(self):
        """프레임 캡처 및 GUI 업데이트 (안전한 방식)"""
        if not self.camera_running or not self.camera_capture:
            return
            
        try:
            # 논블로킹 방식으로 프레임 읽기
            ret, frame = self.camera_capture.read()
            
            if ret and frame is not None and frame.size > 0:
                # OpenCV BGR을 RGB로 변환
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # GUI 크기에 맞게 리사이즈 (비율 유지)
                height, width = frame.shape[:2]
                target_width = 320
                target_height = int(height * target_width / width)
                
                if target_height > 190:  # GUI 높이 제한
                    target_height = 190
                    target_width = int(width * target_height / height)
                
                # 최소 크기 보장
                if target_width < 100 or target_height < 75:
                    target_width, target_height = 320, 240
                
                frame = cv2.resize(frame, (target_width, target_height))
                
                # PIL Image로 변환
                img = Image.fromarray(frame)
                photo = ImageTk.PhotoImage(image=img)
                
                # GUI 업데이트는 메인 스레드에서 (안전하게)
                self.root.after(0, lambda p=photo: self._update_camera_display(p))
                
            else:
                # 프레임 읽기 실패 - 카메라 연결 상태 확인
                if not self.camera_capture.isOpened():
                    self._stop_camera_capture()
                    
        except Exception as e:
            # 카메라 오류 발생 - 재연결 시도
            self._stop_camera_capture()
            # 오류 로그는 5초에 한 번만
            current_time = time.time()
            if current_time - self._last_capture_error_time > 5.0:
                self.add_log(f"❌ 프레임 캡처 오류: {str(e)}", "ERROR")
                self._last_capture_error_time = current_time
    
    def _update_camera_display(self, photo):
        """카메라 화면 업데이트 (메인 스레드에서 실행)"""
        try:
            # GUI가 아직 살아있는지 확인
            if hasattr(self, 'camera_display') and self.camera_display.winfo_exists():
                self.camera_display.config(image=photo)
                self.camera_display.image = photo  # 참조 유지 (GC 방지)
        except Exception as e:
            # GUI 업데이트 오류는 로그 생략 (너무 빈번함)
            pass
    
    def on_closing(self):
        """창 닫기 처리"""
        # 카메라 캡처 중지
        self._stop_camera_capture()
        
        self.root.quit()
        self.root.destroy()