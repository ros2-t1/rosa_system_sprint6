#!/usr/bin/env python3

"""ROSA 시스템 설정 파일"""

# 로봇 기본 설정
ROBOT_NAMES = ['DP_03', 'DP_08', 'DP_09']
BATTERY_THRESHOLD = 40.0
ARRIVAL_DISTANCE_THRESHOLD = 0.25  # 미터

# 위치 좌표 (미터 단위)
LOCATIONS = {
    '왼쪽방': (0.0, 0.9),
    '오른쪽방': (0.4, 0.9),
    '픽업대': (0.15, -0.4),
    '3번 충전소': (-0.2, 0.0),
    '8번 충전소': (-0.2, 0.25),
    '9번 충전소': (-0.2, 0.5),
    '면회실': (0.5, 0.0),
    '출입구': (0.5, 0.2),
    '픽업대기장소': (-0.2, -0.4),
}

# 로봇별 충전소 매핑
ROBOT_CHARGE_STATIONS = {
    'DP_03': '3번 충전소',
    'DP_08': '8번 충전소', 
    'DP_09': '9번 충전소',
}

# ROS 토픽 이름
TOPICS = {
    'robot_status': '/rosa/robot_status',
    'log_messages': '/rosa/log_messages',
    'confirmation_request': '/rosa/confirmation_request',
    'confirmation_response': '/rosa/confirmation_response',
    'task_queue': '/rosa/task_queue',
}

# GUI 설정
GUI_COLORS = {
    'bg_main': '#2b2b2b',
    'bg_frame': '#3b3b3b',
    'text_title': '#00ff88',
    'text_section': '#ffaa00',
    'text_normal': '#ffffff',
}

# 타이밍 설정 (초 단위)
TIMERS = {
    'robot_status_check': 2.0,
    'debug_status_check': 30.0,
    'status_publish': 1.0,
    'command_process': 0.1,
    'task_process': 1.0,
    'queue_optimize': 10.0,
}

# 물품별 ArUco ID (로봇팔 연동용)
ITEM_ARUCO_MAP = {
    "도시락": [4, 5],
    "물": [6, 7],
    "영양제": [8 ,9],
}

# 로봇팔 연동용 토픽 이름 (새 이름으로 수정됨)
ARM_TOPICS = {
    # 서버 -> 로봇팔: 어떤 물품(ArUco ID)을 집어달라고 요청하는 토픽
    'aruco_id_to_arm': '/robot_arm/user_cmd',

    # 로봇팔 -> 서버: 물품을 로봇에 올려놓은 후 완료했다는 신호를 보내는 토픽
    'arm_place_completed': '/robot_arm/status',
}

# 로봇별 ArUco ID (로봇팔 연동용)
ROBOT_ARUCO_MAP = {
    "DP_03": 203,
    "DP_08": 208, 
    "DP_09": 209,
}