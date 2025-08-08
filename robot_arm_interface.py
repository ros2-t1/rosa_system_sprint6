#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String, Int32
import time
from datetime import datetime

# 1단계에서 만들었던 config 파일에서 설정값들을 가져옵니다.
from config import ARM_TOPICS, ITEM_ARUCO_MAP, ROBOT_ARUCO_MAP
from robot_manager import RobotInfo # robot_manager의 RobotInfo 클래스를 가져옵니다.

class RobotArmInterface:
    """로봇팔(HANA Arm)과의 ROS 토픽 통신을 관리하는 클래스"""

    def __init__(self, node, task_processor):
        """
        노드 초기화 및 퍼블리셔, 서브스크라이버 설정
        """
        self.node = node
        self.task_processor = task_processor # task_processor 참조를 저장합니다.
        
        # --- config.py의 설정 사용 ---
        # 1. 서버 -> 로봇팔: 어떤 물품(ArUco ID)을 집어달라고 요청하는 퍼블리셔
        self.pick_request_pub = self.node.create_publisher(
            Int32,
            ARM_TOPICS['aruco_id_to_arm'],
            10)

        # 2. 로봇팔 -> 서버: 물품을 로봇에 올려놓은 후 완료했다는 신호를 받는 서브스크라이버
        self.place_completed_sub = self.node.create_subscription(
            String,
            ARM_TOPICS['arm_place_completed'],
            self._completed_callback,
            10)
            
        self.node.get_logger().info("🦾 로봇팔 인터페이스 초기화 완료")

    def request_pickup(self, robot: RobotInfo, item_name: str) -> bool:
        """
        로봇팔에게 특정 아이템 픽업을 요청합니다.
        
        Args:
            robot (RobotInfo): 물건을 받을 로봇 객체
            item_name (str): 집어야 할 물품의 이름

        Returns:
            bool: 요청 성공 여부
        """
        # config에서 ArUco ID 가져오기
        # item_name에 해당하는 ID가 없으면 첫 번째 ID를 기본값으로 사용
        aruco_ids = ITEM_ARUCO_MAP.get(item_name)
        if not aruco_ids:
            self.node.get_logger().error(f"config에 '{item_name}'의 ArUco ID가 없습니다!")
            return False
            
        # 첫 번째 ArUco ID를 전송
        aruco_id_to_send = aruco_ids[0]
        
        msg = Int32()
        msg.data = int(aruco_id_to_send)
        
        self.pick_request_pub.publish(msg)
        self.node.get_logger().info(f"🦾 -> 로봇팔: '{item_name}'(ArUco ID: {msg.data}) 픽업 요청 전송")
        
        return True

    def _completed_callback(self, msg: String):
        """
        로봇팔로부터 작업 완료 메시지를 수신하는 콜백 함수
        """
        if msg.data.lower() == 'completed':
            self.node.get_logger().info("✅ <- 로봇팔: 적재 완료 메시지 수신!")
            
            # 메인 로직(TaskProcessor)에 작업이 완료되었음을 알립니다.
            self.task_processor.confirmation_manager.handle_arm_pickup_completion()