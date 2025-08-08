#!/usr/bin/env python3

import argparse
import rclpy
import threading
import time
from robot_manager import ROSARobotManager
from task_processor import ROSATaskProcessor
from gui_display import StatusDisplayGUI
# from global_camera_interface import GlobalCameraInterface  # 글로벌 카메라 연동 시 주석 해제

def main():
    """메인 함수"""
    parser = argparse.ArgumentParser(description="ROSA 로봇 관제 시스템")
    parser.add_argument("--input-only", action="store_true", 
                       help="명령 입력 전용 모드 (로그 최소화)")
    parser.add_argument("--status-only", action="store_true", 
                       help="상태 표시 전용 모드 (GUI + 확인 응답)")
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        # 로봇 관리자 생성
        robot_manager = ROSARobotManager()
        robot_manager.status_only = args.status_only
        robot_manager.input_only = args.input_only
        
        # 작업 처리기 생성
        task_processor = ROSATaskProcessor(robot_manager)
        task_processor.status_only = args.status_only
        task_processor.input_only = args.input_only
        
        # 글로벌 카메라 인터페이스 생성 (주석 처리)
        # camera_interface = GlobalCameraInterface(task_processor)  # 글로벌 카메라 연동 시 주석 해제
        
        # 상호 참조 설정
        robot_manager.set_task_processor(task_processor)
        
        if args.status_only:
            print("🖥️ ROSA 상태 표시 모드 실행")
            print("=" * 50)
            print("📊 실시간 로봇 상태 모니터링")
            print("❓ 확인 요청 GUI 응답")
            print("📄 모든 시스템 로그 표시")
            print("=" * 50)
            
            # GUI 설정 및 실행
            gui = StatusDisplayGUI(robot_manager, task_processor)
            robot_manager.set_gui(gui)
            task_processor.set_gui(gui)
            
            # ROS 스핀을 별도 스레드에서 실행
            def ros_spin():
                rclpy.spin(robot_manager)
            
            ros_thread = threading.Thread(target=ros_spin, daemon=True)
            ros_thread.start()
            
            # GUI 실행 (메인 스레드)
            gui.run()
                
        elif args.input_only:
            print("🎤 ROSA 명령 입력 모드 실행")
            print("=" * 50)
            print("💡 명령어만 입력 (상태는 별도 창에서 확인)")
            print("🤖 AI 기반 자연어 명령 처리")
            print("📵 로그 출력 최소화")
            print("=" * 50)
            
            task_processor.start_input_thread()
            rclpy.spin(robot_manager)
            
        else:
            print("🤖 ROSA 통합 모드 실행")
            print("=" * 50)
            print("📊 상태 표시 + 명령 입력")
            print("💡 모든 기능 통합 실행")
            print("=" * 50)
            
            task_processor.start_input_thread()
            time.sleep(1.0)
            task_processor.show_status()
            rclpy.spin(robot_manager)
            
    except KeyboardInterrupt:
        print("\n👋 시스템 종료")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
