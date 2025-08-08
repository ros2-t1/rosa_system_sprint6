# 🤖 ROSA (Robot Operation & Scheduling Assistant) 다중 로봇 관제 시스템 (2025년 8월 8일 업데이트)

## 1. 시스템 개요

ROSA는 3대의 로봇(DP_03, DP_08, DP_09)을 효율적으로 제어하고 작업을 할당하기 위한 중앙 관제 시스템입니다. ROS 2를 기반으로 하며, 다음과 같은 핵심 기능을 제공합니다.

- **다중 로봇 동시 제어**: 3대의 로봇을 단일 시스템에서 모니터링하고 제어합니다.
- **지능형 작업 스케줄링**: AI 기반 스케줄러가 로봇의 현재 상태(위치, 배터리, 작업 현황)를 분석하여 최적의 로봇에게 작업을 할당합니다.
- **자연어 명령 인터페이스**: "왼쪽방에 물 배달해줘"와 같은 일상적인 언어로 로봇에게 명령을 내릴 수 있습니다.
- **실시간 GUI 모니터링**: `Tkinter`로 제작된 GUI를 통해 모든 로봇의 상태, 작업 대기열, 시스템 로그를 실시간으로 확인할 수 있습니다.
- **분리된 실행 모드**: 명령을 입력하는 터미널(`--input-only`)과 상태를 확인하는 GUI(`--status-only`)를 분리하여 운영 편의성을 극대화했습니다.
- **중앙 집중식 TF 관리**: 각 로봇의 TF(Transform) 데이터를 중앙에서 집계하여 RViz 등에서 전체 로봇의 위치를 한 번에 시각화할 수 있습니다.

---

## 2. 주요 업데이트 및 기능 개선 (2025년 8월 8일)

이번 업데이트에서는 시스템의 모듈성과 확장성을 높이고, 로봇팔 연동 기능을 추가하는 데 중점을 두었습니다.

-   **로봇팔(HANA Arm) 연동**:
    *   로봇팔과의 ROS 토픽 기반 통신이 구현되었습니다.
    *   `config.py`에 로봇팔 연동을 위한 `ARM_TOPICS` 및 `ITEM_ARUCO_MAP` 설정이 추가되었습니다.
    *   `robot_arm_interface.py`를 통해 로봇팔에게 물품 픽업 요청을 보내고, 로봇팔의 작업 완료 신호를 수신합니다.
    *   **사용 토픽**:
        *   `server(20) -> arm(14)`: `/robot_arm/user_cmd` (std_msgs/msg/Int32) - 로봇팔에게 픽업할 물품의 ArUco ID를 전달
        *   `arm(14) -> server(20)`: `/robot_arm/status` (std_msgs/msg/String) - 로봇팔의 작업 완료 신호를 수신 ("completed" 문자열)

-   **코드 리팩토링 및 모듈 분리**:
    *   기존 `task_processor.py`의 복잡한 로직을 `task_executor.py` (실제 작업 수행)와 `confirmation_manager.py` (사용자 확인 처리)로 분리하여 코드의 가독성, 유지보수성 및 역할 분담을 명확히 했습니다.
    *   `task_processor.py`는 이제 작업 흐름의 '지휘자' 역할을 수행하며, 실제 실행 및 확인은 각 매니저에게 위임합니다.

-   **GUI 개선**:
    *   `gui_display.py`에 글로벌 카메라 화면 표시 기능이 추가되었습니다.
    *   작업 상태 전환 및 완료된 작업에 대한 상세한 추적 및 시각화 기능이 강화되었습니다.

-   **웨이포인트 기능 (현재 비활성화)**:
    *   Nav2 웨이포인트 주행 기능 통합을 위한 코드는 포함되어 있으나, 현재 `main.py` 실행 시에는 해당 기능이 비활성화되어 있습니다. 로봇 이동은 기존 `move_robot_to_location`을 통해 이루어집니다.

---

## 3. 시스템 아키텍처 및 요구사항

-   **네트워크 설정**
    -   **관제 PC**: `ROS_DOMAIN_ID=20`
    -   **로봇 DP_03**: `ROS_DOMAIN_ID=13`
    -   **로봇 DP_08**: `ROS_DOMAIN_ID=18`
    -   **로봇 DP_09**: `ROS_DOMAIN_ID=19`
    -   **로봇팔 (HANA Arm)**: `ROS_DOMAIN_ID=14`

-   **필요 ROS 2 패키지**
    -   `nav2_map_server`, `domain_bridge`, `rf2o_laser_odometry`, `pinky_bringup`, `pinky_navigation`, `rviz2`
    -   `pinky_interfaces` (ROS 메시지 정의)

---

## 4. 시스템 실행 절차

시스템을 올바르게 실행하려면 **관제 PC**와 **각 로봇 PC**에서 아래 순서대로 명령을 실행해야 합니다.

### Ⅰ. 관제 PC 설정 (Domain ID: 20)

**총 3개의 터미널**을 열고 아래 명령어를 각각 실행합니다.

#### 터미널 1: Map 서버 실행

```bash
# 관제 PC의 도메인 ID 설정
export ROS_DOMAIN_ID=20

# Map 서버를 백그라운드에서 실행합니다.
# 맵 파일 경로는 시스템에 맞게 확인해야 합니다.
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/addinedu/map_1753257471.yaml &
```

#### 터미널 2: Domain Bridge 실행

```bash
# 관제 PC의 도메인 ID 설정
export ROS_DOMAIN_ID=20

# Domain Bridge를 백그라운드에서 실행하여 로봇들과의 통신을 연결합니다.
# 로봇팔 연동을 위한 토픽 설정이 0808_domain.yaml에 포함되어 있습니다.
ros2 run domain_bridge domain_bridge /home/addinedu/jeong/multi_robot_project/0808_system/0808_domain.yaml &
```

#### 터미널 3: TF Aggregator 실행

```bash
# 관제 PC의 도메인 ID 설정
export ROS_DOMAIN_ID=20

# 각 로봇의 TF 데이터를 통합하는 TF 중계기를 실행합니다.
# 이 스크립트가 있는 디렉토리에서 실행해야 합니다.
# (TF Aggregator 스크립트가 필요하다면 해당 경로를 확인해주세요.)
# 예시: python3 /path/to/your/tf_aggregator.py
```

---

### Ⅱ. 각 로봇 PC 설정 (DP_03, DP_08, DP_09)

**3대의 로봇 각각에서** 아래의 절차를 수행합니다. 로봇별로 **3개의 터미널**이 필요합니다.

#### 로봇 1: DP_03 (Domain ID: 13)

```bash
# --- 터미널 1: RF2O Odometry 실행 ---
export ROS_DOMAIN_ID=13
ros2 launch rf2o_laser_odometry 0731_rf2o.launch.py robot_name:=DP_03

# --- 터미널 2: 로봇 Bringup 실행 ---
export ROS_DOMAIN_ID=13
# Note: 패키지를 찾지 못하는 경우, 아래와 같이 절대 경로로 실행해야 합니다.
ros2 launch ~/pinky_violet/install/pinky_bringup/share/pinky_bringup/launch/0731_bringup.launch.xml robot_name:=DP_03

# --- 터미널 3: Nav2 실행 ---
export ROS_DOMAIN_ID=13
ros2 launch pinky_navigation 0731_nav2.launch.xml namespace:=DP_03
```

#### 로봇 2: DP_08 (Domain ID: 18)

```bash
# --- 터미널 1: RF2O Odometry 실행 ---
export ROS_DOMAIN_ID=18
ros2 launch rf2o_laser_odometry 0731_rf2o.launch.py robot_name:=DP_08

# --- 터미널 2: 로봇 Bringup 실행 ---
export ROS_DOMAIN_ID=18
ros2 launch ~/pinky_violet/install/pinky_bringup/share/pinky_bringup/launch/0731_bringup.launch.xml robot_name:=DP_08

# --- 터미널 3: Nav2 실행 ---
export ROS_DOMAIN_ID=18
ros2 launch pinky_navigation 0731_nav2.launch.xml namespace:=DP_08
```

#### 로봇 3: DP_09 (Domain ID: 19)

```bash
# --- 터미널 1: RF2O Odometry 실행 ---
export ROS_DOMAIN_ID=19
ros2 launch rf2o_laser_odometry 0731_rf2o.launch.py robot_name:=DP_09

# --- 터미널 2: 로봇 Bringup 실행 ---
export ROS_DOMAIN_ID=19
ros2 launch ~/pinky_violet/install/pinky_bringup/share/pinky_bringup/launch/0731_bringup.launch.xml robot_name:=DP_09

# --- 터미널 3: Nav2 실행 ---
export ROS_DOMAIN_ID=19
ros2 launch pinky_navigation 0731_nav2.launch.xml namespace:=DP_09
```

---

### Ⅲ. 로봇팔 PC 설정 (HANA Arm, Domain ID: 14)

로봇팔이 연결된 PC에서 로봇팔 제어 프로그램을 실행합니다. 이 프로그램은 `/robot_arm/user_cmd` 토픽을 구독하고 `/robot_arm/status` 토픽을 발행해야 합니다.

```bash
# 로봇팔 PC의 도메인 ID 설정
export ROS_DOMAIN_ID=14

# 로봇팔 제어 프로그램 실행 (예시)
# ros2 run your_arm_package arm_control_node
```

---

### Ⅳ. 관제 프로그램 실행 (관제 PC)

모든 로봇 및 로봇팔의 실행이 완료된 후, 관제 PC에서 **추가로 3개의 터미널**을 열어 아래 프로그램들을 실행합니다.

#### 터미널 4: RViz2 시각화 실행

```bash
# 관제 PC의 도메인 ID 설정
export ROS_DOMAIN_ID=20

# RViz를 실행하여 로봇 상태를 시각적으로 모니터링합니다.
# (미리 설정된 rviz 설정 파일을 -d 옵션으로 불러오는 것을 권장합니다)
rviz2
```

#### 터미널 5: 상태/확인 GUI 실행 (`--status-only`)

```bash
# 관제 PC의 도메인 ID 설정
export ROS_DOMAIN_ID=20

# GUI를 실행합니다.
python3 /home/addinedu/jeong/multi_robot_project/0808_system/main.py --status-only
```

#### 터미널 6: 명령 입력 터미널 실행 (`--input-only`)

```bash
# 관제 PC의 도메인 ID 설정
export ROS_DOMAIN_ID=20

# 자연어 명령을 입력할 터미널을 실행합니다.
python3 /home/addinedu/jeong/multi_robot_project/0808_system/main.py --input-only
```

---

## 5. 사용법

모든 실행이 완료되면 **관제 PC의 터미널 6 (`--input-only`)** 에서 다음과 같은 자연어 명령을 내릴 수 있습니다.

-   **단순 이동 (AI 로봇 선택)**: `오른쪽방 가`
-   **단순 이동 (로봇 지정)**: `3번 로봇 왼쪽방 가`
-   **배달 (AI 로봇 선택)**: `면회실에 물 배달해줘`
-   **배달 (로봇 지정)**: `8번 로봇이 출입구로 서류 배달`
-   **충전소 이동**: `9번 로봇 충전소 가`
-   **복귀 (업무 할당 가능 상태로 대기)**: `3번 로봇 복귀해`

명령이 내려지면 시스템은 최적의 로봇을 판단하여 작업을 할당하고, **터미널 5 (`--status-only`)** 의 GUI와 **터미널 4 (RViz2)** 에 모든 진행 상황이 실시간으로 표시됩니다.

---
