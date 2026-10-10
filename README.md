# Mirobot_ros2

## 간단한 소개

미로봇 ROS2 기능을 분리하기 전의 통합 소스를 보관하는 저장소이다.
기능별 설치와 사용을 위해 제어·양팔·비전·주문 처리 소스를 네 저장소로 분리한다.
신규 사용자는 목적에 맞는 아래 저장소를 사용한다.

## 권장 실행 환경

Ubuntu 22.04, ROS2 Humble, Python 3.10을 권장한다.
기능별 저장소의 검증 범위는 각 문서에서 확인하며, 해당 결과가 이 원본 통합 코드 전체의 동작을 보증하지 않는다.

## 패키지 구조

| 원본 패키지 | 기능별 저장소 | 역할 |
| --- | --- | --- |
| `mirobot_description` + `mirobot_driver` | [mirobot_control_ros2](https://github.com/Jeong-Yun-Kim/mirobot_control_ros2) | 로봇 모델과 단일 로봇 제어를 담당한다. |
| `mirobot_master_slave` | [mirobot_dual_arm_ros2](https://github.com/Jeong-Yun-Kim/mirobot_dual_arm_ros2) | 두 로봇의 복제·순차·왕복 제어를 담당한다. |
| `vision_robot` | [mirobot_vision_ros2](https://github.com/Jeong-Yun-Kim/mirobot_vision_ros2) | 카메라 인식·좌표 변환·큐브 분류·적층을 담당한다. |
| `mirobot_order_delivery` | [mirobot_order_delivery_ros2](https://github.com/Jeong-Yun-Kim/mirobot_order_delivery_ros2) | 로봇 측 주문·피킹·상태 응답을 담당한다. |

원본 `vision_robot/vision_robot/order_yolo_sequencer.py`는 두 로봇을 제어하므로 양팔 저장소의 `mirobot_master_slave` 패키지로 이동한다.
인식 결과를 발행하는 `vision_cubes_publisher.py`는 비전 저장소에 유지한다.

## 빌드 및 실행 방법

신규 설치는 사용할 기능의 README에 따라 의존성·설정·빌드·실행을 준비한다.

- 로봇 모델 표시와 한 대의 관절·XYZ 제어는 [control 설치 문서](https://github.com/Jeong-Yun-Kim/mirobot_control_ros2#빌드-및-실행-방법)를 따른다.
- 두 로봇의 연결과 동작 시퀀스는 [dual_arm 설치 문서](https://github.com/Jeong-Yun-Kim/mirobot_dual_arm_ros2#빌드-및-실행-방법)를 따른다.
- 카메라·YOLO 인식과 분류·적층은 [vision 설치 문서](https://github.com/Jeong-Yun-Kim/mirobot_vision_ros2#빌드-및-실행-방법)를 따른다.
- 주문 기반 피킹과 domain_bridge 연결은 [order_delivery 설치 문서](https://github.com/Jeong-Yun-Kim/mirobot_order_delivery_ros2#빌드-및-실행-방법)를 따른다.

실제 장비와 모델·보정 파일의 준비 조건은 선택한 문서에서 확인한다.
원본 통합 소스는 보관 목적으로 유지하며 기능별 저장소의 수정 사항을 이 원본 코드에 자동 반영하지 않는다.
