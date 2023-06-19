# nanosaur_robotprogramming

step1: make package with dependencies

- ros2 pkg create linetrace --build-type ament_python --dependencies sensor_msgs geometry_msgs cv_bridge opencv-python rclpy

step2: move to linetrace.py in your package

- /linetrace/linetrace/linetrace.py

step3: change entrypoints

- entry_points={
        'console_scripts': [
							'linetrace = linetrace.linetrace:main',
        ],
        
step4: build before your package

- colcon build 

step5: 

- . install/setup.bash

step6: run your node

- ros2 run linetrace linetrace 

### linetrace 코드 설명
nanosaur가 카메라를 사용하여 선을 따라 이동하는 데 도움이 되도록 하는 라인 팔로워를 구현하기 위한 코드입니다.
이 코드는 로봇 운영 체제 2(ROS2)를 활용하여 진행되었으며, 필요한 패키지로는 rclpy, sensor_msgs, geometry_msgs, cv2, numpy, 그리고 cv_bridge 등이 활용되었습니다. 

본 코드의 기능은 다음과 같습니다:

- LineFollower 클래스를 정의하고 생성자를 통해 이미지 처리와 관련된 토픽을 다루는 발행자와 구독자를 초기화합니다.
- 이때 발행자와 구독자는 각각 cmd_vel 및 camera/image_raw 토픽과 관련됩니다.
- 이미지 콜백 함수에서 이미지를 처리하여 라인을 인식하고 따라가는 기능을 구현합니다.이를 위해 흑백 이미지로 변환한 후, 임계값을 적용하여 이진화 처리를 합니다.
- 로봇이 라인을 중앙에 두고 올바른 방향으로 이동하기 위해 에러를 계산합니다. 중심 축 오차는 타겟 값에서 현재 컨투어의 중심 위치를 빼서 얻습니다.
- 각속도를 계산한 후, Twist 메시지를 생성해 로봇 바퀴에 전달합니다. 선을 벗어나는 경우, 로봇이 안전하게 정지하도록 처리합니다.
- 코드 실행 시, LineFollower 인스턴스를 생성하고 rclpy.spin을 사용하여 지속적으로 작동합니다.
- 사용자가 종료하거나 ESC를 누를 경우, 로봇은 이동을 중단하며 이 정보를 다른 노드로 알립니다.

![캡처 001](https://github.com/Carpediem324/nanosaur_robotprogramming/assets/128462226/ce373b57-8461-491b-8773-3e78efc5cbef)

![캡처 002](https://github.com/Carpediem324/nanosaur_robotprogramming/assets/128462226/d3f5284c-8f6c-44bb-95bf-0ac7638a96d5)

---

### 시뮬레이션 영상

https://github.com/Carpediem324/nanosaur_robotprogramming/assets/101110020/e20eb1de-4814-4ea4-95f9-7db27b503985
