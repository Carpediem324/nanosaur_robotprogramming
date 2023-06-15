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

