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





https://github.com/Carpediem324/nanosaur_robotprogramming/assets/101110020/e20eb1de-4814-4ea4-95f9-7db27b503985





### 오류


--- stderr: nanosaur_camera                         
CMake Error at CMakeLists.txt:24 (find_package):
  By not providing "Findjetson-utils.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "jetson-utils", but CMake did not find one.

  Could not find a package configuration file provided by "jetson-utils" with
  any of the following names:

    jetson-utilsConfig.cmake
    jetson-utils-config.cmake

  Add the installation prefix of "jetson-utils" to CMAKE_PREFIX_PATH or set
  "jetson-utils_DIR" to a directory containing one of the above files.  If
  "jetson-utils" provides a separate development package or SDK, be sure it
  has been installed.


---
