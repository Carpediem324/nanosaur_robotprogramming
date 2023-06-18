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

Traceback (most recent call last):
  File "/home/ubuntu/ros2_ws/install/nanosaur_hardware/lib/nanosaur_hardware/nanosaur", line 11, in <module>
    load_entry_point('nanosaur-hardware', 'console_scripts', 'nanosaur')()
  File "/home/ubuntu/ros2_ws/build/nanosaur_hardware/nanosaur_hardware/nanosaur.py", line 194, in main
    rclpy.spin(nanosaur)
  File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/__init__.py", line 190, in spin
    executor.spin_once()
  File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/executors.py", line 684, in spin_once
    raise handler.exception()
  File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/task.py", line 239, in __call__
    self._handler.send(None)
  File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/executors.py", line 404, in handler
    await call_coroutine(entity, arg)
  File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/executors.py", line 330, in _execute_subscription
    await await_or_execute(sub.callback, msg)
  File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/executors.py", line 118, in await_or_execute
    return callback(*args)
  File "/home/ubuntu/ros2_ws/build/nanosaur_hardware/nanosaur_hardware/nanosaur.py", line 173, in drive_callback
    self.mright.set_speed(rpmr)
  File "/home/ubuntu/ros2_ws/build/nanosaur_hardware/nanosaur_hardware/motor.py", line 48, in set_speed
    self._motor.setSpeed(speed)
  File "/usr/local/lib/python3.6/dist-packages/Adafruit_MotorHAT/Adafruit_MotorHAT_Motors.py", line 213, in setSpeed
    self.MC._pwm.setPWM(self.PWMpin, 0, speed*16)
  File "/usr/local/lib/python3.6/dist-packages/Adafruit_MotorHAT/Adafruit_PWM_Servo_Driver.py", line 88, in setPWM
    self.i2c.write8(self.__LED0_ON_L+4*channel, on & 0xFF)
  File "/usr/local/lib/python3.6/dist-packages/Adafruit_GPIO/I2C.py", line 114, in write8
    self._bus.write_byte_data(self._address, register, value)
  File "/usr/local/lib/python3.6/dist-packages/Adafruit_PureIO/smbus.py", line 316, in write_byte_data
    self._device.write(data)
OSError: [Errno 121] Remote I/O error

