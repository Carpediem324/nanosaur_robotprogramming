GST_ARGUS: Creating output stream
CONSUMER: Waiting until producer is connected...
GST_ARGUS: Available Sensor modes :
GST_ARGUS: 3264 x 2464 FR = 21.000000 fps Duration = 47619048 ; Analog Gain range min 1.000000, max 10.625000; Exposure Range min 13000, max 683709000;

GST_ARGUS: 3264 x 1848 FR = 28.000001 fps Duration = 35714284 ; Analog Gain range min 1.000000, max 10.625000; Exposure Range min 13000, max 683709000;

GST_ARGUS: 1920 x 1080 FR = 29.999999 fps Duration = 33333334 ; Analog Gain range min 1.000000, max 10.625000; Exposure Range min 13000, max 683709000;

GST_ARGUS: 1640 x 1232 FR = 29.999999 fps Duration = 33333334 ; Analog Gain range min 1.000000, max 10.625000; Exposure Range min 13000, max 683709000;

GST_ARGUS: 1280 x 720 FR = 59.999999 fps Duration = 16666667 ; Analog Gain range min 1.000000, max 10.625000; Exposure Range min 13000, max 683709000;

GST_ARGUS: 1280 x 720 FR = 120.000005 fps Duration = 8333333 ; Analog Gain range min 1.000000, max 10.625000; Exposure Range min 13000, max 683709000;

GST_ARGUS: Running with following settings:
   Camera index = 0 
   Camera mode  = 5 
   Output Stream W = 1280 H = 720 
   seconds to Run    = 0 
   Frame Rate = 120.000005 
GST_ARGUS: Setup Complete, Starting captures for 0 seconds
GST_ARGUS: Starting repeat capture requests.
CONSUMER: Producer has connected; continuing.
[ WARN:0] global /home/nvidia/host/build_opencv/nv_opencv/modules/videoio/src/cap_gstreamer.cpp (933) open OpenCV | GStreamer warning: Cannot query video position: status=0, value=-1, duration=-1
Traceback (most recent call last):
  File "/home/ubuntu/ros2_ws/install/line3/lib/line3/line3", line 11, in <module>
    load_entry_point('line3', 'console_scripts', 'line3')()
  File "/home/ubuntu/ros2_ws/build/line3/line3/line3.py", line 145, in main
    rclpy.spin(line_follower)
  File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/__init__.py", line 190, in spin
    executor.spin_once()
  File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/executors.py", line 684, in spin_once
    raise handler.exception()
  File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/task.py", line 239, in __call__
    self._handler.send(None)
  File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/executors.py", line 404, in handler
    await call_coroutine(entity, arg)
  File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/executors.py", line 321, in _execute_timer
    await await_or_execute(tmr.callback)
  File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/executors.py", line 118, in await_or_execute
    return callback(*args)
  File "/home/ubuntu/ros2_ws/build/line3/line3/line3.py", line 54, in timer_callback
    linear_velocity, angular_velocity = self.process_image_and_move(img)
  File "/home/ubuntu/ros2_ws/build/line3/line3/line3.py", line 68, in process_image_and_move
    move = control_nanosaur(points, width)
  File "/home/ubuntu/ros2_ws/build/line3/line3/line3.py", line 131, in control_nanosaur
    if point['x'] >= frame_width * 2/3:
TypeError: string indices must be integers
GST_ARGUS: Cleaning up
CONSUMER: Done Success
GST_ARGUS: Done Success