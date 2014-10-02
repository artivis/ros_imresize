ROS_imresize
============

Resize an image topic and update its camera info ( K, width, height ).

Can also undistord image prior to resize, it then update D to null.

So far handle only 'plumb_bob' distortion model.

ROS wiki about camera info :
http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
