oflow_3d_generator
==================

This ROS node receives a sequence of disparity and visual images, and computes their 3D optical flow using a KLT based approach. 

#### Subscribed Topics

- <stereo>/left/<image> ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)): Left rectified stereo image.
- <stereo>/disparity/<image> ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)): Image representing the calculated disparity at each point. Note that the values are scaled to be in the range [0, max_disparity]
- <stereo>/left/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)): Camera info for left camera.
- <stereo>/right/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)): Camera info for right camera.

#### Published Topics

- flow_vectors ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)): The point cloud assosciated with the calculated disparity in the left camera's local reference frame. Motion vectors will be represented by points of type [pcl::PointXYZRGBNormal](http://docs.pointclouds.org/1.7.0/structpcl_1_1_point_x_y_z_r_g_b_normal.html).

#### Parameters

- approximate_sync (bool, default: false): Whether the node should use approximate synchronization of incoming messages. Set to true if cameras do not have synchronised timestamps.
