oflow_3d_generator
==================

This ROS node receives a sequence of disparity and visual images, and computes their 3D optical flow using a KLT based approach. 

#### Subscribed Topics

- <stereo>/left/<image> ([sensor_msgs/Image](www.github.com)): Left rectified stereo image.
- <stereo>/disparity/<image> (sensor_msgs/Image): Image representing the calculated disparity at each point. Note that the values are scaled to be in the range [0, max_disparity]
- <stereo>/left/camera_info (sensor_msgs/CameraInfo): Camera info for left camera.
- <stereo>/right/camera_info (sensor_msgs/CameraInfo): Camera info for right camera.

#### Published Topics

- flow_vectors ([sensor_msgs/PointCloud2](www.github.com)): The point cloud assosciated with the calculated disparity in the left camera's local reference frame.

#### Parameters

~approximate_sync (bool, default: false): Whether the node should use approximate synchronization of incoming messages. Set to true if cameras do not have synchronised timestamps.
