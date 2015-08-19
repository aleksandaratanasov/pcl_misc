Various ROS nodes related to the PCL library

The material here is meant for quickly adding barebone functionality to existing projects

Currently following are available:
- **Cloud publisher**: a simple version of a ROS publisher, which reads from a PCD file and publishes a ROS-compatible point cloud message
- **Cloud subscriber**: a simple version of a ROS publisher, which subscribes to the topic of the **cloud publisher** and applies statistical outlier removal
