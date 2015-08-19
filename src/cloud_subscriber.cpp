#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/publisher.h>
#include <string>

class CloudSubscriber
{
protected:
  ros::NodeHandle nh;
  ros::Subscriber sub;

private:
  sensor_msgs::PointCloud2 cloud;

public:
  CloudSubscriber()
  {
    sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud", 5, &CloudSubscriber::subCallback, this);
  }

  ~CloudSubscriber()
  {
    sub.shutdown();
  }

  void subCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    ROS_INFO_STREAM("Received a cloud message with " << msg->height * msg->width << " points");
  }
};

int main(int argc, char* argv[])
{
  ros::init (argc, argv, "cloud_subscriber");
  ros::NodeHandle nh;

  CloudSubscriber c;

  while(nh.ok())
    ros::spin();

  return 0;
}
