#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl_ros/publisher.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
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

  void subCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if(msg->data.empty())
    {
      ROS_WARN("Received an empty cloud message. Skipping further processing");
      return;
    }

    ROS_INFO_STREAM("Received a cloud message with " << msg->height * msg->width << " points");
    ROS_INFO("Converting ROS cloud message to PCL compatible data structure");
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(*msg, pclCloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr p(new pcl::PointCloud<pcl::PointXYZ>(pclCloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.setInputCloud(p);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    ROS_INFO("Filtering cloud data");
    sor.filter(*pclCloud_filtered);
    ROS_INFO_STREAM("Filtering completed. a cloud message with " << (msg->height*msg->width - pclCloud_filtered->height*pclCloud_filtered->width) << " points as outliers leaving " << pclCloud_filtered->height * pclCloud_filtered->width << " in total");
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
