#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/publisher.h>
#include <string>

class CloudPublisher
{
protected:
  std::string tf_frame;
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

public:
  sensor_msgs::PointCloud2 cloud;
  std::string file_name, cloud_topic;
  pcl_ros::Publisher<sensor_msgs::PointCloud2> pub;

  CloudPublisher()
    : tf_frame("/base_link"),
      private_nh("~")
  {
    cloud_topic = "cloud";
    pub.advertise(nh, cloud_topic.c_str(), 1);
    private_nh.param("frame_id", tf_frame, std::string("/base_link"));
    ROS_INFO_STREAM("Publishing data on topic \"" << nh.resolveName(cloud_topic) << "\" with frame_id \"" << tf_frame << "\"");
  }

  int start()
  {
    if (file_name == "" || pcl::io::loadPCDFile(file_name, cloud) == -1)
      return (-1);
    cloud.header.frame_id = tf_frame;
    return (0);
  }

  bool spin ()
  {
    int nr_points = cloud.width * cloud.height;
    std::string fields_list = pcl::getFieldsList(cloud);
    ros::Rate r(ros::Duration(1,0)); //1s tact

    while(nh.ok ())
    {
      ROS_DEBUG_STREAM_ONCE("Publishing data with " << nr_points
                            << " points " << fields_list
                            << " on topic \"" << nh.resolveName(cloud_topic)
                            << "\" in frame \"" << cloud.header.frame_id << "\"");
      cloud.header.stamp = ros::Time::now();
      pub.publish(cloud);

      r.sleep();
    }
    return (true);
  }
};

int main(int argc, char* argv[])
{
  ros::init (argc, argv, "cloud_publisher");
  ros::NodeHandle nh;

  CloudPublisher c;
  nh.getParam("pcdFile", c.file_name);

  if (c.start () == -1)
  {
    ROS_ERROR_STREAM("Could not load file \"" << c.file_name  <<"\". Exiting...");
    return (-1);
  }
  ROS_INFO_STREAM("Loaded a point cloud with " << c.cloud.width * c.cloud.height
                  << " points (total size is " << c.cloud.data.size() << ") and the following channels: " << pcl::getFieldsList (c.cloud));
  c.spin();

  return 0;
}

/*
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "cloud_publisher");
  ros::NodeHandle nh;

  std::string pcdFile = "";
  nh.getParam("pcdFile", pcdFile);

  if(pcdFile == "") {
    ROS_ERROR_STREAM("Unable to find file \"" << pcdFile << "\"");
    nh.shutdown();
  }

  ros::Publisher pub = nh.advertise<PointCloud> ("cloud_publisher/cloud_from_file", 1); //<PointCloud>

  //PointCloud::Ptr msg(new PointCloud);
  sensor_msgs::PointCloud2 msg;
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcdFile, *msg) == -1)
  if(pcl::io::loadPCDFile (pcdFile, msg) == -1)
  {
    ROS_ERROR_STREAM("Unable to read from file \"" << pcdFile << "\"");
    nh.shutdown();
    return -1;
  }

  ros::Rate r(10);

  while (nh.ok())
  {
    //msg->header.frame_id = "/base_link";
    msg.header.frame_id = "/base_link";
    //pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    //msg->header.stamp = ros::Time::now().toNSec();
    msg.header.stamp = ros::Time::now();
    //pub.publish(msg->makeShared());
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
*/
