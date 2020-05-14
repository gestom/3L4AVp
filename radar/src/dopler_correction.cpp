#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread.hpp>
#include <depth_image_proc/depth_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <limits>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <radar/ppl.h>
#include <radar/radar_fusion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
float fy,fx,cx,cy;
using namespace cv;
using namespace std;
using namespace message_filters;
ros::Publisher radar_pub;


void callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::PointCloud2::ConstPtr& radar)
{

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZHSV>);
  pcl::fromROSMsg(*radar, *pcl_pc);

  for (int i = 0 ; i < pcl_pc->points.size(); i++){
    pcl_pc->points[i].v -= odom->twist.twist.linear.x;
  }

  sensor_msgs::PointCloud2 out ;
  pcl::toROSMsg(*pcl_pc, out);

  radar_pub.publish(out);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dopler_correction");
	ros::NodeHandle n;

  std::string odom_topic;
  std::string radar_in_topic;
  std::string  radar_out_topic;

  n.param<std::string>("odom_in",  odom_topic, "/husky_velocity_controller/odom");
  n.param<std::string>("radar_in",radar_in_topic, "/radar/RScan/aligned");
  n.param<std::string>("radar_out",radar_out_topic, "/radar_fixed");

  radar_pub = n.advertise<sensor_msgs::PointCloud2>(radar_out_topic,1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, odom_topic, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> radar_sub(n, radar_in_topic, 1);
  TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2> sync(odom_sub, radar_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

	ros::spin();
	return 0;
}
