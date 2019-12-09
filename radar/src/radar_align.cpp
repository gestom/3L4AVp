#include <ros/ros.h>
#include <stdio.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.h>
using namespace std;
ros::Publisher radar_pub_;
ros::Subscriber radar_sub_;



void callback(const sensor_msgs::PointCloud2ConstPtr& msg){

  const  tf::TransformListener tf_listener;
  sensor_msgs::PointCloud2 t_pc;
  sensor_msgs::PointCloud2 pc;
  pc = *msg;
  pc.header.stamp = ros::Time(0);
  pcl_ros::transformPointCloud("map", pc, t_pc, tf_listener);
  radar_pub_.publish(t_pc);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "radar_align");
	ros::NodeHandle nh_;
  const  tf::TransformListener tf_listener;
	radar_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/radar/RScan/aligned", 1);
  radar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/radar/RScan", 10,callback);
  ros::spin();
	return 0;
}
