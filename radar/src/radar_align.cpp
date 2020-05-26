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
  tf::StampedTransform transform;
  const  tf::TransformListener tf_listener;
  sensor_msgs::PointCloud2 t_pc;
  sensor_msgs::PointCloud2 pc;
  pc = *msg;
  pc.header.stamp = ros::Time(0);
  pc.header.frame_id = "base_radar_link";


  try{
    tf_listener.waitForTransform("base_radar_link", "map", ros::Time(0), ros::Duration(5.0), ros::Duration(2)); // base on line 193 of /tf/tf.h
    tf_listener.lookupTransform("base_radar_link", "map", ros::Time(0), transform); // based on line 129 /tf/tf.h
  }
  catch(tf::LookupException){
    std::cerr << "Transform lookup failed" << std::endl;
  }
  catch(tf2::ConnectivityException){
    std::cerr << "Something broke. Did you jump back in time?" << std::endl;
  }
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
