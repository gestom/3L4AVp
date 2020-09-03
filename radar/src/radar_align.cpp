#include <ros/ros.h>
#include <stdio.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
using namespace std;
ros::Publisher radar_pub_;
ros::Subscriber radar_sub_;
int lookedUp = 0;

tf::TransformListener* tf_listener = NULL;

void callback(const sensor_msgs::PointCloud2ConstPtr& msg){

  tf::StampedTransform transformm;
  if(lookedUp == 0){
    try{
      tf_listener->waitForTransform("base_radar_link", "map", ros::Time(0), ros::Duration(1.0), ros::Duration(1));
      tf_listener->lookupTransform("base_radar_link", "map", ros::Time(0), transformm);
      lookedUp = 1;
    }
    catch(tf::LookupException){
      std::cerr << "Transform lookup failed" << std::endl;
    }
    catch(tf2::ConnectivityException){
      std::cerr << "Something broke. Did you jump back in time?" << std::endl;
    }
   }

  sensor_msgs::PointCloud2 t_pc;
  sensor_msgs::PointCloud2 pc;
  pc = *msg;
  pc.header.stamp = ros::Time(0);
  pc.header.frame_id = "base_radar_link";


pcl_ros::transformPointCloud("map", pc, t_pc, *tf_listener);
  radar_pub_.publish(t_pc);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "radar_align");
	ros::NodeHandle nh_;
	radar_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/radar/RScan/aligned", 1);
  radar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/radar/RScan", 10,callback);
  ros::MultiThreadedSpinner spinner(4);
  tf_listener = new (tf::TransformListener);
  spinner.spin();
	return 0;
}
