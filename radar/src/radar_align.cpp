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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "radar_align");
	ros::NodeHandle nh_;
  const  tf::TransformListener tf_listener;
  //  radar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/radar/RScan", 1, callback);
	radar_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/radar/RScan/aligned", 1);
  boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedPtr;
   sensor_msgs::PointCloud2 pc;
  sensor_msgs::PointCloud2 t_pc; 
  while (ros::ok())
    {
      sharedPtr  = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/radar/RScan",ros::Duration(2.0));
      if (sharedPtr == NULL){
        ROS_INFO("No point clound messages received");
        continue;
      }else{
        pc = *sharedPtr;
      }
      pc.header.frame_id = "base_radar_link";
      pcl_ros::transformPointCloud("map", pc, t_pc, tf_listener);
      radar_pub_.publish(t_pc);
      ros::spinOnce();
    }
	return 0;
}
