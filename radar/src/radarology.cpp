#include <ros/ros.h>
#include <stdio.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <radar/radiusConfig.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

using namespace std;
float radius=0.5;
ros::Subscriber radar_sub_;
ros::Subscriber laser_sub_;

void callback(radar::radiusConfig &config, uint32_t level) {
	
	ROS_INFO("Reconfigure Request of radius: %f", 
			config.radius);
	radius=config.radius;
}

void radarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*msg, *pcl_pc_in);

	cout << "Radar pointcloud received" << endl;
	
}
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	
	cout << "Legs detecting in process" << endl;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "radarology");
  ros::NodeHandle nh_;

  // Dynamic reconfiguration server
  dynamic_reconfigure::Server<radar::radiusConfig> server;
  dynamic_reconfigure::Server<radar::radiusConfig>::CallbackType f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  radar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/radar/RScan", 1, radarCallback);
  laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, laserCallback);

  ros::spin();
  return 0;
}

