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
#include <vector>
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
	//Convert msg to pcl
	cout << "New pcl received" << endl;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*msg, *pcl_pc);

	//Set up pcl
	pcl::IndicesPtr pc_indices(new std::vector<int>);
	pcl::PassThrough<pcl::PointXYZI> pt;
	pt.setInputCloud(pcl_pc);
	pt.filter(*pc_indices);
	vector<int> neighbourArray;		
	neighbourArray.clear();	

	/* Find neighbours for poincloud points */
	for (int i=0;i< pc_indices->size();i++){
		int neighbourSize=0;
		for(int j=0;j<pc_indices->size();j++){
			//Compute the r2 for sphere two points
			float r2 = (pcl_pc->points[(*pc_indices)[i]].x - pcl_pc->points[(*pc_indices)[j]].x) * (pcl_pc->points[(*pc_indices)[i]].x - pcl_pc->points[(*pc_indices)[j]].x) + 
				(	pcl_pc->points[(*pc_indices)[i]].y - pcl_pc->points[(*pc_indices)[j]].y) * (pcl_pc->points[(*pc_indices)[i]].y - pcl_pc->points[(*pc_indices)[j]].y) + 
				(pcl_pc->points[(*pc_indices)[i]].z - pcl_pc->points[(*pc_indices)[j]].z) * (pcl_pc->points[(*pc_indices)[i]].z - pcl_pc->points[(*pc_indices)[j]].z); 


			//Point lies within defined radius
			if(r2<(radius*radius)){
				neighbourSize++;	
			}	
		}
		neighbourArray.push_back(neighbourSize);
	}

	//Print results
	cout << "Number of neigbours of points are "; 
	for (int i=0;i<pc_indices->size();i++){
		cout <<  neighbourArray[i] << " ";
	}
	cout << endl;

}
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

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

