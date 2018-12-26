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
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
float radius=0.5;
int minClusterSize = 10;
ros::Subscriber radar_sub_;
ros::Subscriber laser_sub_;
ros::Publisher point_pub_;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud::Ptr pcl_msg (new PointCloud);

void callback(radar::radiusConfig &config, uint32_t level) {
	
	ROS_INFO("Reconfigure Request of radius: %f", 
			config.radius);
	radius=config.radius;
	minClusterSize=config.minClusterSize;
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

	pcl_msg->header.frame_id = "laser";
	pcl_msg->height = 1;
	pcl_msg->width = 0;
	pcl_msg->points.clear();

	/* Find neighbours for poincloud points */
	for (int i=0;i< pc_indices->size();i++){
		int neighbourSize=0;
		float xi = pcl_pc->points[(*pc_indices)[i]].x;
		float yi = pcl_pc->points[(*pc_indices)[i]].y;
		float zi = pcl_pc->points[(*pc_indices)[i]].z;
		for(int j=0;j<pc_indices->size();j++){
			//Compute the r2 for sphere two points
			float xj = pcl_pc->points[(*pc_indices)[j]].x;
			float yj = pcl_pc->points[(*pc_indices)[j]].y;
			float zj = pcl_pc->points[(*pc_indices)[j]].z;
			float r2 = (xi-xj)*(xi-xj)+(yi-yj)*(yi-yj)+(zi-zj)*(zi-zj)*0;

			//Point lies within defined radius
			if(r2<(radius*radius)){
				neighbourSize++;	
			}	
		}
		neighbourArray.push_back(neighbourSize);
		if (neighbourSize > minClusterSize){ 
			printf("PUBLISHED %f %f %f\n",xi,yi,zi);
			pcl_msg->points.push_back (pcl::PointXYZ(xi,yi,zi));
			pcl_msg->width++;
		}
	}
	point_pub_.publish (pcl_msg);

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
  point_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/points2", 100000);

  ros::spin();
  return 0;
}

