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
#include <people_msgs/PositionMeasurementArray.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

using namespace std;
float radius=0.5;
int minClusterSize = 10;
ros::Subscriber radar_sub_;
ros::Subscriber laser_sub_;
ros::Publisher point_positive_pub_;
ros::Publisher point_negative_pub_;
ros::Subscriber tracker_sub_;
typedef pcl::PointXYZI PointTypeFull;
typedef pcl::PointCloud<PointTypeFull> PointCloud;
float personX,personY;
PointCloud::Ptr pcl_msg (new PointCloud);

void callback(radar::radiusConfig &config, uint32_t level) {
	
	ROS_INFO("Reconfigure Request of radius: %f", 
			config.radius);
	radius=config.radius;
	minClusterSize=config.minClusterSize;
}

void backupRadarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
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
//			printf("PUBLISHED %f %f %f\n",xi,yi,zi);
//			pcl_msg->points.push_back (pcl::PointXYZ(xi,yi,zi));
			pcl_msg->width++;
		}
	}
//	point_pub_.publish (pcl_msg);

	//Print results
	cout << "Number of neigbours of points are "; 
	for (int i=0;i<pc_indices->size();i++){
		cout <<  neighbourArray[i] << " ";
	}
	cout << endl;

}

void trueRadarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
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

	/* Find neighbours of people detections */
	int neighbourSize=0;
	float xi = personX;
	float yi = personY;
	float zi = 0;
	for(int j=0;j<pc_indices->size();j++){
		//Compute the r2 for sphere two points
		float xj = pcl_pc->points[(*pc_indices)[j]].x;
		float yj = pcl_pc->points[(*pc_indices)[j]].y;
		float zj = pcl_pc->points[(*pc_indices)[j]].z;
		float r2 = (xi-xj)*(xi-xj)+(yi-yj)*(yi-yj)+(zi-zj)*(zi-zj)*0;

		//Point lies within defined radius
		if(r2<(radius*radius)){
//			pcl_msg->points.push_back (pcl::PointXYZ(xj,yj,zj));
			pcl_msg->width++;
		}
	}
//	point_pub_.publish (pcl_msg);

	//Print results
	cout << "Number of neigbours of points are "; 
	/*for (int i=0;i<pc_indices->size();i++){
		cout <<  neighbourArray[i] << " ";
	}*/
	cout << endl;

}


bool distanceSimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  if (sqrt((point_a.x - point_b.x)*(point_a.x - point_b.x)+(point_a.y - point_b.y)*(point_a.y - point_b.y)) < radius)
    return (true);
  else
    return (false);
}

void allRadarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	//Convert msg to pcl
	cout << "New pcl received" << endl;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*msg, *pcl_pc);
	pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);
	pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
	pcl::PointCloud<PointTypeFull>::Ptr	cloud_out (new pcl::PointCloud<PointTypeFull>);
	//pcl::copyPointCloud (*pcl_pc,*cloud_out);

	//Set up pcl
	pcl::IndicesPtr pc_indices(new std::vector<int>);
	pcl::PassThrough<pcl::PointXYZI> pt;
	pt.setInputCloud(pcl_pc);
	pt.filter(*pc_indices);

	//std::cerr << "Segmenting to clusters...\n", tt.tic ();
	pcl::ConditionalEuclideanClustering<PointTypeFull> cec (true);
	cec.setInputCloud (pcl_pc);

	cec.setClusterTolerance (500);//TODO what is it?
	cec.setConditionFunction (&distanceSimilarity);

	//cec.setMaxClusterSize (cloud_with_normals->points.size () / 5);
	cec.segment (*clusters);
	//std::cerr << ">> Done: " << tt.toc () << " ms\n";
	printf("Clusters: %i: ",clusters->size());

	for (int i = 0; i < clusters->size (); ++i)
	{
		printf("%i ",((*clusters)[i]).indices.size());
		float meanX=0;
		float meanY=0;
		for (int j = 0; j <((*clusters)[i]).indices.size(); ++j){
			meanX += pcl_pc->points[((*clusters)[i]).indices[j]].x;
			meanY += pcl_pc->points[((*clusters)[i]).indices[j]].y;
		}
		meanX = meanX/((*clusters)[i]).indices.size();
		meanY = meanY/((*clusters)[i]).indices.size();
		printf("%f, ",sqrt((meanX-personX)*(meanX-personX)+(meanY-personY)*(meanY-personY)));

		float intensity = 0.1;
		pcl_msg->header.frame_id = "laser";
		pcl_msg->height = 1;
		pcl_msg->width = 0;
		pcl_msg->points.clear();
		for (int j = 0; j <((*clusters)[i]).indices.size(); ++j){
			//if (sqrt((meanX-personX)*(meanX-personX)+(meanY-personY)*(meanY-personY)) < 0.5) intensity = 1.0;
			//cloud_out->points[(*clusters)[i].indices[j]].intensity = intensity;
			pcl_msg->points.push_back (pcl_pc->points[(*clusters)[i].indices[j]]);
			pcl_msg->width++;
		}
		if (sqrt((meanX-personX)*(meanX-personX)+(meanY-personY)*(meanY-personY)) > 0.5) point_negative_pub_.publish (pcl_msg); else point_positive_pub_.publish (pcl_msg);
	}

	printf("\n");
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

}

void trackerCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
{
	printf("People: %i: ",(int)msg->people.size());
	for (int i = 0;i<msg->people.size();i++)
	{
		printf("%f %f ",msg->people[i].pos.x,msg->people[i].pos.y);
		personX = msg->people[0].pos.x;
		personY = msg->people[0].pos.y;
	}
	printf("\n");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "radarology");
  ros::NodeHandle nh_;

  // Dynamic reconfiguration server
  dynamic_reconfigure::Server<radar::radiusConfig> server;
  dynamic_reconfigure::Server<radar::radiusConfig>::CallbackType f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  radar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/radar/RScan", 1, allRadarCallback);
  laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, laserCallback);
  tracker_sub_ = nh_.subscribe<people_msgs::PositionMeasurementArray>("/people_tracker_measurements", 1, trackerCallback);
  point_positive_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/positive", 100000);
  point_negative_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/negative", 100000);

  ros::spin();
  return 0;
}

