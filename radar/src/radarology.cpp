#include <ros/ros.h>
#include <stdio.h>
#include <visualization_msgs/MarkerArray.h>
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
float clusterRadius=0.5;
float personDistance=0.5;
int minClusterSize = 10;
ros::Subscriber radar_sub_;
ros::Subscriber laser_sub_;
ros::Publisher point_positive_pub_;
ros::Publisher point_negative_pub_;
ros::Publisher  marker_array_pub_;
ros::Subscriber tracker_sub_;
typedef pcl::PointXYZI PointTypeFull;
typedef pcl::PointCloud<PointTypeFull> PointCloud;
float personX,personY;
PointCloud::Ptr pcl_msg (new PointCloud);

void callback(radar::radiusConfig &config, uint32_t level) {
	
	clusterRadius=config.clusterRadius;
	personDistance=config.personDistance;
	minClusterSize=config.minClusterSize;
}

bool distanceSimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  if (sqrt((point_a.x - point_b.x)*(point_a.x - point_b.x)+(point_a.y - point_b.y)*(point_a.y - point_b.y)) < clusterRadius)
    return (true);
  else
    return (false);
}

void addBoundingBoxMarker(visualization_msgs::MarkerArray::Ptr markerArray,float minX,float maxX,float minY,float maxY,float minZ, float maxZ)
{
    /*** bounding box ***/
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "laser";
    marker.ns = "object3d";
    marker.id = markerArray->markers.size();
    marker.type = visualization_msgs::Marker::LINE_LIST;
    geometry_msgs::Point p[24];
    p[0].x = maxX; p[0].y = maxY; p[0].z = maxZ;
    p[1].x = minX; p[1].y = maxY; p[1].z = maxZ;
    p[2].x = maxX; p[2].y = maxY; p[2].z = maxZ;
    p[3].x = maxX; p[3].y = minY; p[3].z = maxZ;
    p[4].x = maxX; p[4].y = maxY; p[4].z = maxZ;
    p[5].x = maxX; p[5].y = maxY; p[5].z = minZ;
    p[6].x = minX; p[6].y = minY; p[6].z = minZ;
    p[7].x = maxX; p[7].y = minY; p[7].z = minZ;
    p[8].x = minX; p[8].y = minY; p[8].z = minZ;
    p[9].x = minX; p[9].y = maxY; p[9].z = minZ;
    p[10].x = minX; p[10].y = minY; p[10].z = minZ;
    p[11].x = minX; p[11].y = minY; p[11].z = maxZ;
    p[12].x = minX; p[12].y = maxY; p[12].z = maxZ;
    p[13].x = minX; p[13].y = maxY; p[13].z = minZ;
    p[14].x = minX; p[14].y = maxY; p[14].z = maxZ;
    p[15].x = minX; p[15].y = minY; p[15].z = maxZ;
    p[16].x = maxX; p[16].y = minY; p[16].z = maxZ;
    p[17].x = maxX; p[17].y = minY; p[17].z = minZ;
    p[18].x = maxX; p[18].y = minY; p[18].z = maxZ;
    p[19].x = minX; p[19].y = minY; p[19].z = maxZ;
    p[20].x = maxX; p[20].y = maxY; p[20].z = minZ;
    p[21].x = minX; p[21].y = maxY; p[21].z = minZ;
    p[22].x = maxX; p[22].y = maxY; p[22].z = minZ;
    p[23].x = maxX; p[23].y = minY; p[23].z = minZ;
    for(int i = 0; i < 24; i++) marker.points.push_back(p[i]);
    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.5;
    marker.lifetime = ros::Duration(0.1);
    markerArray->markers.push_back(marker);
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
	visualization_msgs::MarkerArray::Ptr markerArray(new visualization_msgs::MarkerArray);

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
	
	int positives = 0;
	for (int i = 0; i < clusters->size (); ++i)
	{
		printf("%i ",((*clusters)[i]).indices.size());
		float meanX=0;
		float meanY=0;
		float tX,tY,tZ;
		float minX,minY,minZ,maxX,maxY,maxZ;
		minX=minY=minZ=+100000;
		maxX=maxY=maxZ=-100000;
		for (int j = 0; j <((*clusters)[i]).indices.size(); ++j){
			tX = pcl_pc->points[((*clusters)[i]).indices[j]].x;
			tY = pcl_pc->points[((*clusters)[i]).indices[j]].y;
			tZ = pcl_pc->points[((*clusters)[i]).indices[j]].z;
			meanX += tX;
			meanY += tY;
			minX = fmin(tX,minX);	
			minY = fmin(tY,minY);	
			minZ = fmin(tZ,minZ);	
			maxX = fmax(tX,maxX);	
			maxY = fmax(tY,maxY);	
			maxZ = fmax(tZ,maxZ);	
		}
		meanX = meanX/((*clusters)[i]).indices.size();
		meanY = meanY/((*clusters)[i]).indices.size();
		printf("%f, ",sqrt((meanX-personX)*(meanX-personX)+(meanY-personY)*(meanY-personY)));

		float intensity = 0.1;
		pcl_msg->header.frame_id = "laser";
		pcl_msg->height = 1;
		pcl_msg->width = 0;
		pcl_msg->points.clear();
		for (int j = 0; j <(int)((*clusters)[i]).indices.size(); ++j){
			pcl_msg->points.push_back (pcl_pc->points[(*clusters)[i].indices[j]]);
			pcl_msg->width++;
		}
		if (sqrt((meanX-personX)*(meanX-personX)+(meanY-personY)*(meanY-personY)) > personDistance)
		{
			point_negative_pub_.publish (pcl_msg);
		} else {
			printf("Offset: %f %f %f %f\n",meanX,meanY,personX,personY);
			point_positive_pub_.publish (pcl_msg);
			positives++;
			addBoundingBoxMarker(markerArray,minX,maxX,minY,maxY,minZ,maxZ);
		}
	}


	printf("\n Positives: %i\n",positives);
	marker_array_pub_.publish(markerArray);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
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

  marker_array_pub_   = nh_.advertise<visualization_msgs::MarkerArray>("/person", 1);
  radar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/radar/RScan", 1, allRadarCallback);
  laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, laserCallback);
  tracker_sub_ = nh_.subscribe<people_msgs::PositionMeasurementArray>("/people_tracker_measurements", 1, trackerCallback);
  point_positive_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/positive", 100000);
  point_negative_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/negative", 100000);

  ros::spin();
  return 0;
}

