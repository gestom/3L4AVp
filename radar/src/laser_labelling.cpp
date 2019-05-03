// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

using namespace sensor_msgs;
using namespace message_filters;

laser_geometry::LaserProjection projector_;
ros::Publisher laser_cloud_pub_;
ros::Publisher marker_array_pub_;
ros::Publisher filtered_cloud_pub_;

int x_limit_min_ = 1.0;
int x_limit_max_ = 9.0;
int y_limit_min_ = -1.9;
int y_limit_max_ = 1.9;

void scanCallback(const LaserScan::ConstPtr& scan_in, const people_msgs::PositionMeasurementArray::ConstPtr& people)
//void scanCallback(const LaserScan::ConstPtr& scan_in)
{
  // if(people->people.size() == 0)
  //   return;

  //std::cerr << people->people[0].pos.x << " --- " << people->people[0].pos.y << std::endl;
  
  PointCloud2 ros_pc2;
  projector_.projectLaser(*scan_in, ros_pc2);
  laser_cloud_pub_.publish(ros_pc2);

  
  
  return;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(ros_pc2, *pcl_pc);


  
  pcl::CropBox<pcl::PointXYZ> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(people->people[0].pos.x-0.3,
				   people->people[0].pos.y-0.3,
				   -9.0, 1.0));
  boxFilter.setMax(Eigen::Vector4f(people->people[0].pos.x+0.3,
				   people->people[0].pos.y+0.3,
				   9.0, 1.0));
  boxFilter.setInputCloud(pcl_pc);
  boxFilter.filter(*pcl_pc);
  
  if(filtered_cloud_pub_.getNumSubscribers() > 0) {
    pcl::toROSMsg(*pcl_pc, ros_pc2);
    filtered_cloud_pub_.publish(ros_pc2);
  }
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pcl_pc);
  
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(2.0);
  ec.setMinClusterSize(3);
  ec.setMaxClusterSize(1000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pcl_pc);
  ec.extract(cluster_indices);

  visualization_msgs::MarkerArray marker_array;
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
      cluster->points.push_back(pcl_pc->points[*pit]);
    }
    cluster->width = cluster->size();
    cluster->height = 1;
    cluster->is_dense = true;

    Eigen::Vector4f min, max;
    pcl::getMinMax3D(*cluster, min, max);

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "laser";
    marker.ns = "leg";
    marker.id = marker.id+1;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    geometry_msgs::Point p[24];
    p[0].x = max[0]; p[0].y = max[1]; p[0].z = max[2];
    p[1].x = min[0]; p[1].y = max[1]; p[1].z = max[2];
    p[2].x = max[0]; p[2].y = max[1]; p[2].z = max[2];
    p[3].x = max[0]; p[3].y = min[1]; p[3].z = max[2];
    p[4].x = max[0]; p[4].y = max[1]; p[4].z = max[2];
    p[5].x = max[0]; p[5].y = max[1]; p[5].z = min[2];
    p[6].x = min[0]; p[6].y = min[1]; p[6].z = min[2];
    p[7].x = max[0]; p[7].y = min[1]; p[7].z = min[2];
    p[8].x = min[0]; p[8].y = min[1]; p[8].z = min[2];
    p[9].x = min[0]; p[9].y = max[1]; p[9].z = min[2];
    p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
    p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
    p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
    p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
    p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
    p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
    p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
    p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
    p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
    p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
    p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
    p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
    p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
    p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
    for(int i = 0; i < 24; i++)
      marker.points.push_back(p[i]);
    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.5;
    marker.lifetime = ros::Duration(0.1);
    marker_array.markers.push_back(marker);
  }
  
  if(marker_array.markers.size()) {
    marker_array_pub_.publish(marker_array);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_labelling");

  ros::NodeHandle n;
  
  message_filters::Subscriber<LaserScan> laser_sub(n, "scan", 100);
  message_filters::Subscriber<people_msgs::PositionMeasurementArray> people_sub(n, "people_tracker_measurements", 100);
  typedef sync_policies::ApproximateTime<LaserScan, people_msgs::PositionMeasurementArray> MySyncPolicy;
  //typedef sync_policies::ExactTime<LaserScan, people_msgs::PositionMeasurementArray> MySyncPolicy;
  
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), laser_sub, people_sub);
  sync.registerCallback(boost::bind(&scanCallback, _1, _2));

  //ros::Subscriber sub = n.subscribe("scan", 100, scanCallback);

  laser_cloud_pub_ = n.advertise<PointCloud2>("scan_pc2", 100);
  marker_array_pub_ = n.advertise<visualization_msgs::MarkerArray>("laser_bbox", 100);
  filtered_cloud_pub_ = n.advertise<PointCloud2>("filtered_cloud", 100);
  ros::spin();
  
  return 0;
}
