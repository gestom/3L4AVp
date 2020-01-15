#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread.hpp>
#include <depth_image_proc/depth_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <limits>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <radar/radar_fusion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
using namespace cv;
using namespace std;
ros::Subscriber groundTruthSub;
ros::Subscriber detectorSub;
boost::shared_ptr<image_transport::ImageTransport> it_;
image_transport::Subscriber sub_depth_;
image_transport::Publisher depth_pub_;
ros::Publisher info_pub_;
ros::Subscriber info_sub_,radar_pose_sub_,leg_pose_sub_,variance_sub_,variance_deep_sub_, deep_radar_sub_, gt_subscriber_;


std::vector<std::vector<float> > rad;
std::vector<std::vector<float> > cam;
std::vector<std::vector<float> > leg;
std::vector<std::vector<float> > deep;
std::vector<float> ccovR;
std::vector<float> ccovD;
std::vector<float> ccovL;
float fy,fx,cx,cy;
float personRadx;
float personRady;
float personCamx;
float personCamy;
float personLegx;
float personLegy;
float personDeepx;
float personDeepy;
float personRadxx;
float personRadyy;
float personCamxx;
float personCamyy;
float personLegxx;
float personLegyy;
float personDeepxx;
float personDeepyy;
float totalDistR;
float totalDistL;
ros::Time timestampS;
int numOfCycles=0;
float covL=0.1;
float covR=0.1;
float covLL=0.1;
float covRR=0.1;
float covD = 0.1;
float covDD = 0.1;
float realX;
float realY;
ros::Publisher evaluator_mux_publisher_;
typedef struct
{
  float maxX,maxY,maxZ;
  float minX,minY,minZ;
  int timestampSecs;
  int timestampNsecs;
}SBox; 

vector<SBox> groundTruth;
vector<SBox> detection;

float boxVolume(SBox box)
{
	if (box.maxX < box.minX || box.maxY < box.minY || box.maxZ < box.minZ) return 0;  
	return (box.maxX - box.minX)*(box.maxY - box.minY)*(box.maxZ - box.minZ);
}

SBox boxOverlap(SBox a,SBox b)
{
	SBox overlap;
	overlap.maxX = fmin(a.maxX,b.maxX);   
	overlap.maxY = fmin(a.maxY,b.maxY);   
	overlap.maxZ = fmin(a.maxZ,b.maxZ);   
	overlap.minX = fmax(a.minX,b.minX);   
	overlap.minY = fmax(a.minY,b.minY);   
	overlap.minZ = fmax(a.minZ,b.minZ);
	return overlap;
}

SBox formBox(visualization_msgs::Marker marker)
{
	SBox result;
	result.maxX=result.maxY=result.maxZ=-1000000;
	result.minX=result.minY=result.minZ=+1000000;
	for (int i = 0;i<(int)marker.points.size();i++)
	{
		result.minX = fmin(marker.points[i].x,result.minX);
		result.minY = fmin(marker.points[i].y,result.minY);
		result.minZ = fmin(marker.points[i].z,result.minZ);
		result.maxX = fmax(marker.points[i].x,result.maxX);
		result.maxY = fmax(marker.points[i].y,result.maxY);
		result.maxZ = fmax(marker.points[i].z,result.maxZ);
	}
	//printf("%.3f %.3f %.3f %.3f %.3f %.3f\n",result.minX,result.maxX,result.minY,result.maxY,result.minZ,result.maxZ);
	result.timestampSecs = marker.header.stamp.sec;
	result.timestampNsecs = marker.header.stamp.nsec;
	return result;
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
    for(unsigned int i = 0; i < 24; i++) marker.points.push_back(p[i]);
    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.5;
    marker.lifetime = ros::Duration(0.1);
    markerArray->markers.push_back(marker);
}

void checkQueues()
{
	vector<int> removeGT;
	vector<int> removeDet;
	//printf("Queues: %d %d\n",groundTruth.size(),detection.size());
	int i = 0;
  int j = 0;

	while (i<groundTruth.size())
	{
		j=0;
		while (j<detection.size())
		{
			if (detection[j].timestampSecs == groundTruth[i].timestampSecs && detection[j].timestampNsecs == groundTruth[i].timestampNsecs)
			{
				groundTruth.erase(groundTruth.begin()+i);
				detection.erase(detection.begin()+j);
				i--;
				j--;
			}
			j++;
		}
		i++;
	}
}

void groundTruthCb(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	for (int i = 0;i<(int)msg->markers.size();i++)groundTruth.push_back(formBox(msg->markers[i]));
	checkQueues();
}

void detectorCb(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	for (int i = 0;i<(int)msg->markers.size();i++)detection.push_back(formBox(msg->markers[i]));
	checkQueues();
}
std::vector<geometry_msgs::PoseWithCovariance>  constructPoseWCovariance (std::vector<std::vector<float> > vec, std::vector<float> cov){

  std::vector<geometry_msgs::PoseWithCovariance> field;
  for(unsigned int i = 0;i<vec.size();i++){

    geometry_msgs::PoseWithCovariance pose;
    pose.pose.position.x = vec[i][0];
    pose.pose.position.y= vec[i][1];
    pose.pose.position.z= vec[i][2];
    pose.covariance = { cov[i], 0, 0, 0, 0, 0,
                        0, cov[i], 0, 0, 0, 0,
                        0, 0, cov[i], 0, 0, 0,
                        0, 0, 0, cov[i], 0, 0,
                        0, 0, 0, 0, cov[i], 0,
                        0, 0, 0, 0, 0, cov[i]};
    field.push_back(pose);
  }
  return field;
}
void groundTruthCallback(const geometry_msgs::PoseArrayConstPtr& msg)
{
	numOfCycles++; 
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  geometry_msgs::TransformStamped transformStamp;
  geometry_msgs::PoseStamped ps;
  ps.header= msg->header;
  try{
  transformStamp =
    tfBuffer.lookupTransform("laser", "map", ros::Time(0), ros::Duration(1.0) );
  }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
  }
    cam.clear();
    std::vector<float> ccovC;
    for(unsigned int i = 0;i<msg->poses.size();i++)
     {
       ps.pose=msg->poses[i];
       tf2::doTransform(ps, ps, transformStamp);
       
        vector<float> vec(3,0);
        vec[0]=ps.pose.position.x;
        vec[1]=-ps.pose.position.y;
        vec[2]=ps.pose.position.z;
        cam.push_back(vec);
        ccovC.push_back(0);
      }

		realX=((1/leg[0][3])*leg[0][0] + (1/ccovR[0])*rad[0][0])/((1/leg[0][3])+(1/ccovR[0]));
		realY=((1/leg[0][3])*leg[0][1] + (1/ccovR[0])*rad[0][1])/((1/leg[0][3])+(1/ccovR[0]));

		//cout<< "Cam/Rad/Leg/Deep " << cam[0][0] << " " << cam[0][1] << " " << rad[0][0] << " " << rad[0][1] << " " << ccovR[0] << " " <<  leg[0][0] << " " << leg[0][1] << " " << leg[0][3] <<  " " << deep[0][0] << " " << deep[0][1] << " " << ccovD[0] << endl;

		float distRC=0;
		float distCL=0;
		distRC=sqrt(pow((cam[0][0]-rad[0][0]),2)+pow((cam[0][1]-rad[0][1]),2));
		distCL=sqrt(pow((cam[0][0]-leg[0][0]),2)+pow((cam[0][0]-leg[0][1]),2));

		if(!isnan(distRC)|| !isnan(distCL)){
			totalDistR+=distRC;
			totalDistL+=distCL;

		}
    radar::radar_fusion message;
    std_msgs::Header header;
    geometry_msgs::PoseWithCovariance pose;
    header = msg->header;
    header.seq = numOfCycles;
    header.frame_id = "map";
    //cout<<"rad"<<endl;
    message.header = header;
    message.rad=constructPoseWCovariance(rad,ccovR);
    //cout<<"LEG"<<endl;
    message.leg=constructPoseWCovariance(leg,ccovL);
    //cout<<"DEEP"<<endl;
    message.deep=constructPoseWCovariance(deep,ccovD);
    //cout<<"CAM"<<endl;
    message.gt=constructPoseWCovariance(cam,ccovC);
    evaluator_mux_publisher_.publish(message);
    cout << "gt   "<<message.gt.size()<<endl;
    cout << "rad   "<<message.rad.size()<<endl;
    cout << "leg   "<<message.leg.size()<<endl;
    cout << "deep   "<<message.deep.size()<<endl;

    
}

void infoCallback(const sensor_msgs::CameraInfoPtr& msg){


        cx = msg->P[2];
        cy = msg->P[6];
        fx = msg->P[0];
        fy = msg->P[5];
	info_pub_.publish(msg);
}

void radarPoseCallback(const geometry_msgs::PoseArrayConstPtr& msg)
{
  if (msg->poses.size() == 0) return;
  rad.clear();
 	for(unsigned int i = 0;i<msg->poses.size();i++)
    {
      vector<float> ps(3,0);
      ps[0]=msg->poses[i].position.x;
      ps[1]=msg->poses[i].position.y;
      ps[2]=msg->poses[i].position.z;
      rad.push_back(ps);
    }

}

void legPoseCallback(const people_msgs::PositionMeasurementArrayConstPtr& msg){
  if (msg->people.size() == 0) return;
  leg.clear();
  ccovL.clear();
 	for(unsigned int i = 0;i<msg->people.size();i++)
    {
      vector<float> ps(3,0);
      ps[0]=msg->people[i].pos.x;
      ps[1]=msg->people[i].pos.y;
      ps[2]=msg->people[i].pos.z;
      ccovL.push_back(msg->people[i].covariance[0]);
         leg.push_back(ps);
    }

}

void deepPoseCallback(const geometry_msgs::PoseArrayConstPtr& msg){
  if (msg->poses.size() == 0) return;
  deep.clear();
 	for(unsigned int i = 0;i<msg->poses.size();i++)
    {
      vector<float> ps(3,0);
      ps[0]=msg->poses[i].position.x;
      ps[1]=msg->poses[i].position.y;
      ps[2]=msg->poses[i].position.z;
      deep.push_back(ps);
    }

}

void varianceCallback(const geometry_msgs::PoseArrayConstPtr& msg){
  ccovR.clear();
  for(unsigned int i = 0;i<msg->poses.size();i++)
    {
      ccovR.push_back(msg->poses[i].position.x);
    }
}

void varianceDeepCallback(const geometry_msgs::PoseArrayConstPtr& msg){
  ccovD.clear();
  for(unsigned int i = 0;i<msg->poses.size();i++)
    {
      ccovD.push_back(msg->poses[i].position.x);
    }
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "evaluator");
	ros::NodeHandle nh_;
	groundTruthSub = nh_.subscribe<visualization_msgs::MarkerArray>("/ground", 1, groundTruthCb);
	detectorSub = nh_.subscribe<visualization_msgs::MarkerArray>("/detector", 1, detectorCb);

  std::vector<float> ps(1,0);
  leg.push_back(ps);
  cam.push_back(ps);
  deep.push_back(ps);
  rad.push_back(ps);
  float a = 0;
  ccovR.push_back(a);
  ccovD.push_back(a);
  ccovL.push_back(a);
  image_transport::ImageTransport it_(nh_);
	depth_pub_  = it_.advertise("/person/depth/image_rect_raw", 1);
	info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/person/depth/camera_info",1);
	radar_pose_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("/pt/svm1",1,radarPoseCallback);
	variance_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("people_tracker/svm1/trajectory_acc",1,varianceCallback);
	variance_deep_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("people_tracker/cnn1/trajectory_acc",1,varianceDeepCallback);
	leg_pose_sub_ = nh_.subscribe<people_msgs::PositionMeasurementArray>("/people_tracker_measurements",1,legPoseCallback); 
	deep_radar_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("/pt/cnn1",1, deepPoseCallback);
  gt_subscriber_ = nh_.subscribe<geometry_msgs::PoseArray>("/person/ground_truth",1,groundTruthCallback);
  evaluator_mux_publisher_ = nh_.advertise<radar::radar_fusion>("/evaulator_mux",1);


	ros::spin();
	return 0;
}

