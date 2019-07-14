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
#include <people_msgs/PositionMeasurementArray.h>

using namespace cv;
using namespace std;
ros::Subscriber groundTruthSub;
ros::Subscriber detectorSub;
boost::shared_ptr<image_transport::ImageTransport> it_;
image_transport::Subscriber sub_depth_;
image_transport::Publisher depth_pub_;
ros::Publisher info_pub_;
ros::Subscriber info_sub_,radar_pose_sub_,leg_pose_sub_,variance_sub_;
float fy,fx,cx,cy;
float personRadx;
float personRady;
float personCamx;
float personCamy;
float personLegx;
float personLegy;
int numofDetection;
float totalDistR;
float totalDistL;
ros::Time timestampS;
int numOfCycles=0;
float covL=0.1;
float covR=0.1;
float realX;
float realY;

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
    for(int i = 0; i < 24; i++) marker.points.push_back(p[i]);
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
				SBox a = groundTruth[i];
				SBox b = detection[j];
				SBox c = boxOverlap(a,b);
	//			printf("Overlap: %.3f\n",boxVolume(c)/(boxVolume(a)+boxVolume(b)-boxVolume(c)));
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

void imageCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{  
	numOfCycles++; 
	cv_bridge::CvImagePtr cv_ptr;
	try     
	{       
		cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1 );
	}
	catch (cv_bridge::Exception& e)
	{       
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	numofDetection++;
	float z = 0;
	float y = 0;
	float x = 0;
	int numPoints=0;

	for(int i=0;i<depth_msg->width;i++){
		for(int j=0;j<depth_msg->height;j++){
			if(j > 350||j < 20 || i < 10 || i > 570)  cv_ptr->image.at<float>(j,i) = 0; 
			if((cv_ptr->image.at<float>(j,i) > 700) &&  (cv_ptr->image.at<float>(j,i) < 3100)){

				z +=  cv_ptr->image.at<float>(j,i)*0.001;        
				x +=  (cv_ptr->image.at<float>(j,i)*0.001) * ((i - 312.463) * (1/628.008));
				y +=  (cv_ptr->image.at<float>(j,i)*0.001) * ((j - 242.325) * (1/628.008));
				numPoints++;
			} else{ 
				cv_ptr->image.at<float>(j,i) = 0;
			}
		}
	}

	if ( !isinf(x) && !isinf(y) && !isinf(z) )
	{
		z = z/numPoints;  
		y = y/numPoints;  
		x = x/numPoints; 
		personCamx=z;
		personCamy=-x; 
		
		realX=((1/covL)*personLegx + (1/covR)*personRadx)/((1/covL)+(1/covR));
		realY=((1/covL)*personLegy + (1/covR)*personRady)/((1/covL)+(1/covR));
		
		//covL=msg->people[0].covariance[0];
		//cout << "Covariance Leg " << covL << endl;
		cout<< "Cam/Rad/Leg " << personCamx << " " << personCamy << " " << personRadx << " " << personRady << " " << covR << " " <<  personLegx << " " << personLegy << " " << covL <<  endl;
		//cout<< " Leg/Rad filter " << realX << " " << realY << endl; 
		//Computing distances
		float distRC=0;
		float distCL=0;
		distRC=sqrt(pow((personCamx-personRadx),2)+pow((personCamy-personRady),2));
		distCL=sqrt(pow((personCamx-personLegx),2)+pow((personCamy-personLegy),2));

		if(!isnan(distRC)|| !isnan(distCL)){
			totalDistR+=distRC;
			totalDistL+=distCL;
			//cout << "Radar current " << distRC << " Average distance " << totalDistR/numofDetection << " Detections " << numofDetection << endl;
			//cout << "Laser current " << distCL << " Average distance " << totalDistL/numofDetection << " Detections " << numofDetection << endl;
				//cout << "Dist radar " << distRC << endl;
				//cout << "Dist Hokuyo " << distCL << endl;
		}

	}
	depth_pub_.publish(cv_ptr->toImageMsg());
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
	personRadx=msg->poses[0].position.x;
	personRady=msg->poses[0].position.y;
	
}

void legPoseCallback(const people_msgs::PositionMeasurementArrayConstPtr& msg){

	
 	for(int i = 0;i<msg->people.size();i++)
        {
	personLegx=msg->people[0].pos.x;
 	personLegy=msg->people[0].pos.y;
	covL=msg->people[0].covariance[0];
	//cout << "Covariance Leg " << covL << endl;
	}
	
}

void varianceCallback(const geometry_msgs::PoseArrayConstPtr& msg){

	covR=msg->poses[0].position.x;
	//cout<<"Covariance Radar " << covR << endl;
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "evaluator");
	ros::NodeHandle nh_;
	groundTruthSub = nh_.subscribe<visualization_msgs::MarkerArray>("/ground", 1, groundTruthCb);
	detectorSub = nh_.subscribe<visualization_msgs::MarkerArray>("/detector", 1, detectorCb);

	image_transport::ImageTransport it_(nh_);
	depth_pub_  = it_.advertise("/person/depth/image_rect_raw", 1);
	sub_depth_ = it_.subscribe("/camera/depth/image_rect_raw", 1, imageCallback);
	info_sub_ = nh_.subscribe("/camera/depth/camera_info",1,infoCallback);
	info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/person/depth/camera_info",1);
	radar_pose_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("/radar_detector_ol/poses",1,radarPoseCallback);
	variance_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("people_tracker/trajectory_acc",1,varianceCallback);
	leg_pose_sub_ = nh_.subscribe<people_msgs::PositionMeasurementArray>("/people_tracker_measurements",1,legPoseCallback); 


	ros::spin();
	return 0;
}

