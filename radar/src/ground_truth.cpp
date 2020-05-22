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
#include <radar/ppl.h>
#include <radar/radar_fusion.h>
float fy,fx,cx,cy;
using namespace cv;
using namespace std;
boost::shared_ptr<image_transport::ImageTransport> it_;
image_transport::Subscriber sub_depth_;
image_transport::Publisher depth_pub_;
ros::Publisher camera_ground_truth_publihser_;
ros::Publisher info_pub_;
ros::Subscriber info_sub_;
void imageCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{

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

	float z = 0;
	float y = 0;
	float x = 0;
	int numPoints=0;

	for(int i=0;i<depth_msg->width;i++){
		for(int j=0;j<depth_msg->height;j++){
			if(j > 350||j < 20 || i < 10 || i > 570)  cv_ptr->image.at<float>(j,i) = 0;
			if((cv_ptr->image.at<float>(j,i) > 700) &&  (cv_ptr->image.at<float>(j,i) < 3100)){

				x +=  cv_ptr->image.at<float>(j,i)*0.00082;
				y +=  (cv_ptr->image.at<float>(j,i)*0.001) * ((i - 312.463) * (1/628.008));
				z +=  (cv_ptr->image.at<float>(j,i)*0.001) * ((j - 242.325) * (1/628.008));
				numPoints++;
			} else{
				cv_ptr->image.at<float>(j,i) = 0;
			}
		}
	}

	if ( !isinf(x) && !isinf(y) && !isinf(z) )
	{


		//z = z/numPoints;  
		//y = -y/numPoints;  
		//x = x/numPoints; 

        //exp1: 
		/*z = x/numPoints;  
		x = y/numPoints; 
        y = 0;*/

        //exp2:
		z = z/numPoints;  
		x = x/numPoints; 
        y = -y/numPoints;

    std_msgs::Header header;
    geometry_msgs:: PoseArray ps;
    geometry_msgs:: Pose pos;

    pos.position.x=x;
    pos.position.y=y;
    pos.position.z=z;
    ps.header=depth_msg->header;
    //ps.header.frame_id="map";
    ps.poses.push_back(pos);
    camera_ground_truth_publihser_.publish(ps);
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
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ground_truth");
	ros::NodeHandle nh_;


	image_transport::ImageTransport it_(nh_);
	depth_pub_  = it_.advertise("/person/depth/image_rect_raw", 1);
	sub_depth_ = it_.subscribe("/camera/depth/image_rect_raw", 1, imageCallback);
	info_sub_ = nh_.subscribe("/camera/depth/camera_info",1,infoCallback);
	info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/person/depth/camera_info",1);

  camera_ground_truth_publihser_ = nh_.advertise<geometry_msgs::PoseArray>("/person/ground_truth",1);
  


	ros::spin();
	return 0;
}
