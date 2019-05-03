#include <ros/ros.h>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <limits>

using namespace std;
ros::Subscriber leg_sub_;
ros::Publisher filter_pub_;
sensor_msgs::LaserScan scan;
void legCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

	//Compute x y position of Laser point
	scan.header.stamp =msg->header.stamp;
	scan.header.frame_id = msg->header.frame_id;
	scan.angle_min = msg->angle_min;
	scan.angle_max = msg->angle_max;
	scan.angle_increment = msg->angle_increment;
	scan.time_increment = msg->time_increment;
	scan.range_min = msg->range_min;
	scan.range_max = msg->range_max;
	scan.ranges.resize(msg->ranges.size());
	//Filtering data
	for(int i=0;i<msg->ranges.size();i++){
		
		//Convert sensoric data from polar coordinates
		float angle = msg->angle_min + (i * msg->angle_increment);
		float x = msg->ranges[i]*cos(angle);
		float y = msg->ranges[i]*sin(angle);
		
		//Corridor map filter
		if(isnan(x)|| isnan(y) || y>1.1 || x> 3.1 || y<-1 || x< 0.5 ){
	       		scan.ranges[i] = -1;
	
		}else{
			scan.ranges[i]=msg->ranges[i];
			
			//Used for map testing
			//cout<< "X " << x << endl;
			//cout<< "Y " << y << endl;
		}

	}
	
	filter_pub_.publish(scan);
}
int main(int argc, char **argv) {
	ros::init(argc, argv, "LegFilter");
	ros::NodeHandle nh_;

	leg_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, legCallback);
	filter_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan/filter",10);

	ros::spin();
	return 0;
}

