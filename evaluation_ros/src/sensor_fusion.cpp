#include <stdio.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <radar/radar_fusion.h>
#include <geometry_msgs/PoseArray.h>
using namespace std;
float temporalFilter = 0.5;
float outlierDistance = 0.5;

ros::Publisher final_poses_kamlan_cnn_,final_poses_switching_svm_,final_poses_kamlan_svm_,final_poses_switching_cnn_, reremuxer;

int laserOutliers,laserMeasurements,radarOutliers,radarMeasurements,kalmanOutliers,switchingOutliers,deepOutliers,deepMeasurements,kalmanDeepOutliers,switchingDeepOutliers;

double camX,camY,camZ, radX,radY,radC,lasX,lasY,lasC,radTX,radTY,lasTX,lasTY,deepX,deepY,deepTX,deepTY,deepC;
double wr,wl,kfX,kfY,radD,lasD,kfD,sfD,sfX,sfY,deepD,wd,kfdX,kfdY,kfdD,sfdX,sfdY,sfdD;

float lastRadX,lastRadY,lastLasX,lastLasY,lastDeepX,lastDeepY;

int  numRad,numLas,numDeep;	

vector<float> camRadDist,camDeepDist,camLasDist,camKfRDist,camSfRDist,camKfDeDist,camSfDeDist;	
int exp_type_;
int person_;

//Sliding average
bool init_avg = true;
int sample_counter = 0;
int sample_size = 200;
float rd,ld,dd,krd,srd,kdd,sdd; 
vector<float> camRadADist,camDeepADist,camLasADist,camKfRADist,camSfRADist,camKfDeADist,camSfDeADist;	

float dist(float x,float y, float rx,float ry)
{
	float dx = x-rx;
	float dy = y-ry;
	return sqrt(dx*dx+dy*dy);
}


void inputCallback(const radar::radar_fusionConstPtr& msg)
{
	std_msgs::Header header;
	geometry_msgs:: PoseArray ps1;
	geometry_msgs:: Pose pos1;
	geometry_msgs:: PoseArray ps2;
	geometry_msgs:: Pose pos2;
	geometry_msgs:: PoseArray ps3;
	geometry_msgs:: Pose pos3;
	geometry_msgs:: PoseArray ps4;
	geometry_msgs:: Pose pos4;
	ps1.header=msg->header;
	ps2.header=msg->header;
	ps3.header=msg->header;
	ps4.header=msg->header;

	switch(exp_type_){

		case 1:
			//experiment 1
			camX  =msg->gt[person_].pose.position.z;
			camY  =msg->gt[person_].pose.position.x;
			camZ  =msg->gt[person_].pose.position.y;
			break;
		case 2:
			//experiment 2 - moving
			camX  =msg->gt[person_].pose.position.x;
			camY  =msg->gt[person_].pose.position.y;
			camZ  =msg->gt[person_].pose.position.z;
			break;

		case 3:
			//experiment 3 - multi
			camX  =msg->gt[person_].pose.position.x;
			camY  =-msg->gt[person_].pose.position.y;
			camZ  =msg->gt[person_].pose.position.z;
			break;	
		default:
			//experiment 1
			camX  =msg->gt[person_].pose.position.z;
			camY  =msg->gt[person_].pose.position.x;
			camZ  =msg->gt[person_].pose.position.y;
	}
	radX =msg->rad[person_].pose.position.x;
	radY =msg->rad[person_].pose.position.y;
	radC  =msg->rad[person_].covariance[0];
	lasX =msg->leg[person_].pose.position.x;
	lasY =msg->leg[person_].pose.position.y;
	lasC  =msg->leg[person_].covariance[0];
	deepX=msg->deep[person_].pose.position.x;
	deepY=msg->deep[person_].pose.position.y;
	deepC =msg->deep[person_].covariance[0];
	/*cout << "rad   "<< msg->rad[0].pose.position.x <<"   "<< msg->rad[0].pose.position.y<<endl;
	cout << "leg   "<< msg->leg[0].pose.position.x <<"   "<< msg->leg[0].pose.position.y<<endl;
	cout << "deep   "<< msg->deep[0].pose.position.x <<"   "<< msg->deep[0].pose.position.y<<endl;*/
	/*cout << "gt   "<< msg->gt[0].pose.position.x  <<"   "<< msg->gt[0].pose.position.y <<"   "<< msg->gt[0].pose.position.z << endl;
	cout << "gt: " << camX << " " << camY << " " << camZ << endl;*/

  

	if (radX == lastRadX && radY == lastRadY) numRad++; else numRad = 0; 
	if (lasX == lastLasX && lasY == lastLasY) numLas++; else numLas = 0;
	if (deepX== lastDeepX && deepY == lastDeepY) numDeep++; else numDeep = 0;
	lastRadX = radX;
	lastRadY = radY;
	lastLasX = lasX;
	lastLasY = lasY;
	lastDeepX = deepX;
	lastDeepY = deepY;

	//gradually inflate covariance in case information is obsolete
	wr = 1/(radC*(pow(2.0,numRad)));
	wl = 1/(lasC*(pow(2.0,numLas)));
	wd = 1/(deepC*(pow(2.0,numDeep)));

	//kalman filter
	kfX = (radX*wr+lasX*wl)/(wr+wl);
	kfY = (radY*wr+lasY*wl)/(wr+wl);
	pos1.position.x=kfX;
	pos1.position.y=kfY;
	ps1.poses.push_back(msg->gt[0].pose);
	final_poses_kamlan_svm_.publish(ps1);

	kfdX = (deepX*wd+lasX*wl)/(wd+wl);
	kfdY = (deepY*wd+lasY*wl)/(wd+wl);
	pos2.position.x=kfdX;
	pos2.position.y=kfdY;
	ps2.poses.push_back(pos2);
	final_poses_kamlan_cnn_.publish(ps2);
	//switching filter
	if (wr > wl) {
		sfX = radX;
		sfY = radY;
	}else{
		sfX = lasX;
		sfY = lasY;
	}
	if (wd > wl) {
		sfdX = deepX;
		sfdY = deepY;
	}else{
		sfdX = lasX;
		sfdY = lasY;
	}

	pos3.position.x=sfX;
	pos3.position.y=sfY;
	ps3.poses.push_back(pos3);
	final_poses_switching_svm_.publish(ps3);
	pos4.position.x=sfdX;
	pos4.position.y=sfdY;
	ps4.poses.push_back(pos4);
	final_poses_switching_cnn_.publish(ps4);

	if (numLas == 0){
		if (dist(lasX,lasY,camX,camY) > outlierDistance) laserOutliers++; 
		laserMeasurements++;
	}
	if (numRad == 0){
		if (dist(radX,radY,camX,camY) > outlierDistance) radarOutliers++; 
		radarMeasurements++;
	}
	if (numDeep == 0){
		if (dist(deepX,deepY,camX,camY) > outlierDistance) deepOutliers++; 
		deepMeasurements++;
	}
	lasD += dist(lasX,lasY,camX,camY);
	radD += dist(radX,radY,camX,camY);
	deepD += dist(deepX,deepY,camX,camY);
	kfD += dist(kfX,kfY,camX,camY);
	sfD += dist(sfX,sfY,camX,camY);
	kfdD += dist(kfdX,kfdY,camX,camY);
	sfdD += dist(sfdX,sfdY,camX,camY);
	camLasDist.push_back(dist(lasX,lasY,camX,camY));
	camRadDist.push_back(dist(radX,radY,camX,camY));
	camDeepDist.push_back(dist(deepX,deepY,camX,camY));
	camKfRDist.push_back(dist(kfX,kfY,camX,camY));
	camSfRDist.push_back(dist(sfX,sfY,camX,camY));
	camKfDeDist.push_back(dist(kfdX,kfdY,camX,camY));
	camSfDeDist.push_back(dist(sfdX,sfdY,camX,camY));
	if (dist(kfX,kfY,camX,camY) > outlierDistance){
		//printf("OULIER %i\n",i);
		kalmanOutliers++; 
	}
	if (dist(sfX,sfY,camX,camY) > outlierDistance){
		//printf("SOULIER %i\n",i);
		switchingOutliers++; 
	}
	if (dist(kfdX,kfdY,camX,camY) > outlierDistance){
		//printf("OULIER %i\n",i);
		kalmanDeepOutliers++; 
	}
	if (dist(sfdX,sfdY,camX,camY) > outlierDistance){
		//printf("SOULIER %i\n",i);
		switchingDeepOutliers++; 
	}

	cout<< "EVALVERSION-TS/Cam/Rad/Leg/Deep" << " " << ros::Time::now() << " " << camX << " " << camY << " " << radX << " " << radY << " " << radC << " " <<  lasX << " " << lasY << " " << lasC <<  " " << deepX << " " << deepY << " " << deepC << endl;

	printf("TS/Las/Rad/Deep/KF/SF/KFD/SFD/numLas/DistCam/DistLas/DistRad/DistDeep/DistKF/DistSF/DistKFD/DistSFD/gtx/gty/gtz/radx/rady %f %f %f %f %f %f %f %f %i %f %f %f %f %f %f %f %f %f %f %f %f %f\n",ros::Time::now().toSec(), dist(lasX,lasY,camX,camY),dist(radX,radY,camX,camY),dist(deepX,deepY,camX,camY),dist(kfX,kfY,camX,camY),dist(sfX,sfY,camX,camY),dist(kfdX,kfdY,camX,camY),dist(sfdX,sfdY,camX,camY),numLas, dist(0, 0, camX, camY), dist(0, 0, lasX, lasY), dist(0, 0, radX, radY), dist(0, 0, deepX, deepY), dist(0, 0, kfX, kfY), dist(0, 0, sfX, sfY), dist(0, 0, kfdX, kfdY), dist(0, 0, sfdX, sfdY), camX, camY, camZ, radX, radY);


    radar::radar_fusion message;
    geometry_msgs::PoseWithCovariance pose;
    header = msg->header;
    header.frame_id = "map";
    message.header = header;


  std::vector<std::vector<float> > rad;
  std::vector<std::vector<float> > cam;
  std::vector<std::vector<float> > leg;
  std::vector<std::vector<float> > deep;

    pose.pose.position.x = radX;
    pose.pose.position.y = radY;
    rad.push_back(pose);
    message.rad = constructPoseWCovariance(rad);
    pose.pose.position.x = camX;
    pose.pose.position.y = camY;
    cam.push_back(pose);
    message.cam = constructPoseWCovariance(rad);
    message.gt = cam;
    pose.pose.position.x = deepX;
    pose.pose.position.y = deepY;
    deep.push_back(pose);
    message.deep = constructPoseWCovariance(rad);
    message.deep = deep;
    pose.pose.position.x = lasX;
    pose.pose.position.y = lasY;
    leg.push_back(pose);
    message.leg = constructPoseWCovariance(rad);
    reremuxer.publish(message);
}

std::vector<geometry_msgs::PoseWithCovariance>  constructPoseWCovariance (std::vector<std::vector<float> > vec){

  std::vector<geometry_msgs::PoseWithCovariance> field;
  for(unsigned int i = 0;i<vec.size();i++){

    geometry_msgs::PoseWithCovariance pose;
    pose.pose.position.x = vec[i][0];
    pose.pose.position.y= vec[i][1];
    pose.pose.position.z= vec[i][2];
    pose.covariance = { 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0};
    field.push_back(pose);
  }
  return field;
}

int main(int argc,char* argv[])
{

	ros::init(argc, argv, "sensor_fusion");
	ros::NodeHandle nh_;
	
	nh_.getParam("/sensor_fusion/exp_type", exp_type_);
	
	nh_.getParam("/sensor_fusion/person",person_);


	ros::Subscriber variance_deep_sub_ = nh_.subscribe<radar::radar_fusion>("/evaluator_remux",20, inputCallback);
	reremuxer = nh_.advertise<radar::radar_fusion>("/evaluator_reremuxer",1);
	final_poses_kamlan_cnn_ = nh_.advertise<geometry_msgs::PoseArray>("/filter/cnn/wighted_filter/pose",1);
	final_poses_switching_svm_ = nh_.advertise<geometry_msgs::PoseArray>("/filter/svm/switching_filter/pose",1);
	final_poses_kamlan_svm_ = nh_.advertise<geometry_msgs::PoseArray>("/filter/svm/wighted_filter/pose",1);
	final_poses_switching_cnn_ = nh_.advertise<geometry_msgs::PoseArray>("/filter/cnn/switching_filter/pose",1);

	/*read input*/
	radD=lasD=sfD=kfD=deepD=kfdD=sfdD=0;
	lastLasX=lastLasY=lastRadX=lastRadY = lastDeepX =lastDeepY = 10000;
	numRad=numLas=numDeep=0;
	laserOutliers=laserMeasurements=radarOutliers=radarMeasurements=switchingOutliers=kalmanOutliers=deepOutliers=deepMeasurements=kalmanDeepOutliers=switchingDeepOutliers=0;

	ros::spin();
}

