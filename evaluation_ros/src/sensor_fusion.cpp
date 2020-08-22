#include <stdio.h>
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

ros::Publisher final_poses_kamlan_cnn_,final_poses_switching_svm_,final_poses_kamlan_svm_,final_poses_switching_cnn_;

int laserOutliers,laserMeasurements,radarOutliers,radarMeasurements,kalmanOutliers,switchingOutliers,deepOutliers,deepMeasurements,kalmanDeepOutliers,switchingDeepOutliers;

double camX,camY,camZ, radX,radY,radC,lasX,lasY,lasC,radTX,radTY,lasTX,lasTY,deepX,deepY,deepTX,deepTY,deepC;
double wr,wl,kfX,kfY,radD,lasD,kfD,sfD,sfX,sfY,deepD,wd,kfdX,kfdY,kfdD,sfdX,sfdY,sfdD;

float lastRadX,lastRadY,lastLasX,lastLasY,lastDeepX,lastDeepY;

int  numRad,numLas,numDeep;	

vector<float> camRadDist,camDeepDist,camLasDist,camKfRDist,camSfRDist,camKfDeDist,camSfDeDist;	

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

	int person = 0;

	//experiment 1
	camX  =msg->gt[person].pose.position.z;
	camY  =msg->gt[person].pose.position.x;
	camZ  =msg->gt[person].pose.position.y;

	//experiment 2 - moving
	camX  =msg->gt[person].pose.position.x;
	camY  =msg->gt[person].pose.position.y;
	camZ  =msg->gt[person].pose.position.z;

	//experiment 3 - multi
	camX  =msg->gt[person].pose.position.x;
	camY  =msg->gt[person].pose.position.z;
	camZ  =msg->gt[person].pose.position.y;

	radX =msg->rad[person].pose.position.x;
	radY =msg->rad[person].pose.position.y;
	radC  =msg->rad[person].covariance[0];
	lasX =msg->leg[person].pose.position.x;
	lasY =msg->leg[person].pose.position.y;
	lasC  =msg->leg[person].covariance[0];
	deepX=msg->deep[person].pose.position.x;
	deepY=msg->deep[person].pose.position.y;
	deepC =msg->deep[person].covariance[0];
	/*
	   cout << "rad   "<< msg->rad[person].pose.position.x <<"   "<< msg->rad[person].pose.position.y<<endl;
	   cout << "leg   "<< msg->leg[person].pose.position.x <<"   "<< msg->leg[person].pose.position.y<<endl;
	   cout << "deep   "<< msg->deep[person].pose.position.x <<"   "<< msg->deep[person].pose.position.y<<endl;
	   cout << "gt   "<< msg->gt[person].pose.position.x <<"   "<< msg->gt[person].pose.position.y<< " " <<  camZ << endl;
	   cout << "gt: " << camX << " " << camY << " " << camZ << endl;
	   */

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

	printf("TS/Las/Rad/Deep/KF/SF/KFD/SFD/numLas/DistCam/DistLas/DistRad/DistDeep/DistKF/DistSF/DistKFD/DistSFD/gtx/gty/gtz/radx/rady %f %f %f %f %f %f %f %f %i %f %f %f %f %f %f %f %f %f %f %f %f %f\n",ros::Time::now().toSec(), dist(lasX,lasY,camX,camY),dist(radX,radY,camX,camY),dist(deepX,deepY,camX,camY),dist(kfX,kfY,camX,camY),dist(sfX,sfY,camX,camY),dist(kfdX,kfdY,camX,camY),dist(sfdX,sfdY,camX,camY),numLas, dist(0, 0, camX, camY), dist(0, 0, lasX, lasY), dist(0, 0, radX, radY), dist(0, 0, deepX, deepY), dist(0, 0, kfX, kfY), dist(0, 0, sfX, sfY), dist(0, 0, kfdX, kfdY), dist(0, 0, sfdX, sfdY), camX, camY, camZ, radX, radY);
}


int main(int argc,char* argv[])
{

	ros::init(argc, argv, "sensor_fusion");
	ros::NodeHandle nh_;

	ros::Subscriber variance_deep_sub_ = nh_.subscribe<radar::radar_fusion>("/evaluator_mux",20, inputCallback);
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

