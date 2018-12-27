#include <ros/ros.h>
#include <stdio.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
ros::Subscriber groundTruthSub;
ros::Subscriber detectorSub;

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
	printf("%.3f %.3f %.3f %.3f %.3f %.3f\n",result.minX,result.maxX,result.minY,result.maxY,result.minZ,result.maxZ);
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
	printf("Queues: %d %d\n",groundTruth.size(),detection.size());
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
				printf("Overlap: %.3f\n",boxVolume(c)/(boxVolume(a)+boxVolume(b)-boxVolume(c)));
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


int main(int argc, char **argv) 
{
  ros::init(argc, argv, "evaluator");
  ros::NodeHandle nh_;
  groundTruthSub = nh_.subscribe<visualization_msgs::MarkerArray>("/ground", 1, groundTruthCb);
  detectorSub = nh_.subscribe<visualization_msgs::MarkerArray>("/detector", 1, detectorCb);

  ros::spin();
  return 0;
}

