#include <stdio.h>
#include <iostream>
#include "dbscan.h"

#define MINIMUM_POINTS 5     // minimum number of cluster
#define EPSILON (0.4)  // distance for clustering, metre^2

void printResults(vector<Point>& points, int num_points)
{
    int i = 0;
    printf("Number of points: %u\n"
        " x     y     z     cluster_id\n"
        "-----------------------------\n"
        , num_points);
    while (i < num_points)
    {
          printf("%5.2lf %5.2lf : %d\n",
                 points[i].x,
                 points[i].y,
                 points[i].clusterID);
          ++i;
    }
}


float thr;
int gt;
float precision[100000];
float recall[100000];

int main(int argc,char *argv[])
{
	FILE* file = fopen(argv[1],"r");
	int n = 0;
	vector<Point> points;
	Point point;
	float x,y,z;
	int allPositives = 0;
	int pads = 0;
	float sum = 0;
	float fifi = 0;
	while (feof(file) == 0){
		fscanf(file,"%f %i %f %f\n",&thr,&gt,&x,&y);
		if (x == 20 && y == 20) pads++;
		point.x = x; 
		point.y = y; 
		point.z = 10; 
		if (n == 0) fifi = thr;
		point.thr = thr - fifi;
	        sum += point.thr;	
		point.gt = gt; 
		if (gt == 1) allPositives++;
		point.clusterID = UNCLASSIFIED; 
		points.push_back(point);
		n++;
	}
	for (int i = 0;i<n;i++){
		// constructor
		DBSCAN ds(MINIMUM_POINTS, EPSILON, points);

		// main loop
		ds.run();

		int falsePositives = 0;
		int falseNegatives = 0;
		int truePositives = 0;

		// result of DBSCAN algorithm
//		printResults(ds.m_points, ds.getTotalPointSize());
		int clusterPos[50];
		for (int i = 0;i<50;i++) clusterPos[i] = -1;
		for (int i = 0;i<ds.getTotalPointSize();i++)
		{
			if (ds.m_points[i].clusterID > -1){
				if (ds.m_points[i].gt == 1) clusterPos[ds.m_points[i].clusterID]  = 1;
				if (ds.m_points[i].gt == 0 && clusterPos[ds.m_points[i].clusterID] != 1) clusterPos[ds.m_points[i].clusterID] = 0;
			}
		}
		for (int i = 0;i<50;i++){
		       if (clusterPos[i] == 0) falsePositives++;	
		       if (clusterPos[i] == 1) truePositives++;	
		}
		falseNegatives = 1-truePositives;
		precision[0] = 1;
		if (truePositives+falsePositives > 0){
			precision[0] = 1.0*truePositives/(truePositives+falsePositives);
		}
		recall[0] = 1.0*truePositives/(truePositives+falseNegatives);

		float avep = 0;
		printf("%f %f %i %i %i %f\n",precision[0],recall[0],truePositives,falsePositives,falseNegatives,(points[0].thr)/(sum)*100/*,(points[0].thr-fifi)*pow(pads/100.0,4)*40*/);
		points.erase (points.begin());
	}
	return 0;
	/*
	   float treshold = 0;
	   int falsePositives = 0;
	   int falseNegatives = 0;
	   int truePositives = 0;	
	   for (int i = 0;i<n;i++)truePositives+=gt[i];
	   falsePositives = n-truePositives;
	   for (int i = 0;i<n;i++){
	   if (gt[i] == 0) falsePositives--;	
	   if (gt[i] == 1) {falseNegatives++;truePositives--;}
	   precision[i] = 1.0*truePositives/(truePositives+falsePositives);	
	   recall[i] = 1.0*truePositives/(truePositives+falseNegatives);
	   }
	   float avep = 0;
	   for (int i = n-2;i>0;i--)
	   {
	   for (int j = i;j>0;j--)
	   {
	   if (precision[i] < precision[j]) precision[i] = precision[j];
	   }
	   if (recall[i] != recall[i-1]){
	   avep += precision[i]*(recall[i-1]-recall[i]);
	   }
	   printf("%f %f %i %i %i %f\n",precision[i],recall[i],truePositives,falsePositives,falseNegatives,avep/recall[i]);
	   }
	   return 0;*/
}
