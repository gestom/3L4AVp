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
		point.thr = thr;// - fifi;
		sum += point.thr;
		point.gt = gt; 
		if (gt == 1) allPositives++;
		point.clusterID = UNCLASSIFIED; 
		points.push_back(point);
		n++;
	}
	int falsePositives[100];
	int falseNegatives[100];
	int truePositives[100];
	float threshold[101];
	for (int i = 0;i<101;i++) threshold[i] = 1.0; 
	for (int i = 0;i<n;i++) threshold[i] = points[i].thr;///sum*100; 
	for (int i = 0;i<n;i++){
		falsePositives[i] = 0;
		falseNegatives[i] = 0;
		truePositives[i] = 0;
		// constructor
		DBSCAN ds(MINIMUM_POINTS, EPSILON, points);

		// main loop
		ds.run();


		// result of DBSCAN algorithm
		//printResults(ds.m_points, ds.getTotalPointSize());
		int clusterPos[50];
		for (int i = 0;i<50;i++) clusterPos[i] = -1;
		for (int i = 0;i<ds.getTotalPointSize();i++)
		{
			if (ds.m_points[i].clusterID > -1){
				if (ds.m_points[i].gt == 1) clusterPos[ds.m_points[i].clusterID]  = 1;
				if (ds.m_points[i].gt == 0 && clusterPos[ds.m_points[i].clusterID] != 1) clusterPos[ds.m_points[i].clusterID] = 0;
			}
		}
		for (int k = 0;k<50;k++){
			if (clusterPos[k] == 0) falsePositives[i]++;	
			if (clusterPos[k] == 1) truePositives[i]++;	
		}
		falseNegatives[i] = 1-truePositives[i];
		precision[i] = 1;
		if (truePositives[i]+falsePositives[i] > 0){
			precision[i] = 1.0*truePositives[i]/(truePositives[i]+falsePositives[i]);
		}
		recall[i] = 1.0*truePositives[i]/(truePositives[i]+falseNegatives[i]);

		float avep = 0;
		points.erase (points.begin());
	}
	for (int i = 0;i<n;i++){
		//printf("%f %f %i %i %i %f\n",precision[i],recall[i],truePositives[i],falsePositives[i],falseNegatives[i],threshold[i]);
	}
	int p = 0;
	threshold[100] = 100;
	for (float t = 0;t<1.0;t=t+0.001)
	{
		p = 0;
		while (threshold[p] < t) p++;
		printf("PR: %i %i %i %i %f %f\n",truePositives[p],falsePositives[p],falseNegatives[p],p,threshold[p],t);
	}	
	return 0;
}
