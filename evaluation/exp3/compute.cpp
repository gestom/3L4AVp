#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;



void outlierFilter(vector<float> *x, vector<float> *y)
{
	/**/
	float dx,dy,lx,ly;
	dx=dy=0;
	lx = (*x)[0];
	ly = (*y)[0];
	int trk = 0;
	for (int i = 1;i<x->size();i++)
	{
		lx = (*x)[i];
		ly = (*y)[i];
		dx = (*x)[i]-(*x)[i-1];
		dy = (*y)[i]-(*y)[i-1];
		if (sqrt(dx*dx+dy*dy) > 1.5 && trk < 15)
		{
			(*x)[i] = (*x)[i-1]; 
			(*y)[i] = (*y)[i-1];
			trk++;
		}else{
			trk = 0; 
		}
	}
}

float transformRot(vector<float> xi, vector<float> yi,vector<float> x,vector<float> y,vector<float> *xt,vector<float> *yt,int inject = 0)
{
	Mat T(2,3,CV_32FC1);
	float cxxx = 0;
	float cxxy = 0;
	float rxxx = 0;
	float rxxy = 0;
	Mat A(xi.size(),4,CV_32FC1);
	Mat b(xi.size(),1,CV_32FC1);

	Mat xx(3,xi.size(),CV_32FC1);
	Mat rx(2,xi.size(),CV_32FC1);

	/*center the values*/
	cxxx = cxxy =rxxx = rxxy = 0;
	for (int i = 0;i<xi.size();i++)
	{
		cxxx += x[i]; 	
		cxxy += y[i]; 	
		rxxx += xi[i]; 	
		rxxy += yi[i]; 	
	}
	cxxx = cxxx/xi.size();
	cxxy = cxxy/xi.size();
	rxxx = rxxx/xi.size();
	rxxy = rxxy/xi.size();
	for (int i = 0;i<xi.size();i++){
		x[i] = x[i]-cxxx;
		y[i] = y[i]-cxxy;
		xi[i] = xi[i]-rxxx;
		yi[i] = yi[i]-rxxy;
	}

	/*caltulate trf matrix*/
	for (int i = 0;i<xi.size();i++){
//		if (i == 0 || x[i] != x[i-1]){
			A.at<float>(i,0)=yi[i]*x[i]-xi[i]*y[i];
			A.at<float>(i,1)=yi[i]*y[i]+xi[i]*x[i];
			A.at<float>(i,2)=yi[i];
			A.at<float>(i,3)=-xi[i];
//		}
	}
	A = (A.t()*A);
	cv::SVD svdMat(A);
	cout << svdMat.w << endl; 	//eigenvalues indicate solution quality
	cv::SVD::solveZ(A,b);

	//normalize vector so that it points in a correct direction
	if (b.at<float>(0,0) < 0) b=-b;
	float a0 = b.at<float>(0,0);
	float a1 = b.at<float>(1,0);
	//normalize vector so that first two coefs for cos and sin of a rotation matrix 
	b = b/sqrt(a0*a0+a1*a1);

	//construct the transformation matrix
	a0 = b.at<float>(0,0);
	a1 = b.at<float>(1,0);
	float tx = b.at<float>(2,0);
	float ty = b.at<float>(3,0);
	T.at<float>(0,0) = 1;
	T.at<float>(0,1) = 0;
	T.at<float>(1,0) = 0;
	T.at<float>(1,1) = 1;
	T.at<float>(0,2) = 0;
	T.at<float>(1,2) = 0;
	if (inject == 0){
		a0 = 1;	
		a1 = 0;
		tx = 0;	
		ty = 0;	
	}

	T.at<float>(0,0) = a0; 
	T.at<float>(0,1) = a1; 
	T.at<float>(1,0) = -a1; 
	T.at<float>(1,1) = a0; 
	T.at<float>(0,2) = tx-cxxx+a0*rxxx-a1*rxxy; 
	T.at<float>(1,2) = ty-cxxy+a1*rxxx+a0*rxxy;
	cout << T << endl;

	//transform the points
	for (int i = 0;i<xi.size();i++)
	{
		xx.at<float>(0,i)=x[i]+cxxx;
		xx.at<float>(1,i)=y[i]+cxxy;
		xx.at<float>(2,i)=1;
		rx.at<float>(0,i)=xi[i]+rxxx;
		rx.at<float>(1,i)=yi[i]+rxxy;
	}
	Mat ttx = T*xx;
	Mat e = ttx-rx;

	//calculate error
	float err = 0;
	for (int i = 0;i<xi.size();i++){
		xt->push_back(ttx.at<float>(0,i));
		yt->push_back(ttx.at<float>(1,i));

	       	err += sqrt(e.at<float>(0,i)*e.at<float>(0,i)+e.at<float>(1,i)*e.at<float>(1,i));
	}
	printf("Error: %f\n",err/xi.size());
	return fabs(a1);
}
	


float dist(float x,float y, float rx,float ry)
{
	float dx = x-rx;
	float dy = y-ry;
	return sqrt(dx*dx+dy*dy);
}

int main(int argc,char* argv[])
{
	/*read input*/
	vector<float> camX,camY,radX,radY,radC,lasX,lasY,lasC,radTX,radTY,lasTX,lasTY; 
	if(argc<3)
	{
		fprintf(stderr,"usage: %s referencePoints.txt points.txt \n",argv[0]); 
		return 0;
	}

	ifstream inFile(argv[1]);
	FILE *outFile = fopen(argv[2],"w");
	float cx,cy,rx,ry,rc,lx,ly,lc;
	string ret;

	while(inFile >> ret >> cx >> cy >> rx >>ry >> rc >> lx >> ly >> lc)
	{
		camX.push_back(cx);
		camY.push_back(cy);
		radTX.push_back(rx);
		radTY.push_back(ry);
		radC.push_back(rc);
		lasTX.push_back(lx);
		lasTY.push_back(ly);
		lasC.push_back(lc);
	}
	outlierFilter(&radTX,&radTY);
	outlierFilter(&lasTX,&lasTY);

	/*perform transformations*/
	transformRot(camX,camY,lasTX,lasTY,&lasX,&lasY,1);
	transformRot(camX,camY,radTX,radTY,&radX,&radY,1);
	
	float wr,wl,radD,lasD,kfD,sfD,lasF,radF,kfF,sfF;
	float kfX[100000];
	float kfY[100000];
	float sfX[100000];
	float sfY[100000];
	lasF =radD=lasD=sfD=kfD=0;
	float lastRadX,lastRadY,lastLasX,lastLasY;
	lastLasX=lastLasY=lastRadX=lastRadY  = 10000;
	int numRad,numLas;	
	numRad=numLas=0;	
	int datSize = 0;
	int randi = 0;
	for (int i = 0;i<camX.size();i++)
	{
		if (i > atoi(argv[3])){
			lasX[i] = lastLasX;
			lasY[i] = lastLasY;
		}	
		if (i == atoi(argv[3]) && i==0) {
			radD=lasD=sfD=kfD=0;
			datSize = 0;	
		}
		datSize++;	
		if (radX[i] == lastRadX && radY[i] == lastRadY) numRad++; else numRad = 0; 
		if (lasX[i] == lastLasX && lasY[i] == lastLasY) numLas++; else numLas = 0;
		
		lastRadX = radX[i];
		lastRadY = radY[i];
		lastLasX = lasX[i];
		lastLasY = lasY[i];
		lasC[i] = 1;//lasC[i];
		radC[i] = 1;//radC[i];

		//gradually inflate covariance in case information is obsolete
		wr = 1/(radC[i]*(pow(2,numRad)));
		wl = 1/(lasC[i]*(pow(2,numLas)));

		//kalman filter
		if (isnormal((radX[i]*wr+lasX[i]*wl)/(wr+wl)) && isnormal((radY[i]*wr+lasY[i]*wl)/(wr+wl))){
			kfX[i] = (radX[i]*wr+lasX[i]*wl)/(wr+wl);
			kfY[i] = (radY[i]*wr+lasY[i]*wl)/(wr+wl);
		}else{
			kfX[i] = (radX[i]*(wr+0.1)+lasX[i]*(wl+0.1))/(wr+wl+0.2);
			kfY[i] = (radY[i]*(wr+0.1)+lasY[i]*(wl+0.1))/(wr+wl+0.2);
		}
		//switching filter
		if (wr > wl) {
			sfX[i] = radX[i];
			sfY[i] = radY[i];
		}else{
			sfX[i] = lasX[i];
			sfY[i] = lasY[i];
		}
		lasF = radF = sfF = kfF = 0; 
		int numF = 0;
		for (int k = max(0,i-atoi(argv[4]));k<i;k++){
			float filT = 0.999;
			lasF += dist(lasX[k],lasY[k],camX[k],camY[k]);
			radF += dist(radX[k],radY[k],camX[k],camY[k]);
			sfF += dist(sfX[k],sfY[k],camX[k],camY[k]);
			if (isnormal(dist(kfX[k],kfY[k],camX[k],camY[k]))){ 
				kfF += dist(kfX[k],kfY[k],camX[k],camY[k]);
			}
			numF++;
		}
		lasF = lasF/numF;
		radF = radF/numF;
		sfF = sfF/numF;
		kfF = kfF/numF;

		lasD += dist(lasX[i],lasY[i],camX[i],camY[i]);
		radD += dist(radX[i],radY[i],camX[i],camY[i]);
		sfD += dist(sfX[i],sfY[i],camX[i],camY[i]);
		if (isnormal(dist(kfX[i],kfY[i],camX[i],camY[i]))){ 
			kfD += dist(kfX[i],kfY[i],camX[i],camY[i]);
		}
		//printf("Las/Rad/KF/SF %f %f %f %f %i\n",dist(lasX[i],lasY[i],camX[i],camY[i]),dist(radX[i],radY[i],camX[i],camY[i]),dist(kfX,kfY,camX[i],camY[i]),dist(sfX,sfY,camX[i],camY[i]),numLas);
		//fprintf(outFile,"BDY %f %f %f %f %i\n",radX[i],radY[i],camX[i],camY[i],numLas);
		//fprintf(outFile,"BDY %f %f %f %f %i\n",lasX[i],lasY[i],camX[i],camY[i],numLas);
		fprintf(outFile,"ERR Las/Rad/KF/SF %f %f %f %f %i\n",dist(lasX[i],lasY[i],camX[i],camY[i]),dist(radX[i],radY[i],camX[i],camY[i]),dist(kfX[i],kfY[i],camX[i],camY[i]),dist(sfX[i],sfY[i],camX[i],camY[i]),numLas);
		fprintf(outFile,"SUM Las/Rad/KF/SF %f %f %f %f %i\n",lasD/(i+1),radD/(i+1),kfD/(i+1),sfD/(i+1),numLas);
		fprintf(outFile,"SLW Las/Rad/KF/SF %f %f %f %f %i\n",lasF,radF,kfF,sfF,numLas);
	}
	lasD = lasD/datSize;
	radD = radD/datSize;
	sfD = sfD/datSize;
	kfD = kfD/datSize;
	printf("Sum Las/Rad/KF/SF %.1f %.1f %.1f %.1f %i\n",100*lasD,100*radD,100*kfD,100*sfD,randi);

	return 0;
}

