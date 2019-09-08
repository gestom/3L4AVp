#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;
float temporalFilter = 0.5;
float outlierDistance = 0.5;

float outlierFilter(vector<float> *x, vector<float> *y)
{
	/*aka temporal filter*/
	int falsePositives = 0;
	int values = 0;
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
		if (sqrt(dx*dx+dy*dy) > temporalFilter && trk < 15)
		{
			(*x)[i] = (*x)[i-1]; 
			(*y)[i] = (*y)[i-1];
			trk++;
		}else{
			trk = 0;
		}
		//printf("%f %f %f %f %f\n",lx,ly,(*x)[i],(*y)[i],sqrt(dx*dx+dy*dy));
	}
	return (float)falsePositives/values;
}

void transformRot(vector<float> xi, vector<float> yi,vector<float> x,vector<float> y,vector<float> nfx,vector<float> nfy,vector<float> *xt,vector<float> *yt,vector<float> *nfxt,vector<float> *nfyt)
{
	Mat T(2,3,CV_32FC1);
	float cxxx = 0;
	float cxxy = 0;
	float rxxx = 0;
	float rxxy = 0;
	//printf("%i\n",xi.size());
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
	//cout << svdMat.w << endl; 	//eigenvalues indicate solution quality
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
	T.at<float>(0,0) = a0; 
	T.at<float>(0,1) = a1; 
	T.at<float>(1,0) = -a1; 
	T.at<float>(1,1) = a0; 
	T.at<float>(0,2) = tx-cxxx+a0*rxxx-a1*rxxy; 
	T.at<float>(1,2) = ty-cxxy+a1*rxxx+a0*rxxy;
	//cout << T << endl;

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

	//printf("A\n");
	//calculate error
	float err = 0;
	for (int i = 0;i<xi.size();i++){
		xt->push_back(ttx.at<float>(0,i));
		yt->push_back(ttx.at<float>(1,i));
	       	err += sqrt(e.at<float>(0,i)*e.at<float>(0,i)+e.at<float>(1,i)*e.at<float>(1,i));
//		printf("%.3f %.3f\n",ttx.at<float>(0,i),ttx.at<float>(1,i));
		//printf("AU %.3f %.3f %.3f %.3f\n",ttx.at<float>(0,i),ttx.at<float>(1,i),xi[i]+rxxx,yi[i]+rxxy);
	}
	//printf("B\n");

	//transform second batch of points
	Mat nfxx(3,nfx.size(),CV_32FC1);
	for (int i = 0;i<nfx.size();i++)
	{
		nfxx.at<float>(0,i)=nfx[i];
		nfxx.at<float>(1,i)=nfy[i];
	}
	Mat nfttx = T*nfxx;

	for (int i = 0;i<nfx.size();i++){
		nfxt->push_back(nfttx.at<float>(0,i));
		nfyt->push_back(nfttx.at<float>(1,i));
//		printf("NF %.3f %.3f %.3f %.3f\n",nfttx.at<float>(0,i),nfttx.at<float>(1,i),xi[i]+rxxx,yi[i]+rxxy);
	}
//	printf("C\n");

//	printf("Error: %f\n",err/xi.size());
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
	int laserOutliers,laserMeasurements,radarOutliers,radarMeasurements,kalmanOutliers,switchingOutliers;
	laserOutliers=laserMeasurements=radarOutliers=radarMeasurements=switchingOutliers=kalmanOutliers=0;

	vector<float> camX,camY,radX,radY,radC,lasX,lasY,lasC,radTX,radTY,lasTX,lasTY,deepX,deepY,deepTX,deepTY; 
	vector<float> radNFX,radNFY,lasNFX,lasNFY,deepNFX,deepNFY;
	vector<float> radNX,radNY,lasNX,lasNY,deepNX,deepNY;
	if(argc<3)
	{
		fprintf(stderr,"usage: %s referencePoints.txt points.txt \n",argv[0]); 
		return 0;
	}

	ifstream inFile(argv[1]);
	FILE *outFile = fopen(argv[2],"w");
	float cx,cy,rx,ry,rc,lx,ly,lc,dx,dy;
	string ret;

	while(inFile >> ret >> cx >> cy >> rx >>ry >> rc >> lx >> ly >> lc >> dx >> dy)
	{
		if (rx*ry !=0){
			camX.push_back(cx);
			camY.push_back(cy);
			radTX.push_back(rx);
			radTY.push_back(ry);
			radC.push_back(rc);
			lasTX.push_back(lx);
			lasTY.push_back(ly);
			lasC.push_back(lc);
			deepTX.push_back(dx);
			deepTY.push_back(dy);
		}

	}
	radNX.reserve(radTX.size());
	radNY.reserve(radTY.size());
	lasNX.reserve(lasTX.size());
	lasNY.reserve(lasTY.size());
	deepNX.reserve(deepTX.size());
	deepNY.reserve(deepTY.size());

	copy(radTX.begin(),radTX.end(),back_inserter(radNX));
	copy(radTY.begin(),radTY.end(),back_inserter(radNY));
	copy(lasTX.begin(),lasTX.end(),back_inserter(lasNX));
	copy(lasTY.begin(),lasTY.end(),back_inserter(lasNY));
	copy(deepTX.begin(),deepTX.end(),back_inserter(deepNX));
	copy(deepTY.begin(),deepTY.end(),back_inserter(deepNY));
	

	float radarFPs = outlierFilter(&radTX,&radTY);
	float laserFPs = outlierFilter(&lasTX,&lasTY);
	float deepFPs = outlierFilter(&deepTX,&deepTY);

	/*perform transformations*/
	transformRot(camX,camY,radTX,radTY,radNX,radNY,&radX,&radY,&radNFX,&radNFY);
	transformRot(camX,camY,lasTX,lasTY,lasNX,lasNY,&lasX,&lasY,&lasNFX,&lasNFY);
	transformRot(camX,camY,deepTX,deepTY,deepNX,deepNY,&deepX,&deepY,&deepNFX,&deepNFY);
	//return 0;

	float wr,wl,kfX,kfY,radD,lasD,kfD,sfD,sfX,sfY,deepD;
	radD=lasD=sfD=kfD=deepD=0;
	float lastRadX,lastRadY,lastLasX,lastLasY,lastDeepX,lastDeepY;
	lastLasX=lastLasY=lastRadX=lastRadY = lastDeepX =lastDeepY = 10000;
	int numRad,numLas,numDeep;	
	numRad=numLas=numDeep=0;	
	for (int i = 0;i<camX.size();i++)
	{
		if (radX[i] == lastRadX && radY[i] == lastRadY) numRad++; else numRad = 0; 
		if (lasX[i] == lastLasX && lasY[i] == lastLasY) numLas++; else numLas = 0;
		lastRadX = radX[i];
		lastRadY = radY[i];
		lastLasX = lasX[i];
		lastLasY = lasY[i];

		//gradually inflate covariance in case information is obsolete
		wr = 1/(radC[i]*(pow(2,numRad)));
		wl = 1/(lasC[i]*(pow(2,numLas)));

		//kalman filter
		kfX = (radX[i]*wr+lasX[i]*wl)/(wr+wl);
		kfY = (radY[i]*wr+lasY[i]*wl)/(wr+wl);

		//switching filter
		if (wr > wl) {
			sfX = radX[i];
			sfY = radY[i];
		}else{
			sfX = lasX[i];
			sfY = lasY[i];
		}
		if (numLas == 0){
			if (dist(lasX[i],lasY[i],camX[i],camY[i]) > outlierDistance) laserOutliers++; 
			laserMeasurements++;
		}
		if (numRad == 0){
			if (dist(radX[i],radY[i],camX[i],camY[i]) > outlierDistance) radarOutliers++; 
			radarMeasurements++;
		}
		lasD += dist(lasX[i],lasY[i],camX[i],camY[i]);
		radD += dist(radX[i],radY[i],camX[i],camY[i]);
		deepD += dist(deepX[i],deepY[i],camX[i],camY[i]);
		kfD += dist(kfX,kfY,camX[i],camY[i]);
		sfD += dist(sfX,sfY,camX[i],camY[i]);
		if (dist(kfX,kfY,camX[i],camY[i]) > outlierDistance){
			//printf("OULIER %i\n",i);
		       	kalmanOutliers++; 
		}
		if (dist(sfX,sfY,camX[i],camY[i]) > outlierDistance){
			//printf("SOULIER %i\n",i);
		       	switchingOutliers++; 
		}
		//printf("Las/Rad/KF/SF %f %f %f %f %i\n",dist(lasX[i],lasY[i],camX[i],camY[i]),dist(radX[i],radY[i],camX[i],camY[i]),dist(kfX,kfY,camX[i],camY[i]),dist(sfX,sfY,camX[i],camY[i]),numLas);
		fprintf(outFile,"ERR Las/Rad/KF/SF/Deep %f %f %f %f %f %i\n",dist(lasX[i],lasY[i],camX[i],camY[i]),dist(radX[i],radY[i],camX[i],camY[i]),dist(kfX,kfY,camX[i],camY[i]),dist(sfX,sfY,camX[i],camY[i]),dist(deepX[i],deepY[i],camX[i],camY[i]),numLas);
		fprintf(outFile,"SUM Las/Rad/KF/SF/Deep %f %f %f %f %f %i\n",lasD/(i+1),radD/(i+1),kfD/(i+1),sfD/(i+1),deepD/(i+1),numLas);
	}
	int siz = camX.size();
	lasD = lasD/siz;
	deepD = deepD/siz;
	radD = radD/siz;
	sfD = sfD/siz;
	kfD = kfD/siz;
	printf("Sum Las/Rad/KF/SF/Deep %.1f %.1f %.1f %.1f %.1f \n",100*lasD,100*radD,100*kfD,100*sfD,deepD);
	//siz = 100;
	printf("FPs Las/Rad/KF/SF %.1f %.1f %.1f %.1f\n",100.0*laserOutliers/laserMeasurements,100.0*radarOutliers/radarMeasurements,100.0*kalmanOutliers/siz,100.0*switchingOutliers/siz);

	return 0;
}

