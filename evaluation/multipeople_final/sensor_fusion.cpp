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


float dist(float x,float y, float rx,float ry)
{
	float dx = x-rx;
	float dy = y-ry;
	return sqrt(dx*dx+dy*dy);
}

int main(int argc,char* argv[])
{
	/*read input*/

	vector<float> cam,camY,rad,radY,radC,las,lasY,lasC,radTX,radTY,lasTX,lasTY,deep,deepY,deepTX,deepTY,deepC;
	vector<float> switc,kalm,switd,kald; 

	if(argc<3)
	{
		fprintf(stderr,"usage: %s referencePoints.txt outputpoints.txt slidingAverageOutput.txt \n",argv[0]); 
		return 0;
	}

	ifstream inFile(argv[1]);
	FILE *outFile = fopen(argv[2],"w");
	FILE *outAvgFile = fopen(argv[3],"w");
	float cx,cy,rx,ry,rc,lx,ly,lc,dx,dy,dc;
	string ret;
	float l,r,d,sf,kf,kfd,sfd;
	float time;
	while(inFile >> ret >> time >> l >> r >> d >> kf >> sf >> kfd >> sfd )
	{
			deep.push_back(d);
			rad.push_back(r);
			las.push_back(l);
			switc.push_back(sf);
			kalm.push_back(kf);
			switd.push_back(sfd);
			kald.push_back(kfd);

		}

	
	float ld,rd,dd,kfdi,sfdi,kfdd,sfdd;
	ld = rd = dd = kfdi= sfdi= kfdd= sfdd = 0;
	for(int i = 0;i<deep.size();i++){
		ld += las[i];
		rd += rad[i];
		dd += deep[i];
		kfdi += kalm[i];
		sfdi += switc[i];
		kfdd += kald[i];
		sfdd += switd[i];


		fprintf(outFile,"SUM Las/Rad/KF/SF/Deep %f %f %f %f %f %f %f %i\n",ld/(i+1),rd/(i+1), dd/(i+1),kfdi/(i+1),sfdi/(i+1),kfdd/(i+1),sfdd/(i+1),i);

	}
}

/*	radNX.reserve(radTX.size());
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
*/


	/*perform transformations*/
	//transformRot(camX,camY,radTX,radTY,radNX,radNY,&radX,&radY,&radNFX,&radNFY);
	//transformRot(camX,camY,lasTX,lasTY,lasNX,lasNY,&lasX,&lasY,&lasNFX,&lasNFY);
	//transformRot(camX,camY,deepTX,deepTY,deepNX,deepNY,&deepX,&deepY,&deepNFX,&deepNFY);
/*		if (numRad == 0){
			if (dist(radX[i],radY[i],camX[i],camY[i]) > outlierDistance) radarOutliers++; 
			radarMeasurements++;
		}
		if (numDeep == 0){
			if (dist(deepX[i],deepY[i],camX[i],camY[i]) > outlierDistance) deepOutliers++; 
			deepMeasurements++;
		}
		lasD += dist(lasX[i],lasY[i],camX[i],camY[i]);
		radD += dist(radX[i],radY[i],camX[i],camY[i]);
		deepD += dist(deepX[i],deepY[i],camX[i],camY[i]);
		kfD += dist(kfX,kfY,camX[i],camY[i]);
		sfD += dist(sfX,sfY,camX[i],camY[i]);
		kfdD += dist(kfdX,kfdY,camX[i],camY[i]);
		sfdD += dist(sfdX,sfdY,camX[i],camY[i]);
		camLasDist.push_back(dist(lasX[i],lasY[i],camX[i],camY[i]));
		camRadDist.push_back(dist(radX[i],radY[i],camX[i],camY[i]));
		camDeepDist.push_back(dist(deepX[i],deepY[i],camX[i],camY[i]));
		camKfRDist.push_back(dist(kfX,kfY,camX[i],camY[i]));
		camSfRDist.push_back(dist(sfX,sfY,camX[i],camY[i]));
		camKfDeDist.push_back(dist(kfdX,kfdY,camX[i],camY[i]));
		camSfDeDist.push_back(dist(sfdX,sfdY,camX[i],camY[i]));
		if (dist(kfX,kfY,camX[i],camY[i]) > outlierDistance){
			//printf("OULIER %i\n",i);
			kalmanOutliers++; 
		}
		if (dist(sfX,sfY,camX[i],camY[i]) > outlierDistance){
			//printf("SOULIER %i\n",i);
			switchingOutliers++; 
		}
		if (dist(kfdX,kfdY,camX[i],camY[i]) > outlierDistance){
			//printf("OULIER %i\n",i);
			kalmanDeepOutliers++; 
		}
		if (dist(sfdX,sfdY,camX[i],camY[i]) > outlierDistance){
			//printf("SOULIER %i\n",i);
			switchingDeepOutliers++; 
		}
		//printf("Las/Rad/KF/SF %f %f %f %f %i\n",dist(lasX[i],lasY[i],camX[i],camY[i]),dist(radX[i],radY[i],camX[i],camY[i]),dist(kfX,kfY,camX[i],camY[i]),dist(sfX,sfY,camX[i],camY[i]),numLas);
		fprintf(outFile,"ERR Las/Rad/KF/SF/Deep/KFD/SFD %f %f %f %f %f %f %f %i\n",dist(lasX[i],lasY[i],camX[i],camY[i]),dist(radX[i],radY[i],camX[i],camY[i]),dist(kfX,kfY,camX[i],camY[i]),dist(sfX,sfY,camX[i],camY[i]),dist(deepX[i],deepY[i],camX[i],camY[i]),dist(kfdX,kfdY,camX[i],camY[i]),dist(sfdX,sfdY,camX[i],camY[i]),numLas);
		fprintf(outFile,"SUM Las/Rad/KF/SF/Deep %f %f %f %f %f %f %f %i\n",lasD/(i+1),radD/(i+1),kfD/(i+1),sfD/(i+1),deepD/(i+1),kfdD/(i+1),sfdD/(i+1),numLas);
	}

	//Sliding average
	int sample_size = 4;
	bool init=true;
	vector<float> camRadADist,camDeepADist,camLasADist,camKfRADist,camSfRADist,camKfDeADist,camSfDeADist;	
	for (int i = 0;i<camX.size();i++)
	{

		//Fill first x values
		if(init){
			camRadADist.push_back(camRadDist[i]);
			camDeepADist.push_back(camDeepDist[i]);
			camLasADist.push_back(camLasDist[i]);
			camKfRADist.push_back(camKfRDist[i]);
			camSfRADist.push_back(camSfRDist[i]);
			camKfDeADist.push_back(camKfDeDist[i]);
			camSfDeADist.push_back(camSfDeDist[i]);

			if(i==sample_size-1) init = false;
			continue;
		}
		float rd,ld,dd,krd,srd,kdd,sdd; 
		rd=ld=dd=krd=srd=kdd=sdd=0;
		
		//Get average
		for(int j=0; j<sample_size;j++){
			rd += camRadADist[j];
			ld += camLasADist[j];
			dd += camDeepADist[j];
			krd += camKfRADist[j];
			srd += camSfRADist[j];
			kdd += camKfDeADist[j];
			sdd += camSfDeADist[j];


		}

		fprintf(outAvgFile,"SlidingAvaverage rad/las/deep/kf/sf/kfDeep/sfDeep %f %f %f %f %f %f %f \n",rd/sample_size,ld/sample_size,dd/sample_size,krd/sample_size,srd/sample_size,kdd/sample_size,sdd/sample_size);


		//Pop first element, add last
		camRadADist.erase(camRadADist.begin());
		camDeepADist.erase(camDeepADist.begin());
		camLasADist.erase(camLasADist.begin());
		camKfRADist.erase(camKfRADist.begin());
		camSfRADist.erase(camSfRADist.begin());
		camKfDeADist.erase(camKfDeADist.begin());
		camSfDeADist.erase(camSfDeADist.begin());
		
		camRadADist.push_back(camRadDist[i]);
		camDeepADist.push_back(camDeepDist[i]);
		camLasADist.push_back(camLasDist[i]);
		camKfRADist.push_back(camKfRDist[i]);
		camSfRADist.push_back(camSfRDist[i]);
		camKfDeADist.push_back(camKfDeDist[i]);
		camSfDeADist.push_back(camSfDeDist[i]);

	}


	int siz = camX.size();
	lasD = lasD/siz;
	deepD = deepD/siz;
	radD = radD/siz;
	sfD = sfD/siz;
	kfD = kfD/siz;
	sfdD = sfdD/siz;
	kfdD = kfdD/siz;
	printf("Sum Las/Rad/KF/SF/Deep %.1f %.1f %.1f %.1f %.1f %.1f %1.f \n",100*lasD,100*radD,100*kfD,100*sfD,deepD,100*kfdD,100*sfdD);
	//siz = 100;
	printf("FPs Las/Rad/KF/SF/FKD/SFD %.1f %.1f %.1f %.1f %.1f %.1f \n",100.0*laserOutliers/laserMeasurements,100.0*radarOutliers/radarMeasurements,100.0*kalmanOutliers/siz,100.0*switchingOutliers/siz,100.0*kalmanDeepOutliers/siz,100.0*switchingDeepOutliers/siz);

	return 0;
}*/

