#include <stdio.h>
#include <stdlib.h>

float trh[100000];
int gt[100000];
float precision[100000];
float recall[100000];

int main(int argc,char *argv[])
{
	FILE* file = fopen(argv[1],"r");
	int n = 0;
	while (feof(file) == 0){
		fscanf(file,"%f %i\n",&trh[n],&gt[n]);
		n++;
	}
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
	return 0;
}
