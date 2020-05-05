#include "stdafx.h"
#include "math.h"
#include "stdio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define TIME_STEP 16
#define PI 3.1415926

int main(int argc, char **argv)
{	
	printf("hello\n");	
	double a[1200][3]; double hip[4], knee[4];
	double JOINTCMD_Angle[8];
	int i, j;
	FILE* fp = fopen("C:\\Users\\HIT_K\\Desktop\\angle2.txt", "r");
	FILE *fp1;
	fp1 = fopen("anglegenerate.txt", "w");
	if (fp == NULL)
	{
		printf("文件无效");
		return -1;
	}
	for (i = 0; i < 1200; i++)
	{
		for (j = 0; j < 3; j++)
		{
			fscanf(fp, "%lf", &a[i][j]);
		}
		fscanf(fp, "\n");
	}

	fclose(fp);

	printf("\n");

	double b[1200][2], c[1200][2], d[1200][2];
	int m, k;
	for (i = 0; i < 1200; i++)//腿2
	{

		int num = i + 600;
		if (num < 1200)
			m = num;
		else
			m = num - 1200;
		b[i][0] = -a[m][1];
		b[i][1] = -a[m][2];
	}

	for (i = 0; i <1200; i++)//腿3
	{
		j = 1199 - i;
		c[i][0] = -a[j][1];
		c[i][1] = -a[j][2];
	}

	for (i = 0; i <1200; i++)//腿4
	{
		k = 1199 - i;
		d[i][0] = -b[k][0];
		d[i][1] = -b[k][1];

	}


	for (i = 0; i < 1200; i++)
	{
		printf("%d %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f\n", i, a[i][1], a[i][2], b[i][0], b[i][1], c[i][0], c[i][1], d[i][0], d[i][1]);
		fprintf(fp1, "%d %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f\n", i, a[i][1], a[i][2], b[i][0], b[i][1], c[i][0], c[i][1], d[i][0], d[i][1]);

	}

	while (1) {

		for (int l = 0; l<1200; l++)
		{
			JOINTCMD_Angle[0] = a[l][1];
			JOINTCMD_Angle[1] = a[l][2];
			JOINTCMD_Angle[2] = b[l][0];
			JOINTCMD_Angle[3] = b[l][1];
			JOINTCMD_Angle[4] = c[l][0];
			JOINTCMD_Angle[5] = c[l][1];
			JOINTCMD_Angle[6] = d[l][0];
			JOINTCMD_Angle[7] = d[l][1];

			hip[0]  = JOINTCMD_Angle[0];
			knee[0] = JOINTCMD_Angle[1];
			hip[1]  = JOINTCMD_Angle[2];
			knee[1] = JOINTCMD_Angle[3];
			hip[2]  = JOINTCMD_Angle[4];
			knee[2] = JOINTCMD_Angle[5];
			hip[3]  = JOINTCMD_Angle[6];
			knee[3] = JOINTCMD_Angle[7];

		}

	}
	return 0;
}
