#include "ros/ros.h"
#include "testcan/IpPos.h"
#include "iostream"
#include "fstream"
#include <math.h>
#include "stdio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define TIME_STEP 16
#define PI 3.1415926
#define postoangle 1440000.0/360 
#define big 2498560.0/360
#define degreetoradius 180.0/3.1415926
#define radiustodegree 3.1415926/180.0
using namespace std;

testcan::IpPos ip_pos;
int sendangle(int id, double angle){
    ip_pos.id = id;
    ip_pos.pos = angle*big*degreetoradius;
    return 0;
}

int main(int argc, char **argv)
{	
	printf("hello\n");
    ros::init(argc, argv, "csp2_node");
    ros::NodeHandle nh_;
	printf("start\n");
    ros::Publisher pos_pub = nh_.advertise<testcan::IpPos>("ip_pos",1000);
    usleep(500000);
    cout << "start\n"<<endl;
   
    //初始化复位为四足形式
    sendangle(22, 0);
    pos_pub.publish(ip_pos);
	sendangle(21, 0);
    pos_pub.publish(ip_pos);
	sendangle(25, 0);
    pos_pub.publish(ip_pos);
	sendangle(23, 0);
    pos_pub.publish(ip_pos);
	sendangle(26, 0);
    pos_pub.publish(ip_pos);
	sendangle(27, 0);
    pos_pub.publish(ip_pos);
	sendangle(20, 0);
    pos_pub.publish(ip_pos);
	sendangle(24, 0);
    pos_pub.publish(ip_pos);
	sendangle(17, 0);
    pos_pub.publish(ip_pos);
	sendangle(12, 0);
    pos_pub.publish(ip_pos);
	sendangle(15, 0);
    pos_pub.publish(ip_pos);
	sendangle(10, 0);
    pos_pub.publish(ip_pos);
	sendangle(14, 0);
    pos_pub.publish(ip_pos);
	sendangle(13, 0);
    pos_pub.publish(ip_pos);
	sendangle(11, 0);
    pos_pub.publish(ip_pos);
	sendangle(16, 0);
    pos_pub.publish(ip_pos);
	// sendangle(17, 86.45*radiustodegree);
    // pos_pub.publish(ip_pos);
	// sendangle(12, -158.4*radiustodegree);
    // pos_pub.publish(ip_pos);
	// sendangle(15, -86.45*radiustodegree);
    // pos_pub.publish(ip_pos);
	// sendangle(10, 158.4*radiustodegree);
    // pos_pub.publish(ip_pos);
	// sendangle(14, -86.45*radiustodegree);
    // pos_pub.publish(ip_pos);
	// sendangle(13, 155*radiustodegree);
    // pos_pub.publish(ip_pos);
	// sendangle(11, 86.45*radiustodegree);
    // pos_pub.publish(ip_pos);
	// sendangle(16, -158.4*radiustodegree);
    // pos_pub.publish(ip_pos);
	usleep(5000000);//微秒us

    //四足收腿，延时１０秒
	sendangle(17, 45.0*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(12, -45.0*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(15, -45.0*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(10, 45.0*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(14, -45.0*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(13, 45.0*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(11, 45.0*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(16, -45.0*radiustodegree);
    pos_pub.publish(ip_pos);
	usleep(5000000);


	double a[1200][3]; double hip[4], knee[4];
	// double aa[1200][9];
	double JOINTCMD_Angle[8];
	int i, j;
    ros::Rate loop_rate(200);
	FILE *fp; 
    fp = fopen("catkin_ws/src/testcan/data/angle2.txt","r");
	FILE *fp1;
	fp1 = fopen("catkin_ws/src/testcan/anglegenerate.txt", "w");
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
			// aa[i][j] = a[i][j];
		}
		fscanf(fp, "\n");
	}

	fclose(fp);

	printf("\n");

	double b[1200][2], c[1200][2], d[1200][2];
	int m, k;
	// for (i = 0; i < 1200; i++)
	// {
	// 	for (j = 0; j < 3; j++)
	// 	{
	// 		a[i][j] = aa[1199-i][j];
	// 	}
	// 	fscanf(fp, "\n");
	// }
	// for (i = 0; i < 1200; i++)//腿2
	// {

	// 	int num = i + 600;
	// 	if (num < 1200)
	// 		m = num;
	// 	else
	// 		m = num - 1200;
	// 	b[i][0] = a[m][1];
	// 	b[i][1] = a[m][2];
	// }

	// for (i = 0; i <1200; i++)//腿3
	// {
	// 	j = 1199 - i;
	// 	c[i][0] = a[j][1];
	// 	c[i][1] = a[j][2];
	// }

	// for (i = 0; i <1200; i++)//腿4
	// {
	// 	int num = 600 - i;
	// 	if (num > 0)
	// 		k = num;
	// 	else
	// 		k = 1799 - i;
	// 	d[i][0] = a[k][1];
	// 	d[i][1] = a[k][2];
	// }
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

	while (ros::ok()) {

		for (int l = 0; l<1200; l++)
		{
            if ( !ros::ok())
            {
                break;
            }
			JOINTCMD_Angle[0] = a[l][1];
			JOINTCMD_Angle[1] = a[l][2];
			JOINTCMD_Angle[2] = b[l][0];
			JOINTCMD_Angle[3] = b[l][1];
			JOINTCMD_Angle[4] = c[l][0];
			JOINTCMD_Angle[5] = c[l][1];
			JOINTCMD_Angle[6] = d[l][0];
			JOINTCMD_Angle[7] = d[l][1];

        // hip[0] = -(PI / 2 + JOINTCMD_Angle[0]);
		// knee[0] = JOINTCMD_Angle[1];
		// hip[1] = PI / 2 + JOINTCMD_Angle[2];
		// knee[1] = -JOINTCMD_Angle[3];
		// hip[2] = PI / 2 + JOINTCMD_Angle[4];
		// knee[2] = -JOINTCMD_Angle[5];
		// hip[3] = -(PI / 2 + JOINTCMD_Angle[6]);
		// knee[3] = JOINTCMD_Angle[7];
			hip[0]  = JOINTCMD_Angle[0];
			knee[0] = JOINTCMD_Angle[1];
			hip[1]  = JOINTCMD_Angle[2];
			knee[1] = JOINTCMD_Angle[3];
			hip[2]  = JOINTCMD_Angle[4];
			knee[2] = JOINTCMD_Angle[5];
			hip[3]  = JOINTCMD_Angle[6];
			knee[3] = JOINTCMD_Angle[7];

			sendangle(11,hip[0]);
			pos_pub.publish(ip_pos);
			sendangle(17,hip[3]);
			pos_pub.publish(ip_pos);
			sendangle(15,hip[1]);
			pos_pub.publish(ip_pos);
			sendangle(14,hip[2]);
			pos_pub.publish(ip_pos);
			sendangle(16,knee[0]);
			pos_pub.publish(ip_pos);
			sendangle(12,knee[3]);
			pos_pub.publish(ip_pos);
			sendangle(10,knee[1]);
			pos_pub.publish(ip_pos);
       		sendangle(13,knee[2]);

			// sendangle(14,hip[0]);
			// pos_pub.publish(ip_pos);
			// sendangle(15,hip[3]);
			// pos_pub.publish(ip_pos);
			// sendangle(17,hip[1]);
			// pos_pub.publish(ip_pos);
			// sendangle(11,hip[2]);
			// pos_pub.publish(ip_pos);
			// sendangle(13,knee[0]);
			// pos_pub.publish(ip_pos);
			// sendangle(10,knee[3]);
			// pos_pub.publish(ip_pos);
			// sendangle(12,knee[1]);
			// pos_pub.publish(ip_pos);
			// sendangle(16,knee[2]);
			// hip[0] = JOINTCMD_Angle[0] + PI/2;
			// knee[0] = -JOINTCMD_Angle[1];                                                                    
			// hip[1] = -PI / 2 + JOINTCMD_Angle[2];
			// knee[1] = JOINTCMD_Angle[3];
			// hip[2] = PI / 2 + JOINTCMD_Angle[4];
			// knee[2] = -JOINTCMD_Angle[5];
			// hip[3] = -(PI / 2 + JOINTCMD_Angle[6]);
			// knee[3] = JOINTCMD_Angle[7];
            // sendangle(11,hip[0]);
            // pos_pub.publish(ip_pos);
            // sendangle(17,-hip[3]);
            // pos_pub.publish(ip_pos);
            // sendangle(15,hip[1]);
            // pos_pub.publish(ip_pos);
            // sendangle(14,-hip[2]);
            // pos_pub.publish(ip_pos);
            // sendangle(16,knee[0]);
            // pos_pub.publish(ip_pos);
            // sendangle(12,-knee[3]);
            // pos_pub.publish(ip_pos);
            // sendangle(10,knee[1]);
            // pos_pub.publish(ip_pos);
            // sendangle(13,-knee[2]);
            // pos_pub.publish(ip_pos);
            printf("%f %f %f %f %f %f %f %f\n", hip[0], knee[0], hip[1], knee[1], hip[2], knee[2], hip[3], knee[3]);
            loop_rate.sleep();
		}

	}
	return 0;
}



    