// trotGen.c.cpp: 定义控制台应用程序的入口点。
//

// #include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "ros/ros.h"
#include "testcan/IpPos.h"


#define aerfa 10000
#define beita 10000
#define miu 0.16
#define omigasttrot 2
#define omigaswtrot 4
#define Azhi 100                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
#define N 10001 
#define PI 3.1416
#define INI 0.4
#define P 1.7576 
#define Iniangle 0.25
#define Q 0.5 
#define postoangle 1440000.0/360 
#define big 2498560.0/360
#define degreetoradius 180.0/3.1415926
#define radiustodegree 3.1415926/180.0

testcan::IpPos ip_pos;
int sendangle(int id, double angle){
    ip_pos.id = id;
    ip_pos.pos = angle*big*degreetoradius;
    return 0;
}

int main(int argc, char *argv[])
{	
	printf("hello\n");
    ros::init(argc, argv, "tort_gen_node");
    ros::NodeHandle nh_;
	printf("start");
    ros::Publisher pos_pub = nh_.advertise<testcan::IpPos>("ip_pos",1000);
	usleep(1000000);
	printf("start\n");
	/*sendangle(22, 0);
    pos_pub.publish(ip_pos);
	// usleep(100000);
	sendangle(21, 0);
    pos_pub.publish(ip_pos);
	// usleep(100000);
	sendangle(17, 0);
    pos_pub.publish(ip_pos);
	// usleep(100000);
	sendangle(12, 0);
    pos_pub.publish(ip_pos);
	// usleep(100000);
	sendangle(25, 0);
    pos_pub.publish(ip_pos);
	// usleep(1000000);
	sendangle(23, 0);
    pos_pub.publish(ip_pos);
	// usleep(1000000);
	sendangle(15, 0);
    pos_pub.publish(ip_pos);
	// usleep(1000000);
	sendangle(10, 0);
    pos_pub.publish(ip_pos);
	// usleep(1000000);
	sendangle(26, 0);
    pos_pub.publish(ip_pos);
	// usleep(1000000);
	sendangle(27, 0);
    pos_pub.publish(ip_pos);
	// usleep(1000000);
	sendangle(14, 0);
    pos_pub.publish(ip_pos);
	// usleep(1000000);
	sendangle(13, 0);
    pos_pub.publish(ip_pos);
	// usleep(1000000);
	sendangle(20, 0);
    pos_pub.publish(ip_pos);
	// usleep(1000000);
	sendangle(24, 0);
    pos_pub.publish(ip_pos);
	// usleep(1000000);
	sendangle(11, 0);
    pos_pub.publish(ip_pos);
	// usleep(1000000);
	sendangle(16, 0);
    pos_pub.publish(ip_pos);
	// usleep(1000000);*/
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
	usleep(5000000);
	sendangle(17, 86.45*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(12, -158.4*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(15, -86.45*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(10, 158.4*radiustodegree);
    pos_pub.publish(ip_pos);

	sendangle(14, -86.45*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(13, 155*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(11, 86.45*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(16, -158.4*radiustodegree);
    pos_pub.publish(ip_pos);
	usleep(5000000);

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
	usleep(10000000);
	int i;
	void rkt2f(double t, double y[], int n, double d[]);//函数声明，微分方程组
	void rkt2(double t, double h, double y[], int n, double eps);//函数声明，求解过程
	double t, h, eps, y[8], hip[4], knee[4];

	y[0] = 0.295; y[1] = 0.3205;
	y[2] = -0.240; y[3] = 0.183;
	y[4] = 0.003; y[5] = 0.409;
	y[6] = 0.434; y[7] = -0.030;
	t = 0.0; h = 0.001; eps = 0.0001;//start point and step
	FILE *fp1;
	fp1 = fopen("catkin_ws/src/testcan/log/position_sensor.txt", "w");
    ros::Rate loop_rate(100);
	for (i = 1; i <= N - 1; i++)//循环执行N-1步
	{
		printf("the number is %d\n",i);
		for (int q = 10; q > 0; q--)
		{
			rkt2(t, h, y, 8, eps);
			t = t + h;

			if (y[3] >= 0)//腿2
			{
				hip[1] = Iniangle * PI;
				knee[1] = -Iniangle * PI;
			}
			else
			{
				hip[1] = -Q / INI * y[3] + Iniangle * PI;
				knee[1] = P * Q / INI * y[3] - Iniangle * PI;
			}

			if (y[5] >= 0)//腿3
			{
				hip[2] = +Iniangle * PI;
				knee[2] = -Iniangle * PI;
			}
			else
			{
				hip[2] = -Q / INI * y[5] + Iniangle * PI;
				knee[2] = P * Q / INI * y[5] - Iniangle * PI;
			}

			if (y[7] >= 0)//腿4
			{
				hip[3] = -Iniangle * PI;
				knee[3] = Iniangle * PI;
			}
			else
			{
				hip[3] = Q / INI * y[7] - Iniangle * PI;
				knee[3] = -P * Q / INI * y[7] + Iniangle * PI;
			}

			if (y[1] >= 0)//腿1 支撑相，保持关节不变
			{
				hip[0] = -Iniangle * PI;
				knee[0] = Iniangle * PI;
			}
			else //摆动相，
			{
				hip[0] = Q / INI * y[1] - Iniangle * PI;
				knee[0] = -P * Q / INI * y[1] + Iniangle * PI;
			}
		}
        sendangle(11,-hip[0]);
        pos_pub.publish(ip_pos);
		sendangle(17,-hip[3]);
        pos_pub.publish(ip_pos);
		sendangle(15,-hip[1]);
        pos_pub.publish(ip_pos);
		sendangle(14,-hip[2]);
        pos_pub.publish(ip_pos);
        sendangle(16,-knee[0]);
        pos_pub.publish(ip_pos);
        sendangle(12,-knee[3]);
        pos_pub.publish(ip_pos);
		sendangle(10,-knee[1]);
        pos_pub.publish(ip_pos);
        sendangle(13,-knee[2]);
        pos_pub.publish(ip_pos);
        printf("pos = %f",hip[3]);
		fprintf(fp1, "%f %f %f %f %f %f %f %f %f\n", t, hip[0], knee[0], hip[1], knee[1], hip[2], knee[2], hip[3], knee[3]);
        // pos_pub.publish(ip_pos);
        loop_rate.sleep();
	}
    return 0;
}
void rkt2f(double t, double y[], int n, double d[])//对角步态
{
	double omiga1, omiga2, omiga3, omiga4;

	omiga1 = omigasttrot / (exp(-Azhi * y[1]) + 1) + omigaswtrot / (exp(Azhi*y[1]) + 1);
	d[0] = aerfa * (miu - y[0] * y[0] - y[1] * y[1])*y[0] - omiga1 * y[1];
	d[1] = beita * (miu - y[0] * y[0] - y[1] * y[1])*y[1] + omiga1 * y[0] - y[3] - y[5] + y[7];

	omiga2 = omigasttrot / (exp(-Azhi * y[3]) + 1) + omigaswtrot / (exp(Azhi*y[3]) + 1);
	d[2] = aerfa * (miu - y[2] * y[2] - y[3] * y[3])*y[2] - omiga2 * y[3];
	d[3] = beita * (miu - y[2] * y[2] - y[3] * y[3])*y[3] + omiga2 * y[2] - y[1] + y[5] - y[7];

	omiga3 = omigasttrot / (exp(-Azhi * y[5]) + 1) + omigaswtrot / (exp(Azhi*y[5]) + 1);
	d[4] = aerfa * (miu - y[4] * y[4] - y[5] * y[5])*y[4] - omiga3 * y[5];
	d[5] = beita * (miu - y[4] * y[4] - y[5] * y[5])*y[5] + omiga3 * y[4] - y[1] + y[3] - y[7];

	omiga4 = omigasttrot / (exp(-Azhi * y[7]) + 1) + omigaswtrot / (exp(Azhi*y[7]) + 1);
	d[6] = aerfa * (miu - y[6] * y[6] - y[7] * y[7])*y[6] - omiga4 * y[7];
	d[7] = beita * (miu - y[6] * y[6] - y[7] * y[7])*y[7] + omiga4 * y[6] + y[1] - y[3] - y[5];

	return;
}
void rkt2(double t, double h, double y[], int n, double eps)
{
	int m, i, j, k;
	double hh, p, dt, x, tt, q, a[4], *g, *b, *c, *d, *e;
	g = (double *)malloc(n * sizeof(double));
	b = (double *)malloc(n * sizeof(double));
	d = (double *)malloc(n * sizeof(double));
	c = (double *)malloc(n * sizeof(double));
	e = (double *)malloc(n * sizeof(double));
	hh = h; m = 1; p = 1.0 + eps; x = t;
	for (i = 0; i <= n - 1; i++) c[i] = y[i];
	while (p >= eps)
	{
		a[0] = hh / 2.0; a[1] = a[0];
		a[2] = hh; a[3] = hh;
		for (i = 0; i <= n - 1; i++)
		{
			g[i] = y[i];
			y[i] = c[i];
		}
		dt = h / m; t = x;
		for (j = 0; j <= m - 1; j++)
		{

			rkt2f(t, y, n, d);

			for (i = 0; i <= n - 1; i++)
			{
				b[i] = y[i];
				e[i] = y[i];
			}
			for (k = 0; k <= 2; k++)
			{
				for (i = 0; i <= n - 1; i++)
				{
					y[i] = e[i] + a[k] * d[i];
					b[i] = b[i] + a[k + 1] * d[i] / 3.0;
				}
				tt = t + a[k];
				rkt2f(tt, y, n, d);
			}
			for (i = 0; i <= n - 1; i++)
				y[i] = b[i] + hh * d[i] / 6.0;
			t = t + dt;
		}
		p = 0.0;
		for (i = 0; i <= n - 1; i++)
		{
			q = fabs(y[i] - g[i]);
			if (q > p) p = q;
		}
		hh = hh / 2.0; m = m + m;
	}
	free(g); free(b); free(c); free(d); free(e);
	return;
}