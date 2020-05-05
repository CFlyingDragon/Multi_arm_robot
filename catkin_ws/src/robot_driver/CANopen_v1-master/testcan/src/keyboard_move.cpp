// Integration2SY.cpp: 定义控制台应用程序的入口点。
//


/*
* File:Integration1.c
* Date:20180621
* Description:采用模式控制，集成运动控制模式，通过键盘进行分类；
* Author：Edison_K
* Modifications:
*/
/*
* You may need to add include files like <webots/distance_sensor.h> or
* <webots/differential_wheels.h>, etc.
*/
// #include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "ros/ros.h"
#include "testcan/IpPos.h"
#include "iostream"
#include "geometry_msgs/Twist.h"

/*
* You may want to add macros here.
*/
#define TIME_STEP 8
#define aerfa 10000
#define beita 10000
#define miu 0.16
#define omigast 0.7
#define omigasw 4.2
#define omigasttrot 2
#define omigaswtrot 4
#define Azhi 100                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
#define N 50001 
#define PI 3.1416
#define INI 0.4
#define Iniangle 0.25

#define postoangle 1440000.0/360 
#define big 2498560.0/360
#define degreetoradius 180.0/3.1415926
#define radiustodegree 3.1415926/180.0

using namespace std;

testcan::IpPos ip_pos;
char KEY = '4';
int sendangle(int id, double angle){
    ip_pos.id = id;
    ip_pos.pos = angle*big*degreetoradius;
    return 0;
}
void keyboardCallback(const geometry_msgs::Twist &vel)
{
    // KEY = '4';
    // if(vel.linear.x>0&&vel.linear.x<0.5)
    //     KEY = '1';
    if(vel.linear.x>0.5)
        KEY = '2';
    else if(vel.linear.x<0.5&&vel.linear.x>0)
        KEY = '1';
    if(vel.angular.z>0)
        KEY = '3';
    if(vel.angular.z<0)
        KEY = '5';
    if(vel.linear.x==0&&vel.angular.z==0)
        KEY = '4';
    std::cout<<"KEY is = "<<KEY<<endl;
}
int main(int argc, char **argv)
{

	printf("hello\n");	
    ros::init(argc, argv, "keyboard_move_node");
    ros::NodeHandle nh_;
	printf("start");
    ros::Publisher pos_pub = nh_.advertise<testcan::IpPos>("ip_pos",1000);
    ros::Subscriber keyboard_cmd_sub = nh_.subscribe("leg_cmd_vel",1,keyboardCallback);
	int i;
	char KEYBO, KEYBO2;
	KEYBO = '4'; KEYBO2 = '4';
	void rkt2ftrot(double t, double y[], int n, double d[]);//函数声明，微分方程组
	void rkt2fwalk(double t, double y[], int n, double d[]);
	void rkt2fstop(double t, double y[], int n, double d[]);
	bool check_down(int keynum);
	void rkt2(double t, double h, double y[], int n, double eps, int KEYBO2);//函数声明，求解过程
	double t, h, eps, y[8], hip[4], knee[4], leg[4];
	double P, Q;
	P = 1; Q = 0.1;

	y[0] = 0.295; y[1] = 0.3205;
	y[2] = -0.240; y[3] = 0.183;
	y[4] = 0.003; y[5] = 0.409;
	y[6] = 0.434; y[7] = -0.030;//intinal number

	t = 0.0; h = 0.001; eps = 0.0001;//start point and step
	FILE *fp1;
	fp1 = fopen("angle.txt", "w");
    ros::Rate loop_rate(100);
	for (i = 1; i <= N - 1; i++)//循环执行N-1步
	{
		//KEYBO = wb_keyboard_get_key(); 加入键盘键入参数进行识别
		//检测识别的数字控制机器人的运动
		//爬行步态：1；对角步态：2；顺时针转弯：3；停止回到零位：5；逆时针转弯：5；
        // std::cin>>KEYBO;
        // KEYBO = getchar();
        KEYBO = KEY;
		if (check_down(KEYBO))
		{
			KEYBO2 = KEYBO;
			printf("Input a number %d\n", KEYBO - 48);
		}
	
		for (int q = 10; q>0; q--)
		{
			rkt2(t, h, y, 8, eps, KEYBO2);
			t = t + h;
			if (KEYBO2 == '3' || KEYBO2 == '5')
			{
				P = 1.7576;
				Q = 0.5;
				for (int m = 0; m<4; m++)
				{
					leg[m] = -y[2 * m] / 8;
				}
				if (y[3] >= 0)//腿2，支撑相,y[2],y[3],leg[1]
				{
					hip[1] = Iniangle * PI;
					knee[1] = -Iniangle * PI;
				}
				else
				{
					hip[1] = -Q / INI * y[3] + Iniangle * PI;
					knee[1] = P * Q / INI * y[3] - Iniangle * PI;
				}
				if (y[5] >= 0)//腿3,y[4],y[5],leg[2]
				{
					hip[2] = +Iniangle * PI;
					knee[2] = -Iniangle * PI;
				}
				else
				{
					hip[2] = -Q / INI * y[5] + Iniangle * PI;
					knee[2] = P * Q / INI * y[5] - Iniangle * PI;
				}
				if (y[7] >= 0)//腿4,y[6],y[7],leg[3]
				{
					hip[3] = -Iniangle * PI;
					knee[3] = Iniangle * PI;
				}
				else
				{
					hip[3] = Q / INI * y[7] - Iniangle * PI;
					knee[3] = -P * Q / INI * y[7] + Iniangle * PI;
				}
				if (y[1] >= 0)//腿1 支撑相，保持关节不变,y[0],y[1],leg[0]
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
			else
			{
				P = 0; Q = 0.1;
				if (y[1] >= 0)//腿1 支撑相，保持关节不变
				{
					hip[0] = -Iniangle * PI - Q * y[1];
					knee[0] = -y[0] + Iniangle * PI;
				}
				else //摆动相，
				{
					hip[0] = Q / INI * y[1] - Iniangle * PI;
					knee[0] = -y[0] + Iniangle * PI + P * y[1];
				}

				if (y[3] >= 0)//腿2
				{
					hip[1] = Iniangle * PI + Q * y[1];
					knee[1] = y[2] - Iniangle * PI;
				}
				else
				{
					hip[1] = -Q / INI * y[3] + Iniangle * PI;
					knee[1] = y[2] - Iniangle * PI + P * y[3];
				}

				if (y[5] >= 0)//腿3
				{
					hip[2] = +Iniangle * PI + Q * y[1];
					knee[2] = -y[4] - Iniangle * PI;
				}
				else
				{
					hip[2] = -Q / INI * y[5] + Iniangle * PI;
					knee[2] = -y[4] - Iniangle * PI + P * y[5];
				}

				if (y[7] >= 0)//腿4
				{
					hip[3] = -Iniangle * PI - Q * y[1];
					knee[3] = y[6] + Iniangle * PI;
				}
				else
				{
					hip[3] = Q / INI * y[7] - Iniangle * PI;
					knee[3] = y[6] + Iniangle * PI + P * y[7];
				}

			}

		}
		//运动顺序 1-4-2-3

		if (KEYBO2 == '3' || KEYBO2 == '5')
		{
			int DIR;
			if (KEYBO2 == '3')
			{
				DIR = 1;
			}
			else
			{
				DIR = -1;
			}
			leg[0] = DIR * leg[0];
			leg[1] = DIR * leg[1];
			leg[2] = DIR * leg[2];
			leg[3] = DIR * leg[3];

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
        // printf("pos = %f",hip[3]);
        ros::spinOnce();
		loop_rate.sleep();
		fprintf(fp1, "%f %f %f %f %f %f %f %f %f %f %f %f %f\n", t, leg[0], leg[1],leg[2],leg[3], y[0], y[1], y[2], y[3], y[4], y[5], y[6], y[7]);
			}

	fclose(fp1);
}

void rkt2fwalk(double t, double y[], int n, double d[])//walk步态
{
	double omiga1, omiga2, omiga3, omiga4;

	omiga1 = omigast / (exp(-Azhi * y[1]) + 1) + omigasw / (exp(Azhi*y[1]) + 1);
	d[0] = aerfa * (miu - y[0] * y[0] - y[1] * y[1])*y[0] - omiga1 * y[1];
	d[1] = beita * (miu - y[0] * y[0] - y[1] * y[1])*y[1] + omiga1 * y[0] - y[3] + y[5] - y[7];

	omiga2 = omigast / (exp(-Azhi * y[3]) + 1) + omigasw / (exp(Azhi*y[3]) + 1);
	d[2] = aerfa * (miu - y[2] * y[2] - y[3] * y[3])*y[2] - omiga2 * y[3];
	d[3] = beita * (miu - y[2] * y[2] - y[3] * y[3])*y[3] + omiga2 * y[2] - y[1] - y[5] + y[7];

	omiga3 = omigast / (exp(-Azhi * y[5]) + 1) + omigasw / (exp(Azhi*y[5]) + 1);
	d[4] = aerfa * (miu - y[4] * y[4] - y[5] * y[5])*y[4] - omiga3 * y[5];
	d[5] = beita * (miu - y[4] * y[4] - y[5] * y[5])*y[5] + omiga3 * y[4] - y[1] + y[3] - y[7];

	omiga4 = omigast / (exp(-Azhi * y[7]) + 1) + omigasw / (exp(Azhi*y[7]) + 1);
	d[6] = aerfa * (miu - y[6] * y[6] - y[7] * y[7])*y[6] - omiga4 * y[7];
	d[7] = beita * (miu - y[6] * y[6] - y[7] * y[7])*y[7] + omiga4 * y[6] + y[1] - y[3] - y[5];

	return;
}

void rkt2ftrot(double t, double y[], int n, double d[])//对角步态
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

void rkt2fstop(double t, double y[], int n, double d[])
{
	//printf("stop start");
	double omiga1, omiga2, omiga3, omiga4;

	omiga1 = omigast / (exp(-Azhi * y[1]) + 1) + omigasw / (exp(Azhi*y[1]) + 1);
	d[0] = aerfa * (miu - y[0] * y[0] - y[1] * y[1])*y[0] - omiga1 * y[1];
	d[1] = beita * (miu - y[0] * y[0] - y[1] * y[1])*y[1];

	omiga2 = omigast / (exp(-Azhi * y[3]) + 1) + omigasw / (exp(Azhi*y[3]) + 1);
	d[2] = aerfa * (miu - y[2] * y[2] - y[3] * y[3])*y[2] - omiga2 * y[3];
	d[3] = beita * (miu - y[2] * y[2] - y[3] * y[3])*y[3];

	omiga3 = omigast / (exp(-Azhi * y[5]) + 1) + omigasw / (exp(Azhi*y[5]) + 1);
	d[4] = aerfa * (miu - y[4] * y[4] - y[5] * y[5])*y[4] - omiga3 * y[5];
	d[5] = beita * (miu - y[4] * y[4] - y[5] * y[5])*y[5];

	omiga4 = omigast / (exp(-Azhi * y[7]) + 1) + omigasw / (exp(Azhi*y[7]) + 1);
	d[6] = aerfa * (miu - y[6] * y[6] - y[7] * y[7])*y[6] - omiga4 * y[7];
	d[7] = beita * (miu - y[6] * y[6] - y[7] * y[7])*y[7];

	return;
}

void rkt2(double t, double h, double y[], int n, double eps, int KEYBO2)
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
			if (KEYBO2 == '1')
			{
				rkt2fwalk(t, y, n, d);
			}
			else if (KEYBO2 == '2' || KEYBO2 == '3' || KEYBO2 == '5')
			{
				rkt2ftrot(t, y, n, d);
			}
			else if (KEYBO2 == '4')
			{
				rkt2fstop(t, y, n, d);
			}
			else
			{
				printf("2->Not a number\n");
			}

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
				if (KEYBO2 == '1')
				{
					rkt2fwalk(tt, y, n, d);
				}
				else if (KEYBO2 == '2' || KEYBO2 == '3' || KEYBO2 == '5')
				{
					rkt2ftrot(tt, y, n, d);
				}
				else if (KEYBO2 == '4')
				{
					rkt2fstop(t, y, n, d);
				}
				else
				{
					printf("1->Not a number\n");
				}

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

bool check_down(int KEYBO) {
	bool down = false;
	if (KEYBO == -1)
	{
		down = false;
	}
	else if (KEYBO == '1' || KEYBO == '2' || KEYBO == '3' || KEYBO == '4' || KEYBO == '5')
	{
		down = true;
	}

	if (down)
	{
		return true;
	}
	else
	{
		return false;
	}
}