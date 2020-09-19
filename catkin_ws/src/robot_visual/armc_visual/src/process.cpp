#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cv.h>
#include <math.h>
#include "armc_visual/process.h"
#include "armc_visual/print_matrix.h"

using namespace cv;
using namespace std;

//相机内参
double u0 = 786.592, v0 = 582.186;
double fu = 882.461, fv = 882.337;
//
//double u0 = 1047.963, v0 = 422.504;
//double fu = 1504.141, fv = 1503.56;

int p_num = 0;
double Position[24][4];
double xc = 0, yc = 0, a = 0, b = 0, theta = 0;      //椭圆的5个参数
double A = 0, B = 0, C = 0, D = 0, E = 0, F = 0;     //椭圆方程的参数
double Q[3][3] = { 0 };                              //系数矩阵Q
double Target_Position[3] = { 0 };                   //目标的位置及法向量
double PosX, PosY, PosZ,L_PosNZx,L_PosNZy,L_PosNZz;  //正数状态下左目下的位置姿态变量
double Target_L_Att_tmp[3]={0};
double Pi = 3.1415926;
cv::Mat  Target_L_Pos, Target_L_Att, Target_L_Neg_Pos,Target_L_Neg_Att;

struct Ellipse
{
	float a;
	float x;
	float y;
	float width;
	float height;
	float angle;
}Ellipse[4];

void process(float center_x,float center_y,float box_width,float box_height,float box_angle, float weizi[6])
{
	int nCount = 0;
	int i=0;
		
		xc = center_x;
		yc = center_y;
		a = box_width;
		b = box_height;
		theta = box_angle;

		//-------------椭圆锥方程的五个系数及系数矩阵Q-----------------
		A = (a*a*sin(theta)*sin(theta) + b*b*cos(theta)*cos(theta))*fu*fu;
		B = (2 * (b*b - a*a)*sin(theta)*cos(theta))*fu*fv;
		C = (a*a*cos(theta)*cos(theta) + b*b*sin(theta)*sin(theta))*fv*fv;
		D = (-2 * (b*b*cos(theta)*cos(theta) + a*a*sin(theta)*sin(theta))*xc
			- 2 * (b*b - a*a)*sin(theta)*cos(theta)*yc)*fu
			+ 2 * (b*b*cos(theta)*cos(theta) + a*a*sin(theta)*sin(theta))*fu*u0
			+ (2 * (b*b - a*a)*sin(theta)*cos(theta))*fu*v0;
		E = (-2 * (a*a*cos(theta)*cos(theta) + b*b*sin(theta)*sin(theta))*yc
			- 2 * (b*b - a*a)*sin(theta)*cos(theta)*xc)*fv
			+ 2 * (a*a*cos(theta)*cos(theta) + b*b*sin(theta)*sin(theta))*fv*v0
			+ (2 * (b*b - a*a)*sin(theta)*cos(theta))*fv*u0;
		F = (b*b*cos(theta)*cos(theta) + a*a*sin(theta)*sin(theta))*xc*xc
			+ (a*a*cos(theta)*cos(theta) + b*b*sin(theta)*sin(theta))*yc*yc
			+ (b*b*cos(theta)*cos(theta) + a*a*sin(theta)*sin(theta))*u0*u0
			+ (a*a*cos(theta)*cos(theta) + b*b*sin(theta)*sin(theta))*v0*v0
			+ (2 * (b*b - a*a)*sin(theta)*cos(theta))*u0*v0
			+ (-2 * (b*b*cos(theta)*cos(theta) + a*a*sin(theta)*sin(theta))*xc
			- 2 * (b*b - a*a)*sin(theta)*cos(theta)*yc)*u0
			+ (-2 * (a*a*cos(theta)*cos(theta) + b*b*sin(theta)*sin(theta))*yc
			- 2 * (b*b - a*a)*sin(theta)*cos(theta)*xc)*v0
			+ 2 * (b*b - a*a)*sin(theta)*cos(theta)*xc*yc
			- a*a*b*b;
		Q[0][0] = A;        Q[0][1] = B / 2;    Q[0][2] = D / 2;
		Q[1][0] = B / 2;    Q[1][1] = C;        Q[1][2] = E / 2;
		Q[2][0] = D / 2;    Q[2][1] = E / 2;    Q[2][2] = F;

		//--------------------矩阵Q的特征值和特征向量---------------------
		double Lambda1, Lambda2, Lambda3; //特征值
		double L_e2[3][1] = { 0 }, L_e3[3][1] = { 0 };//特征向量e2 e3，数组形式
		Mat P, e1;//转换矩阵及向量e1	
		cv::Mat eValMatrixLQ;//矩阵Q的特征值
		cv::Mat eVecMatrixLQ;//矩阵Q的特征向量
		cv::Mat MatrixLQ = cv::Mat(3, 3, CV_64FC1, Q);//生成矩阵Q	
		cv::eigen(MatrixLQ, eValMatrixLQ, eVecMatrixLQ);//计算矩阵Q的特征值及特征向量
		
		Lambda3 = eValMatrixLQ.at<double>(2, 0); Lambda2 = eValMatrixLQ.at<double>(1, 0); Lambda1 = eValMatrixLQ.at<double>(0, 0);
		L_e3[0][0] = eVecMatrixLQ.at<double>(2, 0); L_e3[1][0] = eVecMatrixLQ.at<double>(2, 1); L_e3[2][0] = eVecMatrixLQ.at<double>(2, 2);
		L_e2[0][0] = eVecMatrixLQ.at<double>(1, 0); L_e2[1][0] = eVecMatrixLQ.at<double>(1, 1); L_e2[2][0] = eVecMatrixLQ.at<double>(1, 2);
	
		cv::Mat e2 = cv::Mat(3, 1, CV_64FC1, L_e2);//创建e2向量
		cv::Mat e3 = cv::Mat(3, 1, CV_64FC1, L_e3);//创建e3向量
		e1 = e2.cross(e3);//向量e2叉乘向量e3得出向量e1
		Lambda1 = abs(Lambda1); Lambda2 = abs(Lambda2); Lambda3 = abs(Lambda3);//将特征值绝对值化
		hconcat(e1, e2, P); hconcat(P, e3, P);//由e1 e2 e3生成转换矩阵P	
	
		//-------------------位置姿态计算-----------------------------------------
		Target_Position[0] = sqrt(Lambda3*(Lambda1 - Lambda2) / Lambda1 / (Lambda1 + Lambda3));//位置计算
		Target_Position[1] = 0;
		Target_Position[2] = sqrt(Lambda1*(Lambda2 + Lambda3) / Lambda3 / (Lambda1 + Lambda3));
		Mat Target_Position_tmp = Mat(3, 1, CV_64FC1, Target_Position);//目标位置	
		Target_Position_tmp = P*Target_Position_tmp;//左乘转换矩阵P生成实际的相机坐标系下的三维坐标	

		double Radius = 15;
		PosX = Radius*Target_Position[0];
		PosY = Radius*Target_Position[1];
		PosZ = Radius*Target_Position[2];
		cout << i+1 << "   " << PosX << "   " << PosY << "   " << PosZ << endl;

		Target_L_Att_tmp[0] = sqrt((Lambda1-Lambda2)/(Lambda1+Lambda3));//正数状态下的姿态计算
		Target_L_Att_tmp[1] = 0;
		Target_L_Att_tmp[2] = -sqrt((Lambda2+Lambda3)/(Lambda1+Lambda3));
		Target_L_Att = cv::Mat(3, 1, CV_64FC1, Target_L_Att_tmp);//定义左目下目标法向量的3X1向量:Target_L_Att
		Target_L_Att = P*Target_L_Att;//通过左乘转换矩阵L_P生成实际的相机坐标系下的法向量
		//cout<<"正数状态下目标在左目相机下的法向量为："<<endl<<Target_L_Att<<endl;
		double ang1 = Target_L_Att.ptr<double>(0)[0];
		double ang2 = Target_L_Att.ptr<double>(0)[1];
		double ang3 = Target_L_Att.ptr<double>(0)[2];
		weizi[0]= PosX;
		weizi[1] = PosY;
		weizi[2] = PosZ;
		weizi[3] = ang1;
		weizi[4] = ang2;
		weizi[5] = ang3;
}
