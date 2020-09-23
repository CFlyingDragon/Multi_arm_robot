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
double u0 = 1230.23, v0 = 1021.54;
double fu = 3621.84, fv = 3620.63;
//
//double u0 = 1047.963, v0 = 422.504;
//double fu = 1504.141, fv = 1503.56;

int p_num = 0;
double Position[24][4];
double xc = 0, yc = 0, a = 0, b = 0, theta = 0;      //???5???
double A = 0, B = 0, C = 0, D = 0, E = 0, F = 0;     //???????
double Q[3][3] = { 0 };                              //????Q
double Target_Position_z[3] = { 0 };                   //?????????
double Target_Position_f[3] = { 0 };                   //?????????
double PosX_z, PosY_z, PosZ_z,PosX_f, PosY_f, PosZ_f,L_PosNZx,L_PosNZy,L_PosNZz;  //???????????????
double Target_L_Att_tmp_z[3]={0};
double Target_L_Att_tmp_f[3]={0};
double Pi = 3.1415926;
cv::Mat  Target_L_Pos, Target_L_Att_z, Target_L_Att_f,Target_L_Neg_Pos,Target_L_Neg_Att;

struct Ellipse
{
	float a;
	float x;
	float y;
	float width;
	float height;
	float angle;
}Ellipse[4];

void process(float center_x,float center_y,float box_width,float box_height,float box_angle, float weizi_1[6],float weizi_2[6])
{
    int nCount = 0;
    int i=0;

        xc = center_x;
        yc = center_y;
        a = box_width;
        b = box_height;
        theta = box_angle;

        //-------------???????????????Q-----------------
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

        //--------------------??Q?????????---------------------
        double Lambda1, Lambda2, Lambda3; //???
        double L_e2[3][1] = { 0 }, L_e3[3][1] = { 0 };//????e2 e3?????
        Mat P, e1;//???????e1
        cv::Mat eValMatrixLQ;//??Q????
        cv::Mat eVecMatrixLQ;//??Q?????
        cv::Mat MatrixLQ = cv::Mat(3, 3, CV_64FC1, Q);//????Q
        cv::eigen(MatrixLQ, eValMatrixLQ, eVecMatrixLQ);//????Q?????????

        Lambda3 = eValMatrixLQ.at<double>(2, 0); Lambda2 = eValMatrixLQ.at<double>(1, 0); Lambda1 = eValMatrixLQ.at<double>(0, 0);
        L_e3[0][0] = eVecMatrixLQ.at<double>(2, 0); L_e3[1][0] = eVecMatrixLQ.at<double>(2, 1); L_e3[2][0] = eVecMatrixLQ.at<double>(2, 2);
        L_e2[0][0] = eVecMatrixLQ.at<double>(1, 0); L_e2[1][0] = eVecMatrixLQ.at<double>(1, 1); L_e2[2][0] = eVecMatrixLQ.at<double>(1, 2);

        cv::Mat e2 = cv::Mat(3, 1, CV_64FC1, L_e2);//??e2??
        cv::Mat e3 = cv::Mat(3, 1, CV_64FC1, L_e3);//??e3??
        e1 = e2.cross(e3);//??e2????e3????e1
        Lambda1 = abs(Lambda1); Lambda2 = abs(Lambda2); Lambda3 = abs(Lambda3);//????????
        hconcat(e1, e2, P); hconcat(P, e3, P);//?e1 e2 e3??????P

        //-------------------??????????-----------------------------------------
        Target_Position_z[0] = sqrt(Lambda3*(Lambda1 - Lambda2) / Lambda1 / (Lambda1 + Lambda3));//????
        Target_Position_z[1] = 0;
        Target_Position_z[2] = sqrt(Lambda1*(Lambda2 + Lambda3) / Lambda3 / (Lambda1 + Lambda3));
        Mat Target_Position_tmp_z = Mat(3, 1, CV_64FC1, Target_Position_z);//????
        Target_Position_tmp_z = P*Target_Position_tmp_z;//??????P????????????????

        double Radius = 7.5;
        PosX_z = Radius*Target_Position_z[0];
        PosY_z = Radius*Target_Position_z[1];
        PosZ_z = Radius*Target_Position_z[2];
        cout << i+1 << "   " << PosX_z << "   " << PosY_z << "   " << PosZ_z << endl;

        Target_L_Att_tmp_z[0] = sqrt((Lambda1-Lambda2)/(Lambda1+Lambda3));//??????????
        Target_L_Att_tmp_z[1] = 0;
        Target_L_Att_tmp_z[2] = -sqrt((Lambda2+Lambda3)/(Lambda1+Lambda3));
        Target_L_Att_z = cv::Mat(3, 1, CV_64FC1, Target_L_Att_tmp_z);//???????????3X1??:Target_L_Att
        Target_L_Att_z = P*Target_L_Att_z;//????????L_P???????????????
        //cout<<"???????????????????"<<endl<<Target_L_Att<<endl;
        double ang1_z = Target_L_Att_z.ptr<double>(0)[0];
        double ang2_z = Target_L_Att_z.ptr<double>(0)[1];
        double ang3_z = Target_L_Att_z.ptr<double>(0)[2];
        weizi_1[0]= PosX_z;
        weizi_1[1] = PosY_z;
        weizi_1[2] = PosZ_z;
        weizi_1[3] = ang1_z;
        weizi_1[4] = ang1_z;
        weizi_1[5] = ang1_z;

        //-------------------??????????-----------------------------------------
        Target_Position_f[0] = -sqrt(Lambda3*(Lambda1 - Lambda2) / Lambda1 / (Lambda1 + Lambda3));//????
        Target_Position_f[1] = 0;
        Target_Position_f[2] = sqrt(Lambda1*(Lambda2 + Lambda3) / Lambda3 / (Lambda1 + Lambda3));
        Mat Target_Position_tmp_f = Mat(3, 1, CV_64FC1, Target_Position_f);//????
        Target_Position_tmp_f = P*Target_Position_tmp_f;//??????P????????????????

        PosX_f = Radius*Target_Position_f[0];
        PosY_f = Radius*Target_Position_f[1];
        PosZ_f = Radius*Target_Position_f[2];
        cout << i+1 << "   " << PosX_f << "   " << PosY_f << "   " << PosZ_f << endl;

        Target_L_Att_tmp_f[0] = -sqrt((Lambda1-Lambda2)/(Lambda1+Lambda3));//??????????
        Target_L_Att_tmp_f[1] = 0;
        Target_L_Att_tmp_f[2] = -sqrt((Lambda2+Lambda3)/(Lambda1+Lambda3));
        Target_L_Att_f = cv::Mat(3, 1, CV_64FC1, Target_L_Att_tmp_f);//???????????3X1??:Target_L_Att
        Target_L_Att_f = P*Target_L_Att_f;//????????L_P???????????????
        //cout<<"???????????????????"<<endl<<Target_L_Att<<endl;
        double ang1_f = Target_L_Att_f.ptr<double>(0)[0];
        double ang2_f = Target_L_Att_f.ptr<double>(0)[1];
        double ang3_f = Target_L_Att_f.ptr<double>(0)[2];
        weizi_2[0]= PosX_f;
        weizi_2[1] = PosY_f;
        weizi_2[2] = PosZ_f;
        weizi_2[3] = ang1_f;
        weizi_2[4] = ang1_f;
        weizi_2[5] = ang1_f;
}
