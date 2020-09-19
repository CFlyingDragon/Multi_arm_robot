#ifndef Rect_Detect_H
#define Rect_Detect_H

#include <ctime>
#include <stdio.h>
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>  
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>  
#include "opencv2/highgui.hpp"
#include "fstream"
#include <vector>
#include<math.h>
#include <unistd.h>

#include "IprovedFDCM.h"
using namespace std;
using namespace cv;

#pragma once  


double  lineslope(Vec4f line);

double Linedistance(Vec4f line);
double Pointdistance(float  x1, float  y1, float x2, float y2);

Point2f getCrossPoint(Vec4f LineA, Vec4f LineB);

/*绘制可能组成矩形的4条直线*/
void DrawRect(Vec4f L1,Vec4f L2,Vec4f L3,Vec4f L4);


/*实现排列组合:进行选择*/
void func_select(int n,int k,int a[],int m);

// 对直线进行聚合，主要聚合：共线、且直线上一点位于另一直线上的两条直线a
vector<Vec4f> line_cluster(vector<Vec4f> DetLines);
bool rect_ansys(Vec4f line1,Vec4f line2,Vec4f line3,Vec4f line4);

// int RectDetect(Mat Rect_ROI, Point2f* RectPoint);
//int RectDetect(Mat Rect_ROI);
//int RectDetect(Mat ROI_PIC,vector<Point2f> corner);
//int RectDetect(Mat ROI_PIC,vector<Point2f> corner,bool isDetect);
// vector<Point2f> RectDetect(Mat ROI_PIC,bool isDetect);
int RectDetect(Mat ROI_PIC,bool* isDetect,vector<Point2f> fourcorner);


#endif
