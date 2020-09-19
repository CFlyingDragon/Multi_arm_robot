#include <opencv2/core/utility.hpp>
#include <iostream>
#include <string>
#include <stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include<math.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <time.h>
#include<algorithm>
using namespace std;
using namespace cv;
#pragma once  
struct FastLenOrder
{	//���������һ�ֽṹ
	int order;
	double len;
};
struct FastlabelAngle
{    //����ǰ	���ڱ���ͬһ��label�е� ֱ�߳��ȺͽǶȵĽṹ
	vector <double> len;
	vector <double> angle;

};
#define  Maxlabels 2048 // �������ݳ���ߴ�������
class IprovedFDCM
	{
	public:
		IprovedFDCM(void);
		~IprovedFDCM(void);
	public:
		vector<FastLenOrder>Qorder;		
		vector<FastLenOrder>order;
		int maskSize0 = DIST_MASK_5;
		int voronoiType = 0;
		int distType0 = DIST_L2;
		double Gloable_th = 1000;		 
		double lamda = 2;  //���Ǳ任��lamdaֵ
		Scalar colors[9];
	private:	
		double lineAngle(Vec4f line1, Vec4f line2);
		double lineslope(Vec4f line);
		double lineslope(double x1, double y1, double x2, double y2);
		void lineMatch(Mat &dist, Mat &labelimg, double * LabelSlope, int maxlabel, Vec4f matchline, Vec4f curQline, double * result, vector<FastLenOrder>Qorder, vector<Vec4f> Qlines);
		void showMatch(Mat distmap, Vec4f matchline, Vec4f curQline, int minD, vector<Vec4f> Qlines, vector<Vec4f> lines);

	public:
		double Pointdistance(double  x1, double y1, double x2, double y2);
		 
		void lineSort(vector<Vec4f> line, vector<FastLenOrder>&order);
		void lineSortCut(vector<Vec4f> &line, vector<FastLenOrder>&order, float lenth);
		double  ondistTrans(Mat src, vector<FastLenOrder>Qorder, vector<Vec4f> Qlines, vector<FastLenOrder>order, vector<Vec4f> lines,bool shows, double* rst);
		void  Match(Vec4f matchline, Vec4f curQline, int minD, double*rst);	
};