#include "armc_visual/IprovedFDCM.h"
IprovedFDCM::IprovedFDCM(void)
{
	colors [0] = Scalar(0, 0, 0);
	colors[1]  = Scalar(255, 0, 0);
	colors[2]  = Scalar(255, 128, 0);
	colors[3]  = Scalar(255, 255, 0);
	colors[4]  = Scalar(0, 255, 0);
	colors[5]  = Scalar(0, 128, 255);
	colors[6]  = Scalar(0, 255, 255);
	colors[7]  = Scalar(0, 0, 255);
	colors[8]  = Scalar(255, 0, 255);
	//vector<FastLenOrder> od;
	//vector<FastLenOrder> od2;
	//Qorder = od;
	// order=od;
	
}
IprovedFDCM::~IprovedFDCM(void)
{
}
double IprovedFDCM::Pointdistance(double  x1, double y1, double x2, double y2)
{ //������ 

	return		   sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}
double  IprovedFDCM::lineAngle(Vec4f line1, Vec4f line2)
{//���Ҷ��� //���angle��������С�ļнǣ����Ҫ0-180�ļн����޸ķ���ֵ�÷���
	double cosfi = 0, fi = 0, norm = 0;
	double dsx = line1[0] - line1[2];
	double dsy = line1[1] - line1[3];
	double dex = line2[0] - line2[2];
	double dey = line2[1] - line2[3];
	cosfi = dsx * dex + dsy * dey;
	norm = (dsx * dsx + dsy * dsy) * (dex * dex + dey * dey);
	cosfi /= sqrt(norm);

	if (cosfi >= 1.0) return 0;
	if (cosfi <= -1.0) return 0;
	fi = acos(cosfi);

	if (180 * fi / CV_PI < 90)
	{
		return 180 * fi / CV_PI;
	}
	else
	{
		return 180 - 180 * fi / CV_PI;
	}


}
double  IprovedFDCM::lineslope(Vec4f line)
{//б�� ���ػ���	
	double dsx = line[0] - line[2];
	double dsy = line[1] - line[3] + DBL_MIN;
	//cout << "dsx" << dsx << "dsy" << dsy << "atan2" << atan2(dsx, dsy) << endl;
	return atan(dsy / dsx); // 
}
double  IprovedFDCM::lineslope(double x1, double y1, double x2, double y2)
{//б�� ���ػ���	
	double dsx = x1 - x2;
	double dsy = y1 - y2 + DBL_MIN;
	return atan(dsy / dsx); //ע�⣬antan2�����᷵�ش����޵ķ���
}

/////////////�߳�����//////////////////
bool FastsortMethod(const FastLenOrder &v1, const FastLenOrder &v2)
{
	return v1.len > v2.len;
}
void IprovedFDCM::lineSort(vector<Vec4f> line, vector<FastLenOrder>&order)
{
	order.clear();
	for (int i = 0; i < line.size(); i++)
	{
		FastLenOrder lo;
		lo.len = Pointdistance(line.at(i)[0], line.at(i)[1], line.at(i)[2], line.at(i)[3]);
		lo.order = i;
		order.push_back(lo);
	}
	sort(order.begin(), order.end(), FastsortMethod);
}
void  IprovedFDCM::lineSortCut(vector<Vec4f> &line, vector<FastLenOrder>&order, float lenth)
{   //���ַ����Ὣ�̵��߶��޳���
	order.clear();
	if (lenth > 1)
	{
		int i = 0;
		vector<Vec4f> ::iterator it = line.begin();
		for (int j = 0; j<line.size(); j++)
		{
			FastLenOrder lo;
			lo.len = Pointdistance(line.at(j)[0], line.at(j)[1], line.at(j)[2], line.at(j)[3]);
			lo.order = i;
			if (lo.len > lenth)
			{
				i++;
				order.push_back(lo);
			}
			else
			{
				vector<Vec4f> ::iterator it = line.begin()+j;
				line.erase( it);
				j--;
			}

		}
	}
	else
	{
		for (int i = 0; i < line.size(); i++)
		{
			FastLenOrder lo;
			lo.len = Pointdistance(line.at(i)[0], line.at(i)[1], line.at(i)[2], line.at(i)[3]);
			lo.order = i;
			order.push_back(lo);
		}
	}
	sort(order.begin(), order.end(), FastsortMethod);
}
/////////////�߳�����////////////////// 

void  IprovedFDCM::lineMatch(Mat &dist, Mat &labelimg, double * LabelSlope, int maxlabel, Vec4f matchline, Vec4f curQline, double * result, vector<FastLenOrder>Qorder, vector<Vec4f> Qlines)
{   //���Ǿ���任ƥ�䣬����ƥ��ֵ
	//��������&dist, ����任ͼ
	//Mat &labelimg,  ����任��labelͼ
	//double * LabelSlope, б�ʱ���labelΪx�ĵ��б���� LabelSlope[x]
	//int maxlabel, //label�����ֵ
	//Vec4f matchline, //ԭͼ�б�ƥ���ֱ��
	//Vec4f curQline,  //ģ���б�ƥ���ֱ��
	//double * result, //ƥ��ֵ���������������ݣ���һ����reslut[0]������תalfa�ģ���һ������תalfa+pi��
	//vector<FastLenOrder>Qorder, // Qlines��������������ֱ�߳��������Qorder.at(i).len�Ǹ�ֱ�ߵĳ���Qorder.at(i).order�Ǹ�ֱ����Qlines���±�
	//vector<Vec4f> Qlines//ģ���ֱ��

	//�����߶η���ȷ�����ıߣ������� �Ƕ�alfa��180+alfa�������
	//	for (int i = 0; i <= maxlabel;i++)	{		//cout << LabelSlope[i] << endl;	}
	int row = dist.rows;
	int col = dist.cols;
	double cita = lineslope(curQline) - lineslope(matchline);
	double xm1 = (matchline[0] + matchline[2]) / 2;
	double ym1 = (matchline[1] + matchline[3]) / 2;
	double xm2 = (curQline[0] + curQline[2]) / 2;
	double ym2 = (curQline[1] + curQline[3]) / 2;
	double pt1x, pt2x, pt1y, pt2y;
	double angleErr = 0;
	//int Lb = 0;
	for (int dcount = 0; dcount < 2; dcount++)
	{
		cita = cita + CV_PI*dcount;
		double tx = xm1 - xm2*cos(cita) - ym2*sin(cita);
		double ty = ym1 - ym2*cos(cita) + xm2*sin(cita);
		for (int i = 0; i < Qorder.size(); i++)
		{
			pt1x = Qlines.at(i)[0] * cos(cita) + sin(cita)*Qlines.at(i)[1] + tx;
			pt1y = Qlines.at(i)[1] * cos(cita) - sin(cita)*Qlines.at(i)[0] + ty;
			pt2x = Qlines.at(i)[2] * cos(cita) + sin(cita)*Qlines.at(i)[3] + tx;
			pt2y = Qlines.at(i)[3] * cos(cita) - sin(cita)*Qlines.at(i)[2] + ty;
			double len = Qorder.at(i).len;
			double slope = lineslope(pt1x, pt1y, pt2x, pt2y);
			for (float k = 0; k <= len; k += 1)  //��1�����صľ����ڸ�ֱ���ϱ������������任ֵ
			{
				int x = pt1x + (pt2x - pt1x)*(float)k / len + 0.5;
				int y = pt1y + (pt2y - pt1y)*(float)k / len + 0.5;
				if (x >= 0 && x < col&&y >= 0 && y < row)
				{

					//Lb=labelimg.at<int>(y, x);
					//if (Lb > maxlabel)			lb = maxlabel;
					angleErr = fabs(LabelSlope[labelimg.at<int>(y, x)] - slope);
					if (angleErr > CV_PI / 2)
					{
						angleErr = CV_PI - angleErr;
					}

					result[dcount] += dist.at<float>(y, x) + lamda*angleErr; //chamfer distance trans ƥ�����Ҫ��ʽ
				}
				else
					result[dcount] += 100;//�����Ҫ���飬���ǲ����ñ�Ե��С  //ͼ����ĵ�
			}  //fork

		}	//for i		 
	}//for dcount

}
void  IprovedFDCM::showMatch(Mat distmap, Vec4f matchline, Vec4f curQline, int minD, vector<Vec4f> Qlines, vector<Vec4f> lines)
{
	double cita = lineslope(curQline) - lineslope(matchline);
	Point2f pt1, pt2;

	double xm1 = (matchline[0] + matchline[2]) / 2;
	double ym1 = (matchline[1] + matchline[3]) / 2;
	double xm2 = (curQline[0] + curQline[2]) / 2;
	double ym2 = (curQline[1] + curQline[3]) / 2;
	
	
	cita = cita + CV_PI*minD;
	double tx = xm1 - xm2*cos(cita) - ym2*sin(cita);
	double ty = ym1 - ym2*cos(cita) + xm2*sin(cita);
	
	for (int i = 0; i < Qlines.size(); i++)
	{
		pt1.x = Qlines.at(i)[0] * cos(cita) + sin(cita)*Qlines.at(i)[1] + tx;
		pt1.y = Qlines.at(i)[1] * cos(cita) - sin(cita)*Qlines.at(i)[0] + ty;
		pt2.x = Qlines.at(i)[2] * cos(cita) + sin(cita)*Qlines.at(i)[3] + tx;
		pt2.y = Qlines.at(i)[3] * cos(cita) - sin(cita)*Qlines.at(i)[2] + ty;
		line(distmap, pt1, pt2, CV_RGB(20, 250, 250), 8);
	}
	pt1.x = matchline[0];	pt1.y = matchline[1];	pt2.x = matchline[2];	pt2.y = matchline[3];
	line(distmap, pt1, pt2, CV_RGB(0, 0, 255), 5);
	for (int i = 0; i < lines.size(); i++)
	{
		pt1.x = lines.at(i)[0];
		pt1.y = lines.at(i)[1] ;
		pt2.x = lines.at(i)[2] ;
		pt2.y = lines.at(i)[3] ;
		line(distmap, pt1, pt2, CV_RGB(250, 0, 250), 2);
	}
	
}
void IprovedFDCM::Match(Vec4f matchline, Vec4f curQline, int minD, double* rst)
{
	double cita = lineslope(curQline) - lineslope(matchline);
	Point2f pt1, pt2;
	double xm1 = (matchline[0] + matchline[2]) / 2;
	double ym1 = (matchline[1] + matchline[3]) / 2;
	double xm2 = (curQline[0] + curQline[2]) / 2;
	double ym2 = (curQline[1] + curQline[3]) / 2;

	cita = cita + CV_PI*minD;
	double tx = xm1 - xm2*cos(cita) - ym2*sin(cita);
	double ty = ym1 - ym2*cos(cita) + xm2*sin(cita);
	rst[0] = 1;
	rst[1] = cita;	
	rst[2] = tx; 
	rst[3] = ty;
	
	//cout << "rst is " << cita << "," << tx << "," << ty << "," << endl;
	 
}
double IprovedFDCM::ondistTrans(Mat src, vector<FastLenOrder>Qorder, vector<Vec4f> Qlines, vector<FastLenOrder>order, vector<Vec4f> lines,bool shows, double* rst)
{   //����任
	rst[0] = 0;
	int rows = src.rows;
	int cols = src.cols;	 
	Mat edge = Mat::ones(rows, cols, CV_8UC1) * 255;
	vector <int>centerx, centery;
	vector<double> angles;
	for (int i = 0; i < order.size(); i++)
	{  //ÿ��ֱ�߶����̣��Է����� 
		//line(edge, Point(lines.at(i)[0], lines.at(i)[1]), Point(lines.at(i)[2], lines.at(i)[3]), Scalar(0));
		Vec4f L = lines.at(order.at(i).order);
		float len = order.at(i).len;
		float k1 = 2;
		float k2 = len - 2;
		float pt1x = L[0];			float pt1y = L[1];			float pt2x = L[2];			float pt2y = L[3];
		float x1 = pt1x + (pt2x - pt1x)*(float)k1 / len;
		float y1 = pt1y + (pt2y - pt1y)*(float)k1 / len;
		float x2 = pt1x + (pt2x - pt1x)*(float)k2 / len;
		float y2 = pt1y + (pt2y - pt1y)*(float)k2 / len;
		line(edge, Point(x1, y1), Point(x2, y2), Scalar(0));  //������Եͼ
		centerx.push_back(pt1x / 2 + pt2x / 2);
		centery.push_back(pt1y / 2 + pt2y / 2);
		angles.push_back(lineslope(pt1x, pt1y, pt2x, pt2y));//��¼ÿ��ֱ�ߵĳ��Ⱥ��е�
	}

	int step0 = edge.step;
	int step1 = edge.step[1];
	uchar *pedge = edge.data;
	int esize = edge.elemSize();
	int maskSize = maskSize0;
	int distType = distType0;
	Mat dist, labels, dist8u;
	distanceTransform(edge, dist, labels, distType, maskSize, voronoiType);//����任
	

	//imshow("labels", labels);
	//waitKey(40);

	//normalize(dist, dist, 255, 0, NORM_MINMAX);
	Mat distForMatch;
	dist.convertTo(distForMatch, CV_32S, 1, 0);
	int maxlabel = 0;
	///////////��һ�������ǽ�ÿ��label��Ӧ�ĽǶ����///////////////
	struct FastlabelAngle LA[Maxlabels];
	int lb;
	for (int i = 0; i < centerx.size(); i++)
	{
		lb = labels.at<int>(centery.at(i), centerx.at(i));

		if (lb>maxlabel)
			maxlabel = lb;
		if (lb> Maxlabels - 1)
			lb = Maxlabels - 1;
		//if (lb< 0)
		//	lb = 0;
		//cout << i << endl;
		//cout << lb << endl;
		LA[lb].len.push_back(order.at(i).len);
		LA[lb].angle.push_back(angles.at(i));
	}
	double LabelSlope[Maxlabels] = { 100000 };
	for (int i = 0; i < maxlabel; i++)  //ע��û��label0,
	{
		//if (LA[i].len.size()==0)		{				cout << "this maybe a bug" << endl;	cout << i << endl;		}
		//cout<<LA[i].len.size()<<endl;
		double lensum = 0;
		double lenAsum = 0;
		for (int j = 0; j < LA[i].len.size(); j++)
		{
			lensum += LA[i].len.at(j);
			lenAsum += LA[i].len.at(j)*LA[i].angle.at(j);

		}
		LabelSlope[i] = lenAsum / (lensum + DBL_MIN);
	}
	///////////��һ�������ǽ�ÿ��label��Ӧ�ĽǶ����///////////////
	//cout << "maxlabel   " << maxlabel << endl;
	//cout << "center.size   " << centerx.size() << endl;

	Vec4f matchline, curQline;
	double minSum = DBL_MAX;
	int minq = 0, minl = 0;
	int minD = 0;
	for (int i = 0; i < Qorder.size(); i++)
	{
		curQline = Qlines.at(Qorder.at(i).order);
		for (int j = 0; j < order.size(); j++)
		{
			if (Qorder.at(i).len < 0.8*order.at(j).len)
				continue;
			else if (Qorder.at(i).len > 1.2*order.at(j).len)
				break;
			else
			{
				double result[2] = { 0 };
				matchline = lines.at(order.at(j).order);
				lineMatch(distForMatch, labels, LabelSlope, maxlabel, matchline, curQline, result, Qorder, Qlines);
				if (minSum>min(result[0], result[1]))
				{
					minSum = min(result[0], result[1]);
					minq = i; minl = j;
					if (result[0] < result[1])
						minD = 0;
					else
						minD = 1;

				}
			}
		}
		if (i >0)
			break;
	}
		// cout <<"The matching Cost  "<< minSum << endl;
		// cout << " Gloable_th  " << Gloable_th << endl;
	if (Qlines.size() && lines.size() )
	{

		curQline = Qlines.at(Qorder.at(minq).order);

		matchline = lines.at(order.at(minl).order);  //
		if (shows)
		{
			Mat imgshow=src.clone();
			if (imgshow.channels() < 3)
			{
				vector<Mat> channels;
			 	channels.push_back(imgshow);
				channels.push_back(imgshow);
				channels.push_back(imgshow);
				merge(channels, imgshow); 
			}
			if (minSum < Gloable_th)
			{
				showMatch(imgshow, matchline, curQline, minD, Qlines, lines);
			}
			 
			
			// char fpsC[255];
			// std::sprintf(fpsC, "%d", (int)voronoiType);
			// std::string fpsSt("voronoiType:");
			// fpsSt += fpsC; 			fpsSt += "   distType";
			// std::sprintf(fpsC, "%d", (int)distType);			fpsSt += fpsC;
			// fpsSt += "   maskSize";			std::sprintf(fpsC, "%d", (int)maskSize);			fpsSt += fpsC;
			// cv::putText(imgshow, fpsSt, cv::Point(10, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255));
			//imshow("The Result", imgshow);
		}	
		if (minSum<Gloable_th)
		Match( matchline, curQline, minD ,rst);
	}


	if (shows)
	{		
		if (voronoiType < 0)   //����任ͼ���ӻ�

		{
			// begin "painting" the distance transform result
			dist *= 5000;
			pow(dist, 0.5, dist);
			Mat dist32s, dist8u1, dist8u2;

			dist.convertTo(dist32s, CV_32S, 1, 0.5);
			dist32s &= Scalar::all(255);

			dist32s.convertTo(dist8u1, CV_8U, 1, 0);
			dist32s *= -1;

			dist32s += Scalar::all(255);
			dist32s.convertTo(dist8u2, CV_8U);

			Mat planes[] = { dist8u1, dist8u2, dist8u2 };
			merge(planes, 3, dist8u);
		}
		else
		{
			dist8u.create(labels.size(), CV_8UC3);
			for (int i = 0; i < labels.rows; i++)
			{
				const int* ll = (const int*)labels.ptr(i);
				const float* dd = (const float*)dist.ptr(i);
				uchar* d = (uchar*)dist8u.ptr(i);
				for (int j = 0; j < labels.cols; j++)
				{
					int idx = ll[j] == 0 || dd[j] == 0 ? 0 : (ll[j] - 1) % 8 + 1;
					float scale = 1.f / (1 + dd[j] * dd[j] * 0.0004f);
					int b = cvRound(colors[idx][0] * scale);
					int g = cvRound(colors[idx][1] * scale);
					int r = cvRound(colors[idx][2] * scale);
					d[j * 3] = (uchar)b;
					d[j * 3 + 1] = (uchar)g;
					d[j * 3 + 2] = (uchar)r;
				}
			}
			//labels.convertTo(labels, CV_8U);
			//imshow("labels", labels);
		}
		resize(dist8u, dist8u, Size(dist8u.cols / 2, dist8u.rows / 2));
		//imshow("Distance Map", dist8u);
	}
	return minSum;
}

