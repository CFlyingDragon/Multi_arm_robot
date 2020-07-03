#include<fstream>
#include<iostream>
#include<string>
#include<vector>
#include<cmath>
#include<ctime>
#include<algorithm>
#include<Windows.h>
#include"data.h"
using namespace std;

const double PI = 3.141592653;

vector<Triangle> TriangleVector;
vector<PointNorm> PointVector;
vector<PointNorm> Pose;

void TriShow(const Triangle& tri)      //��ӡ����������Ϣ
{
	cout << "norm: " << tri.norm.x << " " << tri.norm.y << " " << tri.norm.z << endl;
	for (int i = 0; i < 3; i++)
	{
		cout << "vertex: " << tri.vertex[i].x << " " << tri.vertex[i].y << " " << tri.vertex[i].z << endl;
	}
	cout << "length: " << tri.a << " " << tri.b << " " << tri.c << endl;
	cout << "area: " << tri.S << endl;
	cout << "angle: " << 180 / PI * tri.theta[0] << " " << 180 / PI * tri.theta[1] << " " << 180 / PI * tri.theta[2] << endl;
	cout << "delta: " << tri.delta << endl;
}

void PointNormShow(const PointNorm& p)     //��ӡ�������ꡢ���������ڽ����������
{
	cout << "Point: " << p.point.x << " " << p.point.y << " " << p.point.z << endl;
	cout << "NormDir: " << p.normdir.x << " " << p.normdir.y << " " << p.normdir.z << endl;

	cout << "TriIndex:";
	for (int i = 0; i < p.index.size(); i++)
	{
		cout << " " << p.index[i];
	}
	cout << endl;

	cout << "PointIndex:";
	for (int j = 0; j < p.PointIndex.size(); j++)
	{
		cout << " " << p.PointIndex[j];
	}
	cout << endl;
}

bool compare(const Point& p1, const Point& p2)     //�ȽϺ���
{
	if ((abs(p1.x - p2.x) <= 10e-6) && (abs(p1.y - p2.y) <= 10e-6) && (abs(p1.z - p2.z) <= 10e-6))
		return true;
	else
		return false;
}

float length(const Point& p1, const Point& p2)     //���������α߳�
{
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

float area(float a, float b, float c)     //�������������
{
	float p;
	p = (a + b + c) / 2;
	return sqrt(p*(p - a)*(p - b)*(p - c));
}

float angle(float a, float b, float c, int flag)    //�����������ڽ�
{
	if (flag == 0)
		return acos((b*b + c * c - a * a) / (2 * b*c));
	if (flag == 1)
		return acos((a*a + c * c - b * b) / (2 * a*c));
	if (flag == 2)
		return acos((a*a + b * b - c * c) / (2 * a*b));
}

float max3(float a, float b, float c)
{
	float t = a;
	if (b > t)
		t = b;
	if (c > t)
		t = c;
	return t;
}

float min3(float a, float b, float c)
{
	float t = a;
	if (b < t)
		t = b;
	if (c < t)
		t = c;
	return t;
}

float dot(const Point& p0, const Point& p1)      //�������
{
	return p0.x*p1.x + p0.y*p1.y + p0.z*p1.z;
}

Point cross(const Point& p0, const Point& p1)     //�������
{
	Point p;
	p.x = p0.y*p1.z - p0.z*p1.y;
	p.y = p0.z*p1.x - p0.x*p1.z;
	p.z = p0.x*p1.y - p0.y*p1.x;
	return p;
}

void normlize(Point& p0)        //������λ��
{
	float norm = sqrt(p0.x*p0.x + p0.y*p0.y + p0.z*p0.z);
	p0.x = p0.x / norm;
	p0.y = p0.y / norm;
	p0.z = p0.z / norm;
}

Point operator +(const Point& p0, const Point& p1)
{
	Point p;
	p.x = p0.x + p1.x;
	p.y = p0.y + p1.y;
	p.z = p0.z + p1.z;
	return p;
}

Point operator -(const Point& p0, const Point& p1)
{
	Point p;
	p.x = p0.x - p1.x;
	p.y = p0.y - p1.y;
	p.z = p0.z - p1.z;
	return p;
}

Point operator *(float len, const Point& p0)
{
	Point p;
	p.x = p0.x * len;
	p.y = p0.y * len;
	p.z = p0.z * len;
	return p;
}

Point operator /(const Point& p0, float len)
{
	Point p;
	p.x = p0.x / len;
	p.y = p0.y / len;
	p.z = p0.z / len;
	return p;
}

void ReadFile(string str)         //��ȡ�ļ�
{
	ifstream infile(str);
	if (!infile.is_open())
	{
		cout << "open error!" << endl;
		Sleep(1000);
		exit(0);
	}

	string temp;
	Triangle tempTriAngle;

	float max_angle;
	float min_angle;

	int trinumber = 0;

	infile >> temp;  //read "solid"
	if (temp.compare("solid") != 0)
	{
		exit(0);
	}

	infile >> temp;        //get rid of "file_name"

	infile >> temp;  //read "facet"
	while (temp.compare("facet") == 0)
	{
		trinumber++;
		infile >> temp;  //get rid of "normal "

		infile >> tempTriAngle.norm.x;  //save norm
		infile >> tempTriAngle.norm.y;
		infile >> tempTriAngle.norm.z;

		infile >> temp;
		infile >> temp;      //get rid of "outer loop"

		for (int i = 0; i < 3; i++)
		{
			infile >> temp;  //get rid of "vertex"
			infile >> tempTriAngle.vertex[i].x;   //save vertex
			infile >> tempTriAngle.vertex[i].y;
			infile >> tempTriAngle.vertex[i].z;

		}

		//solve length
		tempTriAngle.a = length(tempTriAngle.vertex[1], tempTriAngle.vertex[2]);
		tempTriAngle.b = length(tempTriAngle.vertex[0], tempTriAngle.vertex[2]);
		tempTriAngle.c = length(tempTriAngle.vertex[0], tempTriAngle.vertex[1]);

		//solve Area
		tempTriAngle.S = area(tempTriAngle.a, tempTriAngle.b, tempTriAngle.c);

		//solve angle
		tempTriAngle.theta[0] = angle(tempTriAngle.a, tempTriAngle.b, tempTriAngle.c, 0);
		tempTriAngle.theta[1] = angle(tempTriAngle.a, tempTriAngle.b, tempTriAngle.c, 1);
		tempTriAngle.theta[2] = angle(tempTriAngle.a, tempTriAngle.b, tempTriAngle.c, 2);

		//solve the max and min angle
		max_angle = max3(tempTriAngle.theta[0], tempTriAngle.theta[1], tempTriAngle.theta[2]);
		min_angle = min3(tempTriAngle.theta[0], tempTriAngle.theta[1], tempTriAngle.theta[2]);

		//�������ε���״����
		tempTriAngle.delta = (cos(min_angle) - cos(max_angle)) / 2;

		TriangleVector.push_back(tempTriAngle);  //save struct "triangle"

		infile >> temp;  //get rid of "endloop"
		infile >> temp;  //get rid of "endfacet"
		infile >> temp;  //read "facet"
	}

	infile.close();

	cout << "The number of triangle: " << trinumber << endl;
	cout << "The total number of vertex is: " << trinumber * 3 << endl;
	cout << "The last triangle:" << endl;
	TriShow(TriangleVector[trinumber - 1]);
}

void SavePoint(const vector<Triangle>& Tri)       //���涥����Ϣ
{
	PointNorm tempPoint;

	for (int i = 0; i < 3; i++)
	{
		tempPoint.point = Tri[0].vertex[i];
		tempPoint.index.push_back(0);  //save the index of adjacent triangle
		tempPoint.PointIndex.push_back(i);  //save the index of point
		PointVector.push_back(tempPoint);
		tempPoint.index.clear();
		tempPoint.PointIndex.clear();
	}

	int pointnumber = 3;

	for (int j = 1; j < Tri.size(); j++)
	{
		for (int k = 0; k < 3; k++)
		{
			for (int i = pointnumber - 1; i >= 0; i--)
			{
				if (compare(PointVector[i].point, Tri[j].vertex[k]) == true)
				{
					PointVector[i].index.push_back(j);
					PointVector[i].PointIndex.push_back(k);
					break;          //remove the repetitive point
				}
				if (i == 0)
				{
					tempPoint.point = Tri[j].vertex[k];
					tempPoint.index.push_back(j);  //save the index of adjacent triangle
					tempPoint.PointIndex.push_back(k);  //save the index of point
					PointVector.push_back(tempPoint);
					tempPoint.index.clear();
					tempPoint.PointIndex.clear();
					pointnumber++;
				}
			}
		}
	}

	cout << endl;
	cout << "The number of point: " << pointnumber << endl;
}

void SolveNorm(vector<PointNorm>& p, const vector<Triangle>& Tri)    //���㷨����
{
	float sum_x = 0;
	float sum_y = 0;
	float sum_z = 0;
	float vectorNorm;
	int sum = 0;

	int mode = 2;  //ѡ����㷨�����ķ���

	for (int i = 0; i < p.size(); i++)
	{
		for (int j = 0; j < p[i].index.size(); j++)
		{
			if (mode == 1)   //ȡƽ��ֵ
			{
				sum_x += Tri[p[i].index[j]].norm.x;
				sum_y += Tri[p[i].index[j]].norm.y;
				sum_z += Tri[p[i].index[j]].norm.z;
			}

			if (mode == 2)   //�����Ȩƽ��ֵ
			{
				sum_x += Tri[p[i].index[j]].norm.x * Tri[p[i].index[j]].S;
				sum_y += Tri[p[i].index[j]].norm.y * Tri[p[i].index[j]].S;
				sum_z += Tri[p[i].index[j]].norm.z * Tri[p[i].index[j]].S;
			}

			if (mode == 3)   //������������״�����ķ�ʸ����
			{
				sum_x += (1 - Tri[p[i].index[j]].delta)*Tri[p[i].index[j]].theta[p[i].PointIndex[j]] * Tri[p[i].index[j]].norm.x / Tri[p[i].index[j]].S;
				sum_y += (1 - Tri[p[i].index[j]].delta)*Tri[p[i].index[j]].theta[p[i].PointIndex[j]] * Tri[p[i].index[j]].norm.y / Tri[p[i].index[j]].S;
				sum_z += (1 - Tri[p[i].index[j]].delta)*Tri[p[i].index[j]].theta[p[i].PointIndex[j]] * Tri[p[i].index[j]].norm.z / Tri[p[i].index[j]].S;
			}

			if (mode == 4)   //��Ӧ�ڽǼ�Ȩƽ��ֵ
			{
				sum_x += Tri[p[i].index[j]].theta[p[i].PointIndex[j]] * Tri[p[i].index[j]].norm.x;
				sum_y += Tri[p[i].index[j]].theta[p[i].PointIndex[j]] * Tri[p[i].index[j]].norm.y;
				sum_z += Tri[p[i].index[j]].theta[p[i].PointIndex[j]] * Tri[p[i].index[j]].norm.z;
			}

			if (mode == 5)   //��Ӧ�ڽ����Ҽ�Ȩƽ��ֵ
			{
				sum_x += sin(Tri[p[i].index[j]].theta[p[i].PointIndex[j]]) * Tri[p[i].index[j]].norm.x;
				sum_y += sin(Tri[p[i].index[j]].theta[p[i].PointIndex[j]]) * Tri[p[i].index[j]].norm.y;
				sum_z += sin(Tri[p[i].index[j]].theta[p[i].PointIndex[j]]) * Tri[p[i].index[j]].norm.z;
			}
		}

		sum += p[i].index.size();

		vectorNorm = sqrt(sum_x*sum_x + sum_y * sum_y + sum_z * sum_z);
		sum_x /= vectorNorm;
		sum_y /= vectorNorm;
		sum_z /= vectorNorm;   //��λ��

		p[i].normdir.x = sum_x;
		p[i].normdir.y = sum_y;
		p[i].normdir.z = sum_z;   //solve the unit normal vector of vertex
	}

	cout << endl;
	cout << "The total number of vertex is: " << sum << endl;
	cout << "The first point:" << endl;
	PointNormShow(p[0]);
	cout << endl;
}

void VertexDir(const vector<PointNorm>& p, vector<Triangle>& Tri)    //�����������������ķ�����
{
	for (int i = 0; i < p.size(); i++)
	{
		for (int j = 0; j < p[i].index.size(); j++)
		{
			Tri[p[i].index[j]].normdir[p[i].PointIndex[j]] = p[i].normdir;
		}
	}
}

float FindMax(const vector<PointNorm>& p)       //�����ֵ
{
	float max;
	max = p[0].point.x;
	for (int i = 1; i < p.size(); i++)
	{
		if (p[i].point.x > max)
			max = p[i].point.x;
	}
	return max;
}

float FindMin(const vector<PointNorm>& p)      //����Сֵ
{
	float min;
	min = p[0].point.x;
	for (int i = 1; i < p.size(); i++)
	{
		if (p[i].point.x < min)
			min = p[i].point.x;
	}
	return min;
}

bool operator<(const PointNorm& p1, const PointNorm& p2)
{
	if (p1.point.y < p2.point.y)
		return true;
	else
		return false;
}

vector<Point> Bezier_curve_control_point(const Point& v0, const Point& v3, const Point& n1, const Point& n2)  //���������߿��Ƶ�
{
	Point b = v3 - v0;
	Point n = cross(b, n1);
	normlize(n);
	Point n22 = n2 - dot(n, n2)*n;
	normlize(n22);

	Point s1 = cross(n1, n);
	Point s2 = cross(n22, n);

	float a = dot(s1, s2);
	float t1 = (2 * dot(s1, b) - a * dot(s2, b)) / (4 - a * a);
	float t2 = (2 * dot(s2, b) - a * dot(s1, b)) / (4 - a * a);

	Point v1 = v0 + t1 * s1;
	Point v2 = v3 - t2 * s2;

	vector<Point> v;
	v.push_back(v1);
	v.push_back(v2);
	return v;
}

vector<Point> Bezier_curve(const Point& v0, const Point& v1, const Point& v2, const Point& v3, float t)     //���������߲�ֵ
{
	Point v = pow(1 - t, 3)*v0 + 3 * t*(1 - t)*(1 - t)*v1 + 3 * t*t*(1 - t)*v2 + pow(t, 3)*v3;
	Point v_d = -(1 - t)*(1 - t)*v0 + (1 - t)*(1 - 3 * t)*v1 + t*(2 - 3 * t)*v2 + t*t*v3;
	normlize(v_d);

	vector<Point> PointDir;
	PointDir.push_back(v);
	PointDir.push_back(v_d);
	return PointDir;
}

float Bezier_curve_alpha(const Point& v0, const Point& v1, const Point& v2, const Point& v3, float t)   //���ַ����������߲�ֵϵ��
{
	float left = 0;
	float right = 1;
	float alpha = 0.5;
	vector<Point> v = Bezier_curve(v0, v1, v2, v3, alpha);

	while (abs(v[0].x - t) > 0.05)
	{
		if (v[0].x < t)
			left = alpha;
		else
			right = alpha;

		alpha = (left + right) / 2;
		v = Bezier_curve(v0, v1, v2, v3, alpha);
	}

	return alpha;
}

PointNorm Intersection(const Triangle & Tri, int a, int b, float t)    //���ƽ������������Ľؽ������꼰��ʸ
{
	PointNorm p2;

	vector<Point> control_point = Bezier_curve_control_point(Tri.vertex[a], Tri.vertex[b], Tri.normdir[a], Tri.normdir[b]);

	float alpha = Bezier_curve_alpha(Tri.vertex[a], control_point[0], control_point[1], Tri.vertex[b], t);

	vector<Point> v = Bezier_curve(Tri.vertex[a], control_point[0], control_point[1], Tri.vertex[b], alpha);

	Point face_norm = cross(Tri.vertex[b] - Tri.vertex[a], Tri.normdir[a]);
	Point Dir = cross(face_norm, v[1]);
	normlize(Dir);

	p2.point = v[0];
	p2.normdir = Dir;

	return p2;
}

void SolveSection(vector<PointNorm>& p, const vector<Triangle>& Tri, float t)       //��������������ؽ���
{
	PointNorm pointnorm;
	for (int i = 0; i < Tri.size(); i++)
	{
		if (abs(Tri[i].vertex[0].x - t) < 0.001)
		{
			if (abs(Tri[i].vertex[1].x - t) < 0.001)
			{
				if (abs(Tri[i].vertex[2].x - t) < 0.001)
				{
					for (int k = 0; k < 3; k++)
					{
						pointnorm.point = Tri[i].vertex[k];
						pointnorm.normdir = Tri[i].normdir[k];
						p.push_back(pointnorm);
					}
				}
				else
				{
					for (int k = 0; k < 2; k++)
					{
						pointnorm.point = Tri[i].vertex[k];
						pointnorm.normdir = Tri[i].normdir[k];
						p.push_back(pointnorm);
					}
				}
			}

			else if (Tri[i].vertex[1].x < t)
			{
				if (abs(Tri[i].vertex[2].x - t) < 0.001)
				{
					pointnorm.point = Tri[i].vertex[0];
					pointnorm.normdir = Tri[i].normdir[0];
					p.push_back(pointnorm);

					pointnorm.point = Tri[i].vertex[2];
					pointnorm.normdir = Tri[i].normdir[2];
					p.push_back(pointnorm);
				}
				else if (Tri[i].vertex[2].x < t)
				{
					pointnorm.point = Tri[i].vertex[0];
					pointnorm.normdir = Tri[i].normdir[0];
					p.push_back(pointnorm);
				}
				else
				{
					pointnorm.point = Tri[i].vertex[0];
					pointnorm.normdir = Tri[i].normdir[0];
					p.push_back(pointnorm);

					pointnorm = Intersection(Tri[i], 1, 2, t);
					p.push_back(pointnorm);
				}
			}

			else
			{
				if (abs(Tri[i].vertex[2].x - t) < 0.001)
				{
					pointnorm.point = Tri[i].vertex[0];
					pointnorm.normdir = Tri[i].normdir[0];
					p.push_back(pointnorm);

					pointnorm.point = Tri[i].vertex[2];
					pointnorm.normdir = Tri[i].normdir[2];
					p.push_back(pointnorm);
				}
				else if (Tri[i].vertex[2].x < t)
				{
					pointnorm.point = Tri[i].vertex[0];
					pointnorm.normdir = Tri[i].normdir[0];
					p.push_back(pointnorm);

					pointnorm = Intersection(Tri[i], 2, 1, t);
					p.push_back(pointnorm);
				}
				else
				{
					pointnorm.point = Tri[i].vertex[0];
					pointnorm.normdir = Tri[i].normdir[0];
					p.push_back(pointnorm);
				}
			}
		}

		else if (Tri[i].vertex[0].x < t)
		{
			if (abs(Tri[i].vertex[1].x - t) < 0.001)
			{
				if (abs(Tri[i].vertex[2].x - t) < 0.001)
				{
					for (int k = 1; k < 3; k++)
					{
						pointnorm.point = Tri[i].vertex[k];
						pointnorm.normdir = Tri[i].normdir[k];
						p.push_back(pointnorm);
					}
				}
				else if (Tri[i].vertex[2].x < t)
				{
					pointnorm.point = Tri[i].vertex[1];
					pointnorm.normdir = Tri[i].normdir[1];
					p.push_back(pointnorm);
				}
				else
				{
					pointnorm.point = Tri[i].vertex[1];
					pointnorm.normdir = Tri[i].normdir[1];
					p.push_back(pointnorm);

					pointnorm = Intersection(Tri[i], 0, 2, t);
					p.push_back(pointnorm);
				}
			}

			else if (Tri[i].vertex[1].x < t)
			{
				if (abs(Tri[i].vertex[2].x - t) < 0.001)
				{
					pointnorm.point = Tri[i].vertex[2];
					pointnorm.normdir = Tri[i].normdir[2];
					p.push_back(pointnorm);
				}
				else if (Tri[i].vertex[2].x < t)
				{
					continue;
				}
				else
				{
					pointnorm = Intersection(Tri[i], 0, 2, t);
					p.push_back(pointnorm);

					pointnorm = Intersection(Tri[i], 1, 2, t);
					p.push_back(pointnorm);
				}
			}

			else
			{
				if (abs(Tri[i].vertex[2].x - t) < 0.001)
				{
					pointnorm.point = Tri[i].vertex[2];
					pointnorm.normdir = Tri[i].normdir[2];
					p.push_back(pointnorm);

					pointnorm = Intersection(Tri[i], 0, 1, t);
					p.push_back(pointnorm);
				}
				else if (Tri[i].vertex[2].x < t)
				{
					pointnorm = Intersection(Tri[i], 0, 1, t);
					p.push_back(pointnorm);

					pointnorm = Intersection(Tri[i], 2, 1, t);
					p.push_back(pointnorm);
				}
				else
				{
					pointnorm = Intersection(Tri[i], 0, 1, t);
					p.push_back(pointnorm);

					pointnorm = Intersection(Tri[i], 0, 2, t);
					p.push_back(pointnorm);
				}
			}
		}

		else
		{
			if (abs(Tri[i].vertex[1].x - t) < 0.001)
			{
				if (abs(Tri[i].vertex[2].x - t) < 0.001)
				{
					for (int k = 1; k < 3; k++)
					{
						pointnorm.point = Tri[i].vertex[k];
						pointnorm.normdir = Tri[i].normdir[k];
						p.push_back(pointnorm);
					}
				}
				else if (Tri[i].vertex[2].x < t)
				{
					pointnorm.point = Tri[i].vertex[1];
					pointnorm.normdir = Tri[i].normdir[1];
					p.push_back(pointnorm);

					pointnorm = Intersection(Tri[i], 2, 0, t);
					p.push_back(pointnorm);
				}
				else
				{
					pointnorm.point = Tri[i].vertex[1];
					pointnorm.normdir = Tri[i].normdir[1];
					p.push_back(pointnorm);
				}
			}

			else if (Tri[i].vertex[1].x < t)
			{
				if (abs(Tri[i].vertex[2].x - t) < 0.001)
				{
					pointnorm.point = Tri[i].vertex[2];
					pointnorm.normdir = Tri[i].normdir[2];
					p.push_back(pointnorm);

					pointnorm = Intersection(Tri[i], 1, 0, t);
					p.push_back(pointnorm);
				}
				else if (Tri[i].vertex[2].x < t)
				{
					pointnorm = Intersection(Tri[i], 1, 0, t);
					p.push_back(pointnorm);

					pointnorm = Intersection(Tri[i], 2, 0, t);
					p.push_back(pointnorm);
				}
				else
				{
					pointnorm = Intersection(Tri[i], 1, 0, t);
					p.push_back(pointnorm);

					pointnorm = Intersection(Tri[i], 1, 2, t);
					p.push_back(pointnorm);
				}
			}

			else
			{
				if (abs(Tri[i].vertex[2].x - t) < 0.001)
				{
					pointnorm.point = Tri[i].vertex[2];
					pointnorm.normdir = Tri[i].normdir[2];
					p.push_back(pointnorm);
				}
				else if (Tri[i].vertex[2].x < t)
				{
					pointnorm = Intersection(Tri[i], 2, 0, t);
					p.push_back(pointnorm);

					pointnorm = Intersection(Tri[i], 2, 1, t);
					p.push_back(pointnorm);
				}
				else
				{
					continue;
				}
			}
		}
	}
}

void Dispersed(vector<PointNorm>& p, float step)   //���ܼ����޳�
{
	vector<PointNorm> buffer;
	Point point0 = p[0].point;

	buffer.push_back(p[0]);

	for (int i = 1; i < p.size(); i++)
	{
		if (length(point0, p[i].point) >= step)
		{
			buffer.push_back(p[i]);
			point0 = p[i].point;
		}
	}

	p = buffer;

}

void Interpolation(vector<PointNorm>& p, float step)   //��ϡ���֮���ñ��������߲�ֵ
{
	vector<PointNorm> buffer;
	PointNorm point0 = p[0];
	PointNorm interpoint;
	int n;

	buffer.push_back(p[0]);

	for (int i = 1; i < p.size(); i++)
	{
		if (length(point0.point, p[i].point) >= 2 * step)
		{
			n = length(point0.point, p[i].point) / step;

			vector<Point> control_point = Bezier_curve_control_point(point0.point, p[i].point, point0.normdir, p[i].normdir);

			Point face_norm = cross(p[i].point - point0.point, point0.normdir);

			for (int j = 1; j < n; j++)
			{
				vector<Point> v = Bezier_curve(point0.point, control_point[0], control_point[1], p[i].point, float(j)/n);

				Point Dir = cross(face_norm, v[1]);
				normlize(Dir);

				interpoint.point = v[0];
				interpoint.normdir = Dir;

				buffer.push_back(interpoint);
			}

		}

		buffer.push_back(p[i]);

		point0 = p[i];
	}

	p = buffer;

}

void SolveStep(vector<float>& nz, const vector<PointNorm>& p, float h)    //��������������з�������z�����
{
	float max = FindMax(p);
	float min = FindMin(p);

	int count = 0;
	float sum = 0;

	for (float t = min; t < max; t += h)
	{
		for (int i = 0; i < p.size(); i++)
		{
			if ((p[i].point.x > t-h/2) && (p[i].point.x < t + h/2))
			{
				count++;
				sum += p[i].normdir.z;
			}
		}

		sum /= count;
		nz.push_back(abs(sum));

		count = 0;
		sum = 0;
	}
}

void PathPlan(const vector<PointNorm>& p, const vector<Triangle>& Tri, float step)    //·���滮
{
	float max = FindMax(p);
	float min = FindMin(p);

	vector<PointNorm> temp;
	vector<float> nz;
	float h = 2;//0.8 * step;
	int flag = 0;

	SolveStep(nz, p, h);

	for (int i = 0; i < nz.size(); i++)
		cout << nz[i] << ' ';
	cout << endl;

	float dist = step * nz[0];

	for (float t = min; t <= max; t += dist)   //ֱ��t��x�����������ɨ��
	{

		for (int i = 1; i < nz.size(); i++)
		{
			if ((t >= min + (i-0.5)*h) && (t < min + (i+0.5)*h))
			{
				dist = step * nz[i];    //����Ӧ������ƽ�沽��ֵ
				break;
			}
		}

		SolveSection(temp, Tri, t);    //���ƽ������������Ľ���

		flag = !flag;

		if (flag)
			sort(temp.begin(), temp.end());   //��y��������
		else
			sort(temp.rbegin(), temp.rend());

		if (!temp.empty())
		{
			Dispersed(temp, 0.2);      //�����������ݵ���Ϊ0.2mm
			Interpolation(temp, 0.2);
		}

		for (int j = 0; j < temp.size(); j++)
			Pose.push_back(temp[j]);

		cout << temp.size() << endl;

		temp.clear();
	}

	cout << endl;
	cout << "The number of point after path planning: " << Pose.size() << endl;
}

void OutFile(const vector<PointNorm>& p, string str)    //������������Ϣ
{
	ofstream outfile(str);
	if (!outfile)
	{
		cout << "open error!" << endl;
		Sleep(1000);
		exit(0);
	}

	for (int i = 0; i < p.size(); i++)
	{
		outfile << "pose" << " " << p[i].point.x << " " << p[i].point.y << " " << p[i].point.z << " ";
		outfile << p[i].normdir.x << " " << p[i].normdir.y << " " << p[i].normdir.z << endl;
	}
	//��·���滮�Ľ�����浽�ı�
	outfile.close();
}

void ReadTest(string str)     //��ȡ�ļ�
{
	ifstream infile(str);
	if (!infile)
	{
		cout << "open error!" << endl;
		Sleep(1000);
		exit(0);
	}

	cout << endl;
	string Temp;
	for (int i = 0; i < 8; i++)
	{
		infile >> Temp;
		cout << Temp << endl;  //��ȡ�ı����ݲ���ʾ
	}

	infile.close();
}

int main()
{
	clock_t start, end1;

	start = clock();

	ReadFile("sphere.stl");    //����stlģ��(ASCII���ʽ)���洢����������Ϣ�������ļ����Լ��趨��

	SavePoint(TriangleVector);     //��ȡ��������Ķ�����Ϣ�����øģ�

	SolveNorm(PointVector, TriangleVector);     //���������ڸ������㴦�ķ����������øģ�

	VertexDir(PointVector, TriangleVector);      //�����������񶥵㴦�����������øģ�

	PathPlan(PointVector, TriangleVector, 4.0);      //·���滮��4.0��ʾ�滮��·���о�Ϊ4mm���о�����Լ��趨��

	OutFile(Pose, "sphere_4_10_1.txt");    //�����滮��ĵ���ĩ����̬���ı��ļ��������ļ����Լ��趨��

	ReadTest("sphere_4_10_1.txt");      //��ȡ����ĩ����̬���ı��ļ��������ã����ÿ�������ע�͵���
										//����������㵥λ��mm�������������stlģ�͵�����ϵ���ԣ�������Ϊ��λ������
										//�����ļ���ʽ����һ���ַ���"pose"����2-4�У���ά���ꣻ��5-7�У�������
	end1 = clock();

	cout << endl << (double)(end1 - start) / CLOCKS_PER_SEC <<"s"<< endl;    //����ִ��ʱ��

	system("pause");
	return 0;
}
