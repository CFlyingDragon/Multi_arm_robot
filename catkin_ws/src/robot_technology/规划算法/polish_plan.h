#ifndef POLISH_PLAN_H_INCLUDED
#define POLISH_PLAN_H_INCLUDED

#include<vector>

struct Point
{
	float x;
	float y;
	float z;
};

struct Triangle
{
	Point norm;
	Point vertex[3];
	Point normdir[3];
	float a;
	float b;
	float c;
	float S;
	float theta[3];
	float delta;
};

struct PointNorm
{
	Point point;
	Point normdir;
	std::vector<int> index;
	std::vector<int> PointIndex;
};

#endif // POLISH_PLAN_H_INCLUDED
