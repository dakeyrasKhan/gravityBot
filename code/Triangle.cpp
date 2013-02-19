#include "Triangle.hpp"
#include <exception>
#include <iostream>

Triangle::Triangle(const Point& p0, const Point& p1, const Point& p2) 
	: p0(p0), p1(p1), p2(p2)
{
}

int Triangle::SignMask(const Point& p)
{
	return (p[X] > 100.*std::numeric_limits<double>::epsilon() ? 1 << 2 : 0)
		| (p[X] < -100.*std::numeric_limits<double>::epsilon() ? 1 << 5 : 0) 
		| (p[Y] >  100.*std::numeric_limits<double>::epsilon() ? 1 << 1 : 0) 
		| (p[Y] < -100.*std::numeric_limits<double>::epsilon() ? 1 << 4 : 0) 
		| (p[Z] >  100.*std::numeric_limits<double>::epsilon() ? 1 << 0 : 0)
		| (p[Z] < -100.*std::numeric_limits<double>::epsilon() ? 1 << 3 : 0);
}

bool Triangle::IntersectSphere(const Point& center, const double radius) const
{
	Point p[3], e[3];
	p[0] = p0 - center;
	p[1] = p1 - center;
	p[2] = p2 - center;

	double radius2 = radius*radius;
	for(int i=0; i<3; i++)
		if( p[i].Norm2()  < radius2)
			return true;
             
	// This is the buggy part
	for(int i=0; i<3; i++)
	{
		e[i] = p[i] - p[(i+1)%3];
		double n2 = e[i].Norm2();
		double d = (p[i] | e[i]);
		if(d > 0 && -d < n2 && p[i].Norm2() - d*d/n2 < radius2 )
			return true;
	}

	Point n = e[0]^e[1];
	n = n.Normalize();
	double d = (p[0]|n);
	if( abs(d) >= radius)
		return false;

	n = d*n;

	int sign01 = SignMask(e[0]^(p[0] - n)); 
	int sign12 = SignMask(e[1]^(p[1] - n));
	int sign20 = SignMask(e[2]^(p[2] - n));            
	return (sign01 & sign12 & sign20) != 0;
}

bool Triangle::IntersectPoint(const Point& p) const
{                 
#ifdef _DEBUG
	double test = (p0 - p) | ((p0-p1)^(p1-p2));
	Point t01 = (p0 - p1)^(p0 - p);
	Point t12 = (p1 - p2)^(p1 - p);
	Point t20 = (p2 - p0)^(p2 - p);
	int sign01 = SignMask(t01); 
	int sign12 = SignMask(t12);
	int sign20 = SignMask(t20); 
#else
	int sign01 = SignMask((p0 - p1)^(p0 - p)); 
	int sign12 = SignMask((p1 - p2)^(p1 - p));
	int sign20 = SignMask((p2 - p0)^(p2 - p));  
#endif

	return (sign01 & sign12 & sign20) != 0;
}

const Point& Triangle::operator[](const int i) const
{
	switch (i)
	{
	case 0:
		return p0;
	case 1:
		return p1;
	case 2:
		return p2;

	default:
#ifdef WIN32
		throw std::exception("Out of range index in Triangle");
#endif
		throw std::exception();	
	}
}

bool Triangle::IntersectCylinder(const Point& c0, const Point& c1, const double radius) const
{
	// check intersection with the edges
	if(SegmentSegmentDistance(p0, p1, c0, c1) < radius)
		return true;
	if(SegmentSegmentDistance(p1, p2, c0, c1) < radius)
		return true;
	if(SegmentSegmentDistance(p2, p1, c0, c1) < radius)
		return true;

	// check intersection with the inner 
	Point n = ((p0-p1)^(p0-p2)).Normalize();
	Point intersect;

	double origin = n|p0;
	double nC0 = (n|c0) - origin;
	double nC1 = (n|c1) - origin;

	if(nC0*nC1 > 0)
	{

		if(abs(nC0) < abs(nC1))
		{
			if(abs(nC0) > radius)
				return false;

			intersect = c0 - n*nC0;
		}
		else
		{
			if(abs(nC1) > radius)
				return false;

			intersect = c1 - n*nC1;
		}
	}
	else
	{
		Point dir = c1 - c0;
		intersect = c0 + (nC0/(dir|n))*dir.Normalize();
	}

	return IntersectPoint(intersect);
}