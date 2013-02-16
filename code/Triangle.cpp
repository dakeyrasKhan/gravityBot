#include "Triangle.hpp"
#include <exception>

Triangle::Triangle(const Point& p0, const Point& p1, const Point& p2) 
	: p0(p0), p1(p1), p2(p2)
{
}

int Triangle::SignMask(const Point& p)
{
	return (p[X] > std::numeric_limits<double>::epsilon() ? 1 << 2 : 0)
		| (p[X] < -std::numeric_limits<double>::epsilon() ? 1 << 5 : 0) 
		| (p[Y] >  std::numeric_limits<double>::epsilon() ? 1 << 1 : 0) 
		| (p[Y] < -std::numeric_limits<double>::epsilon() ? 1 << 4 : 0) 
		| (p[Z] >  std::numeric_limits<double>::epsilon() ? 1 << 0 : 0)
		| (p[Z] < -std::numeric_limits<double>::epsilon() ? 1 << 3 : 0);
}

bool Triangle::IntersectSphere(const Point& center, const double radius) const
{
	Point p[3], e[3];
	p[0] = p0 - center;
	p[1] = p1 - center;
	p[2] = p2 - center;

	double radius2 = radius*radius;
	for(int i=0; i<3; i++)
		if( (p[i]|p[i])  < radius2)
			return true;
                    
	for(int i=0; i<3; i++)
	{
		e[i] = p[i] - p[(i+1)%3];
		double n2 = e[i].Norm2();
		double d = -(p[i] | e[i]);
		if(d > 0 && d < n2 && p[i].Norm2() - d*d/n2 < radius2 )
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
	int sign01 = SignMask((p0 - p1)^(p0 - p)); 
	int sign12 = SignMask((p1 - p2)^(p1 - p));
	int sign20 = SignMask((p2 - p0)^(p2 - p));            
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

	// check intersection with the inner triangle
	return IntersectPoint(SegmentPlaneClosestPoint(((p0-p1)^(p0-p2)).Normalize(), p0, c0, c1));
}