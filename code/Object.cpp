#include "Object.hpp"

void Object::Translate(double x, double y, double z)
{
	Point p;
	p[0] = x; p[1] = y; p[2] = z;

	for(auto& x : points)
		x += p;
}


void Object::Rotate(Point axis, double angle)
{
	axis = axis.Normalize();
	double c = cos(angle);
	double s = sin(angle);
	double x = axis[0];
	double y = axis[1];
	double z = axis[2];

	std::array<Point, 3> m;
	m[0][0] = x*x*(1-c) + c;	m[0][1] = x*y*(1-c) - z*s;	m[0][2] = z*x*(1-c) + y*s;
	m[1][0] = x*y*(1-c) + z*s;	m[1][1] = y*y*(1-c) + c;	m[1][2] = z*y*(1-c) - x*s;
	m[2][0] = x*z*(1-c) - y*s;	m[2][1] = y*z*(1-c) + x*s;	m[2][2] = z*z*(1-c) + c;

	for(auto& p : points)
	{
		Point p2 = p;
		p[0] = m[0][0]*p2[0] + m[0][1]*p2[1] + m[0][2]*p2[2];
		p[1] = m[1][0]*p2[0] + m[1][1]*p2[1] + m[1][2]*p2[2];
		p[2] = m[2][0]*p2[0] + m[2][1]*p2[1] + m[2][2]*p2[2];
	}
}