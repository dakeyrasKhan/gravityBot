#pragma once
#include "Vector.hpp"

class Triangle
{
public:
	Triangle(const Point& p0, const Point& p1, const Point& p2);

	bool IntersectSphere(const Point& p, const double radius) const;
	bool IntersectPoint(const Point& p) const;
	bool IntersectCylinder(const Point& c0, const Point& c1, const double radius) const;
	const Point& operator[](const int) const;

	static int SignMask(const Point& p);
private:
	Point p0, p1, p2;
};

