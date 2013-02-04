#pragma once
#include "Vector.hpp"
#include "Matrix.hpp"


class Box
{
public:
	Box(const Point& center, const Point& size, const Matrix& rotation = Matrix::Identity)
		: center(center), size(size), rotation(rotation) { };

	bool Intersect(Point p0, Point p1, Point p2) const;	// Intersect a triangle
	bool Intersect(Box b) const;						// Intersect a box
	bool Intersect(Point p, const double radius) const;	// Intersect a sphere
	static int SignMask(const Point& p);

private:
	Point center, size;
	Matrix rotation;

	static int FacePlaneMask(const Point& p);
	static int PlaneMask(const Point& p);
	static int CheckPoint(const Point& p0, const Point& p1, const double alpha, const int mask);
	static bool CheckLine(const Point& p0, const Point& p1, const int outcode_diff);
	static bool IntersectPointTriangle(const Point& p, const Point& p0, const Point& p1, const Point& p2);
};

