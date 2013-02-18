#pragma once
#include "Vector.hpp"
#include "Matrix.hpp"
#include "Object.hpp"


class Box
{
public:
	Box() { };
	Box(const Point& center, const Point& size, const Matrix& rotation = Matrix::Identity)
		: center(center), size(size), rotation(rotation) { };

	bool Intersect(Point p0, Point p1, Point p2) const;	
	bool IntersectBox(Box b) const;						
	bool IntersectSphere(Point p, const double radius) const;
	bool IntersectCylinder(Point c0, Point c1, const double radius) const;

	static Box GetRotationBoundingBox(const Point& size, 
		const Matrix& start, const Matrix& end, const double externRadius = 0);
	const Matrix& Rotation() const { return rotation; };
	const Point& Center() const { return center; };
	const Point& Size() const { return size; };

	Object GetObject() const;

private:
	Point center, size;
	Matrix rotation;

	static int FacePlaneMask(const Point& p);
	static int PlaneMask(const Point& p);
	static int CheckPoint(const Point& p0, const Point& p1, const double alpha, 
		const int mask);
	static bool CheckLine(const Point& p0, const Point& p1, const int outcode_diff);
};

