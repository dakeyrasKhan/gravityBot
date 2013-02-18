#pragma once
#include <vector>
#include "Vector.hpp"
#include "Matrix.hpp"

class Object
{
public:
	void Translate(double x, double y, double z);
	void Translate(const Point& p) { Translate(p[X], p[Y], p[Z]); };
	void Rotate(const double angle, const Point& axis);
	void Rotate(const double angle, const axis axis);
	void Rotate(const Matrix& m);
	void operator+=(const Object& obj);

	std::vector<Point> points;
	std::vector<std::array<int, 3>> triangles;
};