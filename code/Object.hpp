#pragma once
#include <vector>
#include "Vector.hpp"

class Object
{
public:
	void Translate(double x, double y, double z);
	void Rotate(const double angle, const Point& axis);
	void Rotate(const double angle, const axis axis);
	void operator+=(const Object& obj);

	std::vector<Point> points;
	std::vector<std::array<int, 3>> triangles;
};