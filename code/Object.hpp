#pragma once
#include <vector>
#include "Vector.hpp"

class Object
{
public:
	void Translate(double x, double y, double z);
	void Rotate(Point axis, double angle);
	void operator+=(const Object& obj);

	std::vector<Point> points;
	std::vector<std::array<int, 3>> triangles;
};