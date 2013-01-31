#pragma once
#include "Vector.hpp"
#include "Matrix.hpp"

class Box
{
public:
	Box(const Point& center, const Point& size, const Matrix& rotation = Matrix::Identity)
		: center(center), size(size), rotation(rotation) { };
	bool Intersect(const Point& p0, const Point& p1, const Point& p2);

private:
	Point center, size;
	Matrix rotation;
};

