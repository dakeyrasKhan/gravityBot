#pragma once
#include "Vector.hpp"
#include "Matrix.hpp"
#include <ozcollide\OBB.h>


class Box
{
public:
	Box(const Point& center, const Point& size, const Matrix& rotation = Matrix::Identity)
		: center(center), size(size), rotation(rotation) { };

	// bool Intersect(const Point& p0, const Point& p1, const Point& p2);
	ozcollide::OBB ToOzcollide() const;

private:
	Point center, size;
	Matrix rotation;
};

