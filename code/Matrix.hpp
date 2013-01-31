#pragma once
#include "Vector.hpp"

class Matrix
{
public:
	Matrix() { };
	Matrix(const Point& l0, const Point& l1, const Point& l2);
	Matrix(const std::array<double, 3>* lignes);

	Point& operator[](const int i) { return lignes[i]; };
	const Point& operator[](const int i) const { return lignes[i]; };

	Matrix operator*(const Matrix& m) const;
	Point operator*(const Point& p) const;

	const static Matrix Identity;
	static Matrix Rotate(const double angle, const axis axis);
	static Matrix Rotate(const double angle, Point axis);

private:
	std::array<Point, 3> lignes;
};

