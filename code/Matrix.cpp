#include "Matrix.hpp"


std::array<double, 3> id[] =
{
	{ 1, 0, 0 },
	{ 0, 1, 0 },
	{ 0, 0, 1 }
};

const Matrix Matrix::Identity = Matrix(id);

Matrix::Matrix(const Point& l0, const Point& l1, const Point& l2)
{
	lignes[0] = l0;
	lignes[1] = l1;
	lignes[2] = l2;
}

Matrix::Matrix(const std::array<double, 3>* lignes)
{
	this->lignes[0] = lignes[0];
	this->lignes[1] = lignes[1];
	this->lignes[2] = lignes[2];
}

Matrix Matrix::operator*(const Matrix& m) const
{
	Matrix out;
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			out[i][j] = lignes[i][0]*m[0][j] + lignes[i][1]*m[1][j] + lignes[i][2]*m[2][j];

	return out;
}

Point Matrix::operator*(const Point& p) const
{
	Point out;
	for(int i=0; i<3; i++)
		out[i] = p[0]*lignes[i][0] + p[1]*lignes[i][1] + p[2]*lignes[i][2];
	return out;
}

Point operator*(const Point& p, const Matrix& m)
{
	Point out;
	for(int i=0; i<3; i++)
		out[i] = p[0]*m[0][i] + p[1]*m[1][i] + p[2]*m[2][i];
	return out;
}


Matrix Matrix::Rotate(const double angle, const axis axis)
{
	Matrix out = Matrix::Identity;
	double c = cos(angle);
	double s = sin(angle);

	switch (axis)
	{
	case X:
		out[1][1] = c;
		out[1][2] = -s;
		out[2][1] = s;
		out[2][2] = c;
		break;
	case Y:
		out[0][0] = c;
		out[0][2] = -s;
		out[2][0] = s;
		out[2][2] = c;
		break;
	case Z:
		out[0][0] = c;
		out[0][1] = -s;
		out[1][0] = s;
		out[1][1] = c;
		break;
	}

	return out;
}

Matrix Matrix::Rotate(const double angle, Point axis)
{
	axis = axis.Normalize();
	double c = cos(angle);
	double s = sin(angle);
	double x = axis[0];
	double y = axis[1];
	double z = axis[2];

	Matrix m;
	m[0][0] = x*x*(1-c) + c;	m[0][1] = x*y*(1-c) - z*s;	m[0][2] = z*x*(1-c) + y*s;
	m[1][0] = x*y*(1-c) + z*s;	m[1][1] = y*y*(1-c) + c;	m[1][2] = z*y*(1-c) - x*s;
	m[2][0] = x*z*(1-c) - y*s;	m[2][1] = y*z*(1-c) + x*s;	m[2][2] = z*z*(1-c) + c;

	return m;
}

Matrix Matrix::Transpose() const
{
	Matrix out;
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			out[i][j] = lignes[j][i];

	return out;
}

double Matrix::Trace() const
{
	return lignes[0][0] + lignes[1][1] + lignes[2][2];
}