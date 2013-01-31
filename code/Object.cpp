#include "Object.hpp"
#include "Matrix.hpp"

void Object::Translate(double x, double y, double z)
{
	Point p;
	p[0] = x; p[1] = y; p[2] = z;

	for(auto& x : points)
		x += p;
}


void Object::Rotate(const double angle, const Point& axis)
{
	Matrix m = Matrix::Rotate(angle, axis);
	for(auto& x : points)
		x = m*x;
}


void Object::Rotate(const double angle, const axis axis)
{
	Matrix m = Matrix::Rotate(angle, axis);
	for(auto& x : points)
		x = m*x;
}


void Object::operator+=(const Object& obj)
{
	int pointSize = points.size();
	int triangleSize = triangles.size();

	points.insert(points.end(), obj.points.begin(), obj.points.end());
	triangles.insert(triangles.end(), obj.triangles.begin(), obj.triangles.end());
	for(int i=triangleSize; i<triangles.size(); i++)
		for(auto& x : triangles[i])
			x += pointSize;
}