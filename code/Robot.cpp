#include "Robot.hpp"


Robot::Robot(Point size) : size(size)
{
	std::array<double, 3> vertices[] =
	{
		{0, 0, 0},
		{size[0], 0, 0},
		{0, size[1], 0},
		{0, 0, size[2]},
		{size[0], size[1], 0},
		{size[0], 0, size[2]},
		{0, size[1], size[2]},
		{size[0], size[1], size[2]}
	};

	std::array<int, 3> faces[] = 
	{
		{0, 1, 2},
		{1, 4, 2},
		{0, 1, 3},
		{1, 5, 3},
		{0, 2, 3},
		{2, 6, 3},
		{1, 4, 5},
		{4, 7, 5},
		{3, 5, 6},
		{5, 7, 6},
		{2, 6, 4},
		{4, 6, 7}
	};

	for(int i=0; i<8; i++)
		object.points.push_back(vertices[i]);
	for(int i=0; i<12; i++)
		object.triangles.push_back(faces[i]);
}


ozcollide::OBB Robot::GetBox() const
{
	ozcollide::OBB box;
	box.extent = ozcollide::Vec3f(size[0], size[1], size[2]);
	return box;
}