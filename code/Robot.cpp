#include "Robot.hpp"
#include <cstdio>

Robot::Robot(Point s): baseSize(s), yPos(0.25)
{
	std::array<double, 3> vertices[] =
	{
		{0, 0, 0},
		{baseSize[0], 0, 0},
		{0, baseSize[1], 0},
		{0, 0, baseSize[2]},
		{baseSize[0], baseSize[1], 0},
		{baseSize[0], 0, baseSize[2]},
		{0, baseSize[1], baseSize[2]},
		{baseSize[0], baseSize[1], baseSize[2]}
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
	object.Translate(-baseSize[0]/2, -baseSize[1]/2, -baseSize[2]/2);
}

void Robot::printSize() const
{
	printf("robot size : %lf %lf %lf \n",baseSize[0],baseSize[1],baseSize[2]);
}

ozcollide::OBB Robot::GetBox() const
{
	ozcollide::OBB box;
	box.extent = ozcollide::Vec3f(baseSize[0]/2, baseSize[1]/2, baseSize[2]/2);
	return box;
}

Object Robot::GetObject(const Position& pos) const
{
	Point y; y[0] = 0; y[1] = 1; y[2] = 0;
	Object obj = object;
	obj.Rotate(y, pos[ROBOT_ROT]);
	obj.Translate(pos[ROBOT_X], yPos, pos[ROBOT_Z]);
	return obj;
}  