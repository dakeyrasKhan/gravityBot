#include "Robot.hpp"
#include <cstdio>

Robot::Robot(Point s): 
	baseSize(s), 
	yPos(0.25), 
	armWidth(0.1), 
	arm0Length(1),
	arm1Length(1)
{
	baseObject = BuildBox(s);
	baseObject.Translate(-s[0]/2, -s[1]/2, -s[2]/2);

	Point p; 
	p[0] = p[2] = armWidth;

	p[1] = arm0Length;
	arm0Object = BuildBox(p);
	arm0Object.Translate(-armWidth/2, 0, -armWidth/2);

	p[1] = arm1Length;
	arm1Object = BuildBox(p);
	arm1Object.Translate(-armWidth/2, 0, -armWidth/2);
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
	Point z; z[0] = 0; z[1] = 0; z[2] = 1;

	Object object = arm1Object;
	object.Rotate(z, pos[ROBOT_ARM1]);
	object.Translate(0, arm0Length, 0);

	object += arm0Object;
	object.Rotate(z, pos[ROBOT_ARM0]);
	object.Translate(0, baseSize[1]/2, 0);

	object += baseObject;
	object.Rotate(y, pos[ROBOT_ROT]);
	object.Translate(pos[ROBOT_X], yPos, pos[ROBOT_Z]);

	return object;
} 

Object Robot::BuildBox(const Point& size)
{
	Object object;
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

	return object;
}