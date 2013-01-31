#include "Robot.hpp"
#include <cstdio>

Robot::Robot(Point s): 
	baseSize(s), 
	yPos(0.25), 
	armWidth(0.1), 
	arm0Length(1),
	arm1Length(1),
	baseArmLength(0.2)
{
	Point p;
	p[0] = p[2] = armWidth;

	baseObject = BuildBox(s);
	baseObject.Translate(-s[0]/2, 0, -s[2]/2);

	p[1] = baseArmLength;
	Object baseArm = BuildBox(p);
	baseArm.Translate(-armWidth/2, s[1], -armWidth/2);
	baseObject += baseArm;


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

std::array<ozcollide::OBB, 2> Robot::GetBaseBoxes(const Position& pos) const
{
	std::array<ozcollide::OBB, 2> boxes;

	boxes[0].center = ozcollide::Vec3f(pos[ROBOT_X], yPos, pos[ROBOT_Z]);
	boxes[0].extent = ozcollide::Vec3f(baseSize[0]/2, baseSize[1]/2, baseSize[2]/2);

	double baseArmY = yPos + baseSize[1]/2 + baseArmLength/2;
	boxes[1].center = ozcollide::Vec3f(pos[ROBOT_X], baseArmY, pos[ROBOT_Z]);
	boxes[1].extent = ozcollide::Vec3f(armWidth/2, baseArmLength/2, armWidth/2);


	boxes[0].matrix.m_[0][0] = cos(pos[ROBOT_ROT]); 
	boxes[0].matrix.m_[0][1] = 0; 
	boxes[0].matrix.m_[0][2] = sin(pos[ROBOT_ROT]);
	boxes[0].matrix.m_[1][0] = 0;
	boxes[0].matrix.m_[1][1] = 1;
	boxes[0].matrix.m_[1][2] = 0;
	boxes[0].matrix.m_[2][0] = -sin(pos[ROBOT_ROT]); 
	boxes[0].matrix.m_[2][1] = 0; 
	boxes[0].matrix.m_[2][2] = cos(pos[ROBOT_ROT]);
	boxes[1].matrix = boxes[0].matrix;

	return boxes;
}

std::array<ozcollide::OBB, 2> Robot::GetArmsBoxes(const Position& pos) const
{
	std::array<ozcollide::OBB, 2> boxes;
	boxes[0].extent = ozcollide::Vec3f(armWidth/2, arm0Length/2, armWidth/2);
	boxes[0].center = ozcollide::Vec3f(0, arm0Length/2, 0);

	boxes[1].extent = ozcollide::Vec3f(armWidth/2, arm1Length/2, armWidth/2);
	boxes[1].center = ozcollide::Vec3f(0, arm1Length/2, 0);

	boxes[0].matrix.m_[0][0] = cos(pos[ROBOT_ROT])*cos(pos[ROBOT_ARM0]); 
	boxes[0].matrix.m_[0][1] = -cos(pos[ROBOT_ROT])*sin(pos[ROBOT_ARM0]); 
	boxes[0].matrix.m_[0][2] = sin(pos[ROBOT_ROT]);
	boxes[0].matrix.m_[1][0] = sin(pos[ROBOT_ARM0]);
	boxes[0].matrix.m_[1][1] = cos(pos[ROBOT_ARM0]);
	boxes[0].matrix.m_[1][2] = 0;
	boxes[0].matrix.m_[2][0] = -sin(pos[ROBOT_ROT])*cos(pos[ROBOT_ARM0]); 
	boxes[0].matrix.m_[2][1] = sin(pos[ROBOT_ROT])*sin(pos[ROBOT_ARM0]); 
	boxes[0].matrix.m_[2][2] = cos(pos[ROBOT_ROT]);

	double alpha = pos[ROBOT_ARM0] + pos[ROBOT_ARM1];
	boxes[1].matrix.m_[0][0] = cos(pos[ROBOT_ROT])*cos(alpha); 
	boxes[1].matrix.m_[0][1] = -cos(pos[ROBOT_ROT])*sin(alpha);
	boxes[1].matrix.m_[0][2] = sin(pos[ROBOT_ROT]);
	boxes[1].matrix.m_[1][0] = sin(alpha);
	boxes[1].matrix.m_[1][1] = cos(alpha);
	boxes[1].matrix.m_[1][2] = 0;
	boxes[1].matrix.m_[2][0] = -sin(pos[ROBOT_ROT])*cos(alpha);
	boxes[1].matrix.m_[2][1] = sin(pos[ROBOT_ROT])*sin(alpha); 
	boxes[1].matrix.m_[2][2] = cos(pos[ROBOT_ROT]);
	
	boxes[0].center = boxes[0].matrix.mul(boxes[0].center);
	boxes[1].center = boxes[1].matrix.mul(boxes[1].center);
	boxes[1].center += 2.*boxes[0].center;

	ozcollide::Vec3f baseArmTop(pos[ROBOT_X], yPos + baseSize[1]/2 + baseArmLength, pos[ROBOT_Z]);
	boxes[0].center += baseArmTop;
	boxes[1].center += baseArmTop;

	return boxes;
}

Object Robot::GetObject(const Position& pos) const
{
	Object object = arm1Object;
	object.Rotate(pos[ROBOT_ARM1], Z);
	object.Translate(0, arm0Length, 0);

	object += arm0Object;
	object.Rotate(pos[ROBOT_ARM0], Z);
	object.Translate(0, baseSize[1]+baseArmLength, 0);

	object += baseObject;
	object.Rotate(pos[ROBOT_ROT], Y);
	object.Translate(pos[ROBOT_X], yPos - baseSize[1]/2, pos[ROBOT_Z]);

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