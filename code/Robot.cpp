#include "Robot.hpp"
#include <cstdio>

Position Robot::SetAngles(double dist,Position p){
	Position res = p;
	//Al-Kashi
	double angle1 = acos((dist*dist+arm0Length*arm0Length-
						  arm1Length*arm1Length)/
						(2.*dist*arm0Length));

	double angle2 = acos((arm0Length*arm0Length+arm1Length*arm1Length-
						  dist*dist)/
						(2.*arm1Length*arm0Length));
	
	//on ajoute l'angle de base à angle1
	angle1 += acos(baseArmLength/dist);

	res[1]=angle1;res[2]=angle2;

	return res;
}

Position Robot::Catch(Position currentPos, Point obj){
	Position res;
	res[3]=currentPos[3];res[4]=currentPos[4];

	double distZ = obj[Z]-currentPos[ROBOT_Z];
	double distX = obj[X]-currentPos[ROBOT_X];
	double angle = atan(distZ/distX);
	double dist = sqrt(distX*distX+distZ*distZ);
	return SetAngles(dist,res);

}

Position Robot::RandomCatch(Point p)
{
	double length = (maxDist()-minSpace)*(double(rand())/double(RAND_MAX))+minSpace;
	double angle = 2.*Pi*(double(rand())/double(RAND_MAX));

	double dist = sqrt(length*length+baseArmLength*baseArmLength);

	double x = p[X]-cos(angle)*length;
	double z = p[Z]-sin(angle)*length;
	Position res;
	res[0]=angle;res[3]=x;res[4]=z;

	return SetAngles(dist,res);
}

Robot::Robot(Point s): 
	baseSize(s), 
	yPos(0.25), 
	armWidth(0.1), 
	arm0Length(1),
	arm1Length(1),
	minSpace(1),
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

	baseArmY = yPos + baseSize[1]/2 + baseArmLength/2;
	baseHeight = baseArmY + baseArmLength/2;
}

void Robot::printSize() const
{
	printf("robot size : %lf %lf %lf \n",baseSize[0],baseSize[1],baseSize[2]);
}

std::array<Box, 2> Robot::GetBaseBoxes(const Position& pos) const
{
	std::array<double, 3> center0 = { pos[ROBOT_X], yPos, pos[ROBOT_Z] };
	std::array<double, 3> center1 = { pos[ROBOT_X], baseArmY, pos[ROBOT_Z] };
	std::array<double, 3> size1 = { armWidth, baseArmLength, armWidth };
	Matrix rotation = Matrix::Rotate(pos[ROBOT_ROT], Y);
	
	std::array<Box, 2> out = { Box(center0, baseSize, rotation), Box(center1, size1, rotation) };
	return out;
}

std::array<Box, 2> Robot::GetArmsBoxes(const Position& pos) const
{
	std::array<double, 3> center0 = { 0, arm0Length/2, 0 };
	std::array<double, 3> center1 = { 0, arm1Length/2, 0 };

	std::array<double, 3> size0 = { armWidth, arm0Length, armWidth };
	std::array<double, 3> size1 = { armWidth, arm1Length, armWidth };

	Matrix rotation0 = Matrix::Rotate(pos[ROBOT_ROT], Y)*Matrix::Rotate(pos[ROBOT_ARM0], Z);
	Matrix rotation1 = rotation0*Matrix::Rotate(pos[ROBOT_ARM1], Z);

	center0 = rotation0*center0;
	center1 = rotation1*center1 + 2.*center0;

	std::array<double, 3> baseArmTop = { pos[ROBOT_X], baseHeight, pos[ROBOT_Z] };
	center0 += baseArmTop;
	center1 += baseArmTop;

	std::array<Box, 2> out = { Box(center0, size0, rotation0), Box(center1, size1, rotation1) };
	return out;
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