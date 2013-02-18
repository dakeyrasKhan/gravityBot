#include "Robot.hpp"
#include <cstdio>

Position Robot::SetAngles(double y,double dist,Position p){
	Position res = p;
	//Al-Kashi
	double angle1 = acos((dist*dist+arm0Length*arm0Length-
						  arm1Length*arm1Length)/
						(2.*dist*arm0Length));

	double angle2 = acos((arm0Length*arm0Length+arm1Length*arm1Length-
						  dist*dist)/
						(2.*arm1Length*arm0Length));
	
	//on ajoute l'angle de base à angle1
	angle1 += asin(y/dist)+Pi/2.;

	res[1]=Pi-angle1;res[2]=Pi-angle2;
	//res[1]=Pi-acos(baseArmY/dist);res[2]=0;
	return res;
}

Position Robot::Catch(Position currentPos, Point obj){
	Position res;
	res[3]=currentPos[3];res[4]=currentPos[4];

	double distZ = obj[Z]-currentPos[ROBOT_Z];
	double distX = obj[X]-currentPos[ROBOT_X];
	double hyp = sqrt(distZ*distZ+distX*distX);
	double angle = acos(distZ/hyp);
	if(distX<0)
		angle=-angle;
	double dist = sqrt(distX*distX+distZ*distZ);
	res[ROBOT_ROT]=3.*Pi/2.-angle;
	res[BALL_X]=obj[X];res[BALL_Y]=obj[Y];res[BALL_Z]=obj[Z];

	double y = obj[Y]-baseArmY;
	return SetAngles(y,sqrt(dist*dist+y*y),res);

}

Position Robot::RandomCatch(Point p)
{
	double y = p[Y]-baseArmY;

	double length = (maxDist(y)-minSpace)*(double(rand())/double(RAND_MAX))+minSpace;
	double angle = 2.*Pi*(double(rand())/double(RAND_MAX));

	double x = p[X]+cos(angle)*length;
	double z = p[Z]+sin(angle)*length;

	double dist = length*length + y*y;

	Position res;
	res[0]=angle;res[3]=x;res[4]=z;
	res[BALL_X]=p[X];res[BALL_Y]=p[Y];res[BALL_Z]=p[Z];
	
	
	return SetAngles(y,sqrt(dist),res);
}


Position Robot::RandomDrop(Point p)
{
	double newY = (maxY()-minY)*(double(rand())/double(RAND_MAX))+minY;
	Point n = p;
	n[Y] = newY;
	return RandomCatch(n);
}

Robot::Robot(Point s): 
	baseSize(s), 
	yPos(0.25), 
	armWidth(0.1), 
	arm0Length(1),
	arm1Length(1.5),
	minSpace(1),
	baseArmLength(0.2),
	minY(0.2)
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

Position Robot::CorrectBallPos(Position pos) const
{
	double dist = sin(pos[ROBOT_ARM0])*arm0Length+
				  sin(pos[ROBOT_ARM0]+pos[ROBOT_ARM1])*arm1Length;

	pos[BALL_X]=-cos(pos[ROBOT_ROT])*dist+pos[ROBOT_X];
	pos[BALL_Z]=-sin(pos[ROBOT_ROT])*dist+pos[ROBOT_Z];

	double h = baseArmY+cos(pos[ROBOT_ARM0])*arm0Length+
   			   cos(pos[ROBOT_ARM0]+pos[ROBOT_ARM1])*arm1Length+baseArmLength/2;

	pos[BALL_Y] = h;
	return pos;
}

std::array<Box, 4> Robot::GetBoundingBoxes(const Position& start, const Position& end) const
{
	std::array<Box, 2> baseBoxesStart = GetBaseBoxes(start);
	std::array<Box, 2> baseBoxesEnd	  = GetBaseBoxes(end);

	std::array<Box, 2> armsBoxesStart = GetArmsBoxes(start);
	std::array<Box, 2> armsBoxesEnd	  = GetArmsBoxes(end);

	std::array<Box, 4> out;

	Point sizeInflate = baseBoxesStart[0].Center() - baseBoxesEnd[0].Center();

	for(int i=0; i<2; i++)
	{
		Box boundingBox = Box::GetRotationBoundingBox(baseBoxesStart[i].Size(), 
			baseBoxesStart[i].Rotation(), baseBoxesEnd[i].Rotation());

		out[i] = Box((baseBoxesStart[i].Center() + baseBoxesEnd[i].Center())/2, 
			boundingBox.Size() + abs(sizeInflate*boundingBox.Rotation()), 
			boundingBox.Rotation());
	}

	double alpha;
	Box boundingBox = Box::GetRotationBoundingBox(armsBoxesStart[0].Size(), 
		armsBoxesStart[0].Rotation(), armsBoxesEnd[0].Rotation(), arm0Length/2, &alpha);

	out[2] = Box((armsBoxesStart[0].Center() + armsBoxesEnd[0].Center())/2,
		boundingBox.Size() + abs(sizeInflate*boundingBox.Rotation()),
		boundingBox.Rotation());
	
	boundingBox = Box::GetRotationBoundingBox(armsBoxesStart[1].Size(), 
		armsBoxesStart[1].Rotation(), armsBoxesEnd[1].Rotation(), arm1Length/2);

	// Can be improved a lot
	out[3] = Box((armsBoxesStart[1].Center() + armsBoxesEnd[1].Center())/2,
		boundingBox.Size() + abs(sizeInflate*boundingBox.Rotation()) + 
		2.5*(arm0Length + arm1Length)*alpha,
		boundingBox.Rotation());
	return out;

}