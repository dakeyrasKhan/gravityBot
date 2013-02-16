#include <iostream>
#include <fstream>
#include "Scene.hpp"

Scene::Scene(const char* sceneFile, const Point& robotSize) : 
	robot(robotSize), ballRadius(0.2)
{
	ReadObjFile(sceneFile);
	BuildBaseScene();
}


void Scene::ReadObjFile(const char* fileName)
{
	posSize[ROBOT_ROT] = 2*Pi;
	posSize[ROBOT_ARM0] = posSize[ROBOT_ARM1] = Pi/2.; 
	negSize[ROBOT_ROT] = 0;
	negSize[ROBOT_ARM0] = negSize[ROBOT_ARM1] = -Pi/2.;
	posSize[ROBOT_X] = posSize[ROBOT_Z] = negSize[ROBOT_X] = negSize[ROBOT_Z] = 0;

	std::ifstream sceneFile(fileName);
	while(!sceneFile.eof())
	{
		Point p;
		std::array<int, 3> triangle;

		switch(sceneFile.get())
		{
		case 'v':
			sceneFile >> p[0] >> p[1] >> p[2];
			staticScene.points.push_back(p);

			if(p[0] > posSize[ROBOT_X])
				posSize[ROBOT_X] = p[0];
			else if(p[0] < negSize[ROBOT_X])
				negSize[ROBOT_Z] = p[0];

			if(p[2] > posSize[ROBOT_Z])
				posSize[ROBOT_Z] = p[2];
			else if(p[2] < negSize[ROBOT_Z])
				negSize[ROBOT_Z] = p[2];
			break;
		case 'f':
			sceneFile >> triangle[0] >> triangle[1] >> triangle[2];
			triangle -= 1;
			staticScene.triangles.push_back(triangle);
			break;
		case '\n':
			break;
		default:
			sceneFile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}
	}
	maxSize=0;
	for(auto x : posSize)
		maxSize = std::max(maxSize, x);
	for(auto x : negSize)
		maxSize = std::max(maxSize, -x);
}


void Scene::BuildBaseScene()
{
	// Build the polygons vector
	double robotBottom = robot.yPos - robot.baseSize[1]/2;
	for(auto& t : staticScene.triangles)
	{
		Point p[3] = { staticScene.points[t[0]], staticScene.points[t[1]], staticScene.points[t[2]] };
		Point n = ((p[1]-p[0])^(p[2]-p[0])).Normalize();;
		Triangle trig(p[0], p[1], p[2]);

		if(n[0] == 0 && n[1] == 1 && n[2] == 0)
		{
			groundTriangles.push_back(trig);

			if(p[0][1] != robotBottom)
				noBaseGroundTriangles.push_back(t);
		}
		else
			noBaseGroundTriangles.push_back(t);

		triangles.push_back(trig);
	}
}


bool Scene::Collision(const Position& pos, const int ballStatus) const
{
	std::array<Box, 2> baseBoxes = robot.GetBaseBoxes(pos);
	std::array<Box, 2> armsBoxes = robot.GetArmsBoxes(pos);

	Point ballPos;
	ballPos[X] = pos[BALL_X];
	ballPos[Y] = pos[BALL_Y];
	ballPos[Z] = pos[BALL_Z];

	return RobotCollision(baseBoxes, armsBoxes, ballPos, (ballStatus & 0x1) == 0)
		|| GroundCollision(baseBoxes, armsBoxes, ballPos, (ballStatus & 0x2) == 0);
}

// Test the intersection of the robot with himself
bool Scene::RobotCollision(const std::array<Box, 2>& baseBoxes, 
						   const std::array<Box, 2>& armsBoxes,
						   const Point& ballPos,
						   const bool testBallArm1) const
{
	return baseBoxes[0].IntersectBox(armsBoxes[0]) 
		|| baseBoxes[0].IntersectBox(armsBoxes[1]) 
		|| baseBoxes[1].IntersectBox(armsBoxes[1])
		|| baseBoxes[0].IntersectSphere(ballPos, ballRadius)
		|| baseBoxes[1].IntersectSphere(ballPos, ballRadius)
		|| armsBoxes[0].IntersectSphere(ballPos, ballRadius)
		|| (testBallArm1 && armsBoxes[1].IntersectSphere(ballPos, ballRadius));
}


bool Scene::GroundCollision(const std::array<Box, 2>& baseBoxes, 
							const std::array<Box, 2>& armsBoxes,
							const Point& ballPos,
							const bool testBall) const
{
	for(const auto& t : noBaseGroundTriangles)
		for(const auto& b : baseBoxes)
			if(b.Intersect(
				staticScene.points[t[0]], 
				staticScene.points[t[1]], 
				staticScene.points[t[2]]))
					return true;

	for(const auto& t : staticScene.triangles)
		for(const auto& b : armsBoxes)
			if(b.Intersect(
				staticScene.points[t[0]], 
				staticScene.points[t[1]], 
				staticScene.points[t[2]]))
					return true;

	if(testBall)
		for(auto t : staticScene.triangles)
			if(Triangle(staticScene.points[t[0]], 
				staticScene.points[t[1]],
				staticScene.points[t[2]]).IntersectSphere(ballPos, ballRadius))
				return true;

	return false;
}



Point Scene::Drop(Position p)
{
	double heigth = -std::numeric_limits<double>::infinity();

	// Find a candidate
	Point hit;
	hit[X] = p[BALL_X];
	hit[Z] = p[BALL_Z];

	for(auto t : groundTriangles)
	{
		double h = t[0][Y];

		if(h > heigth && h < p[BALL_Y])
		{
			hit[Y] = h;
			if(t.IntersectPoint(hit))
				heigth = h;
		}
	}

	// Check if we have found a candidate
	if(heigth == -std::numeric_limits<double>::infinity())
		throw NoDropPointException();

	hit[Y] = heigth + ballRadius;

	// Check that the candidate is valid
	Point start = hit;
	start[Y] = p[BALL_Y];

	/*
	for(auto t : triangles)
		if(t.IntersectCylinder(start, hit, ballRadius))
			throw NoDropPointException();

	for(auto b : robot.GetBaseBoxes(p))
		if(b.IntersectCylinder(hit, start, ballRadius))
			throw NoDropPointException();

	for(auto b : robot.GetArmsBoxes(p))
		if(b.IntersectCylinder(hit, start, ballRadius))
			throw NoDropPointException();
	*/

	return hit;
}


bool Scene::ValidMove(const Position& a, const Position& b, const int ballStatus) const
{
	double length = Position((b-a)).Norm();
	
	if(length<=0.1)
		return true;

	Position mid = Position((a+b))/2.;
	if((ballStatus & BALL_ON_ARM) != 0)
		mid = robot.CorrectBallPos(mid);

	if(Collision(mid))
		return false;

	return ValidMove(a, mid, ballStatus) && ValidMove(mid, b, ballStatus);
}


////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////				TODO				////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////



std::vector<Position> Scene::Optimize(std::vector<Position> path)
{
	int optimization=0;
	std::vector<Position> retour = path;
	for(int essai=0;essai<NB_TRY;essai++){
		int a = rand()%retour.size()-1;
		int b = rand()%retour.size()-1;
		if(a==b)
			continue;
		if(a>b){
			int c=a;
			a=b;
			b=c;
		}

		double c= double(rand())/double(RAND_MAX);
		double d= double(rand())/double(RAND_MAX);

		Position start = retour[a]+(retour[a+1]-retour[a])*c;
		Position end = retour[b]+(retour[b+1]-retour[b])*d;


		if(ValidMove(start,end)){
			retour.erase(retour.begin()+a+1,retour.begin()+b+1);
			retour.insert(retour.begin()+a+1,end);
			retour.insert(retour.begin()+a+1,start);
			optimization++;
			if(optimization>20)
				break;
		}
	}
	return retour;
}
