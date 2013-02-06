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

		if(n[0] != 0 || n[1] != 1 || n[2] != 0 || p[0][1] != robotBottom)
			baseScene.push_back(t);
	}
}


bool Scene::Collision(const Position& pos) const
{
	std::array<Box, 2> baseBoxes = robot.GetBaseBoxes(pos);
	std::array<Box, 2> armsBoxes = robot.GetArmsBoxes(pos);

	Point ballPos;
	ballPos[X] = pos[BALL_X];
	ballPos[Y] = pos[BALL_Y];
	ballPos[Z] = pos[BALL_Z];

	return RobotCollision(baseBoxes, armsBoxes, ballPos, true)
		|| GroundCollision(baseBoxes, armsBoxes, ballPos, true);
}

// Test the intersection of the robot with himself
bool Scene::RobotCollision(const std::array<Box, 2>& baseBoxes, 
						   const std::array<Box, 2>& armsBoxes,
						   const Point& ballPos,
						   const bool testBallArm1) const
{
	return baseBoxes[0].Intersect(armsBoxes[0]) 
		|| baseBoxes[0].Intersect(armsBoxes[1]) 
		|| baseBoxes[1].Intersect(armsBoxes[1])
		|| baseBoxes[0].Intersect(ballPos, ballRadius)
		|| baseBoxes[1].Intersect(ballPos, ballRadius)
		|| armsBoxes[0].Intersect(ballPos, ballRadius)
		|| (testBallArm1 && armsBoxes[1].Intersect(ballPos, ballRadius));
}


bool Scene::GroundCollision(const std::array<Box, 2>& baseBoxes, 
							const std::array<Box, 2>& armsBoxes,
							const Point& ballPos,
							const bool testBall) const
{
	for(const auto& t : baseScene)
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
		for(int i=0; i<staticScene.triangles.size(); i++)
			if(IntersectTriangleSphere(i, ballPos))
				return true;

	return false;
}


bool Scene::IntersectTriangleSphere(const int triangle, const Point& sphereCenter) const
{
	Point p[3], e[3];
	double radius2 = ballRadius*ballRadius;

	for(int i=0; i<3; i++)
	{
		p[i] = staticScene.points[staticScene.triangles[triangle][i]] - sphereCenter;

		if( (p[i]|p[i])  < radius2)
			return true;
	}
                    
	for(int i=0; i<3; i++)
	{
		int i2 = (i+1)%3;
		e[i] = p[i] - p[i2];
		double n2 = e[i].Norm2();
		double d = -(p[i] | e[i]);
		if(d > 0 && d < n2 && p[i].Norm2() - d*d/n2 < radius2 )
			return true;
	}

	Point n = e[0]^e[1];
	n = n.Normalize();
	double d = (p[0]|n);
	if( abs(d) >= ballRadius)
		return false;

	n = d*n;

	int sign01 = Box::SignMask(e[0]^(p[0] - n)); 
	int sign12 = Box::SignMask(e[1]^(p[1] - n));
	int sign20 = Box::SignMask(e[2]^(p[2] - n));            
	return (sign01 & sign12 & sign20) != 0;
}


////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////				TODO				////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

Point Scene::Drop(Position p)
{
	return Point();
}

bool Scene::validMove(Position a, Position b, bool with, Point* object)
{
	double length = Position((b-a)).Norm();
	
	if(length<=0.01)
		return true;

	Position mid = Position((a+b))/2.;
	if(Collision(mid))
		return false;

	return validMove(a, mid, with, object) && validMove(mid, b, with, object);
}


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


		if(validMove(start,end,false,NULL)){
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
