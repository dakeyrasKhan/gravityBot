#include <iostream>
#include <fstream>
#include <ozcollide/aabbtreepoly_builder.h>
#include "Scene.hpp"

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


Scene::Scene(const char* sceneFile, const Point& robotSize) : 
	collisionTreeBase(nullptr),
	robot(robotSize)
{
	ReadObjFile(sceneFile);
	BuildCollisionTree();
}


Scene::~Scene()
{
	if(collisionTreeBase != nullptr)
		collisionTreeBase->destroy();
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


void Scene::BuildCollisionTree()
{
	ozcollide::AABBTreePolyBuilder builder;

	// Build the polygons vector
	ozcollide::Vector<ozcollide::Polygon> polygonsBase;
	double robotBottom = robot.yPos - robot.baseSize[1]/2;
	for(auto& t : staticScene.triangles)
	{
		Point p[3] = { staticScene.points[t[0]], staticScene.points[t[1]], staticScene.points[t[2]] };
		Point n = ((p[1]-p[0])^(p[2]-p[0])).Normalize();

		ozcollide::Polygon poly;
		poly.setNormal(ozcollide::Vec3f(n[0], n[1], n[2]));
		poly.setIndicesMemory(3, t.data());

		if(n[0] != 0 || n[1] != 1 || n[2] != 0 || p[0][1] != robotBottom)
			polygonsBase.add(poly);

	}

	// Build the vertices vector
	ozcollide::Vector<ozcollide::Vec3f> vertices;
	for(int i=0; i<staticScene.points.size(); i++)
		vertices.add(ozcollide::Vec3f(staticScene.points[i][0], 
			staticScene.points[i][1], staticScene.points[i][2]));

	collisionTreeBase = builder.buildFromPolys(
		polygonsBase.mem(), polygonsBase.size(), 
		vertices.mem(), vertices.size());
}


bool Scene::Collision(Position pos, bool with, Point* object)
{
	std::array<Box, 2> baseBoxes = robot.GetBaseBoxes(pos);
	std::array<Box, 2> armsBoxes = robot.GetArmsBoxes(pos);

	return false
		|| collisionTreeBase->isCollideWithOBB(baseBoxes[0].ToOzcollide())
		|| collisionTreeBase->isCollideWithOBB(baseBoxes[1].ToOzcollide())
		|| collisionTreeBase->isCollideWithOBB(armsBoxes[0].ToOzcollide())
		|| collisionTreeBase->isCollideWithOBB(armsBoxes[1].ToOzcollide())
		;
}


////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////				TODO				////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

Point Scene::Drop(Position p)
{
	return Point();
}

bool Scene::validMove(Position a, Position b, bool with, Point* object,bool print)
{
	double length=Position((b-a)).Norm();
	if(print){
		std::cout<<"length : "<<length<<" eps : "<<EPS<<std::endl;
		for(auto p : a)
			std::cout<<"---"<<p<<std::endl;
		for(auto p : b)
			std::cout<<"---"<<p<<std::endl;
	}
	
	//std::cout<<"length : "<<length<<" eps : "<<EPS<<std::endl;
	if(length<=0.01){
		//std::cout<<"return"<<std::endl;
		return true;
	}
	//std::cout<<"continue"<<std::endl;
	Position mid = Position((a+b))/2.;
	if(Collision(mid,with,object))
		return false;
	if(print)
		std::cout<<"continue"<<std::endl;
	return validMove(a,mid,with,object,print) && validMove(mid,b,with,object,print);
}
