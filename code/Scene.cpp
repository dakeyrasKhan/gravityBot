#include <iostream>
#include <fstream>
#include <ozcollide/aabbtreepoly_builder.h>
#include "Scene.hpp"


Scene::Scene(const char* sceneFileName) : collisionTree(nullptr), robotY(ROBOT_Y)
{
	Point robotSize;
	robotSize[0] = 1; robotSize[1] = .5; robotSize[2] = 1.5;
	robot = Robot(robotSize);

	ReadObjFile(sceneFileName);
	BuildCollisionTree();
}


Scene::~Scene()
{
	if(collisionTree != nullptr)
		collisionTree->destroy();
}


void Scene::ReadObjFile(const char* fileName)
{
	posSize[ROBOT_ROT] = 2*Pi;
	posSize[ROBOT_X] = 0;
	posSize[ROBOT_Z] = 0;
	negSize[ROBOT_ROT] = 0;
	negSize[ROBOT_X] = 0;
	negSize[ROBOT_Z] = 0;

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
				negSize[ROBOT_Y] = p[0];

			if(p[2] > posSize[ROBOT_Y])
				posSize[ROBOT_Y] = p[2];
			else if(p[2] < negSize[ROBOT_Y])
				negSize[ROBOT_Y] = p[2];
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

	for(auto x : posSize)
		maxSize = std::max(maxSize, x);
	for(auto x : negSize)
		maxSize = std::max(maxSize, -x);
}


void Scene::BuildCollisionTree()
{
	ozcollide::AABBTreePolyBuilder builder;

	// Build the polygons vector
	std::vector<ozcollide::Polygon> polygons;
	for(auto& t : staticScene.triangles)
	{
		Point p[3] = { staticScene.points[t[0]], staticScene.points[t[1]], staticScene.points[t[2]] };
		Point n = ((p[1]-p[0])^(p[2]-p[0])).Normalize();

		if(n[0] == 0 && n[1] == 1 && n[2] == 0 && p[0][1] == ROBOT_Y - ROBOT_HEIGHT/2)
			continue;

		ozcollide::Polygon poly;
		poly.setNormal(ozcollide::Vec3f(n[0], n[1], n[2]));
		poly.setIndicesMemory(3, t.data());
		polygons.push_back(poly);

	}

	// Build the vertices vector
	std::vector<ozcollide::Vec3f> vertices;
	for(int i=0; i<staticScene.points.size(); i++)
		vertices.push_back(ozcollide::Vec3f(staticScene.points[i][0], staticScene.points[i][1], staticScene.points[i][2]));

	collisionTree = builder.buildFromPolys(polygons.data(), polygons.size(), vertices.data(), vertices.size());
}


bool Scene::Collision(Position pos, bool with, Point* object)
{
	ozcollide::Matrix3x3 robotRotation;
	robotRotation.identity();
	robotRotation.m_[0][0] = cos(pos[ROBOT_ROT]);
	robotRotation.m_[2][0] = sin(pos[ROBOT_ROT]);
	robotRotation.m_[0][2] = -sin(pos[ROBOT_ROT]);
	robotRotation.m_[2][2] = cos(pos[ROBOT_ROT]);

	ozcollide::OBB robotBox = robot.GetBox();
	robotBox.center = ozcollide::Vec3f(pos[ROBOT_X], robotY, pos[ROBOT_Z]);
	robotBox.matrix = robotRotation;

	ozcollide::AABBTreePoly::OBBColResult result;
	collisionTree->collideWithOBB(robotBox, result);

	return result.polys_.size() > 0;
}


Object Scene::RobotObject(Position pos) const
{
	Object obj = robot.GetObject();
	Point y; y[0] = 0; y[1] = 1; y[2] = 0;
	obj.Rotate(y, pos[ROBOT_ROT]);
	obj.Translate(pos[ROBOT_X], ROBOT_Y, pos[ROBOT_Z]);
	return obj;
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
	double length=Position((b-a)).Norm();
	if(length<=EPS)
		return true;

	Position mid = Position((a+b))/2.;
	if(Collision(mid,with,object))
		return false;
	return validMove(a,mid,with,object) && validMove(mid,b,with,object);
}
