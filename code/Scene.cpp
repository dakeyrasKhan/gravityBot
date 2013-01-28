#include <iostream>
#include <fstream>
#include <ozcollide/aabbtreepoly_builder.h>
#include "Scene.h"

bool Scene::validMove(Position a, Position b)
{
	double length=Position((b-a)).Norm();
	if(length<=EPS)
		return true;

	Position mid = Position((a+b))/2.;
	if(Collision(mid))
		return false;
	return validMove(a,mid) && validMove(mid,b);
}


Scene::Scene(const char* sceneFileName) : collisionTree(nullptr), robotY(0)
{
	ReadObjFile(sceneFileName);

	robot.xSide = 2;
	robot.ySide = 1;
	robot.zSide = 3;

}


Scene::~Scene()
{
	if(collisionTree != nullptr)
		collisionTree->destroy();
}


void Scene::ReadObjFile(const char* fileName)
{
	std::ifstream sceneFile(fileName);
	while(!sceneFile.eof())
	{
		Point p;
		std::array<int, 3> triangle;

		switch(sceneFile.get())
		{
		case 'v':
			sceneFile >> p[0] >> p[1] >> p[2];
			points.push_back(p);
			break;
		case 'f':
			sceneFile >> triangle[0] >> triangle[1] >> triangle[2];
			triangle -= 1;
			triangles.push_back(triangle);
			break;
		case '\n':
			break;
		default:
			sceneFile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}
	}
}


void Scene::BuildCollisionTree()
{
	ozcollide::AABBTreePolyBuilder builder;

	// Build the polygons vector
	std::vector<ozcollide::Polygon> polygons;
	for(auto t : triangles)
	{
		Point p[3] = { points[t[0]], points[t[1]], points[t[2]] };
		Point n = ((p[1]-p[0])^(p[2]-p[0])).Normalize();

		ozcollide::Polygon poly;
		poly.setNormal(ozcollide::Vec3f(n[0], n[1], n[2]));
		poly.setIndicesMemory(3, t.data());

	}

	// Build the vertices vector
	std::vector<ozcollide::Vec3f> vertices;
	for(int i=0; i<points.size(); i++)
		vertices.push_back(ozcollide::Vec3f(points[i][0], points[i][1], points[i][2]));

	collisionTree = builder.buildFromPolys(polygons.data(), polygons.size(), vertices.data(), vertices.size());
}


bool Scene::Collision(Position pos)
{
	ozcollide::Matrix3x3 robotRotation;
	robotRotation.identity();
	robotRotation.m_[0][0] = cos(pos[ROBOT_ROT]);
	robotRotation.m_[2][0] = sin(pos[ROBOT_ROT]);
	robotRotation.m_[0][2] = -sin(pos[ROBOT_ROT]);
	robotRotation.m_[2][2] = cos(pos[ROBOT_ROT]);

	ozcollide::OBB robotBox;
	robotBox.extent = ozcollide::Vec3f(robot.xSide, robot.ySide, robot.zSide);
	robotBox.center = ozcollide::Vec3f(pos[ROBOT_X], robotY, pos[ROBOT_Z]);
	robotBox.matrix = robotRotation;

	ozcollide::AABBTreePoly::OBBColResult result;
	collisionTree->collideWithOBB(robotBox, result);

	return result.polys_.size() == 0;
}