#include <iostream>
#include <fstream>
#include <ozcollide/aabbtreepoly_builder.h>
#include "Scene.h"

Point Scene::Drop(Position p){
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


Scene::Scene(const char* sceneFileName) : collisionTree(nullptr)
{
	ReadObjFile(sceneFileName);
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


bool Scene::Collision(Position pos,bool with,Point* object)
{
	return false;
}