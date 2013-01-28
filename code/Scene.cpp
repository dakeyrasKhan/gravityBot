#include "Scene.h"
#include <iostream>
#include <fstream>

bool Scene::collision(Position pos)
{
	return false;
}

bool Scene::validMove(Position a, Position b)
{
	double length=Position((b-a)).Norm();
	if(length<=EPS)
		return true;

	Position mid = Position((a+b))/2.;
	if(collision(mid))
		return false;
	return validMove(a,mid) && validMove(mid,b);
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

}