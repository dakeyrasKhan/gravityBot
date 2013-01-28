#pragma once
#include <algorithm>
#include <ozcollide/aabbtree_poly.h>
#include "Vector.hpp"
#include "Object.hpp"
#include "Robot.hpp"

class Scene
{
public:
	Scene(const char* sceneFile);
	~Scene();

	bool Collision(Position position, bool, Point*);

	std::array<double, DIM_CONF> size;
	double maxSize(){ return *std::max_element(size.begin(),size.end()); }


	bool validMove(Position, Position, bool, Point*);


	Point Drop(Position);
	const Object& StaticScene() const { return staticScene; };

private:
	void ReadObjFile(const char* fileName);
	void BuildCollisionTree();

	Object staticScene;
	Robot robot;
	const double robotY;

	ozcollide::AABBTreePoly* collisionTree;
};

