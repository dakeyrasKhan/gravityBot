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
	const Object& StaticScene() const { return staticScene; };
	Object RobotObject(Position) const;

	Position GetPosSize() const { return posSize; };
	Position GetNegSize() const { return negSize; };
	double GetMaxSize() const { return maxSize; };


	bool validMove(Position, Position, bool, Point*);
	Point Drop(Position);

private:
	void ReadObjFile(const char* fileName);
	void BuildCollisionTree();

	Object staticScene;
	Robot robot;
	const double robotY;

	ozcollide::AABBTreePoly* collisionTree;

	Position posSize, negSize;
	double maxSize;
};

