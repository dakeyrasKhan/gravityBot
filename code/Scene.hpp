#pragma once
#include <algorithm>
#include <ozcollide/aabbtree_poly.h>
#include "Vector.hpp"
#include "Object.hpp"
#include "Robot.hpp"

#define NB_TRY 1000

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

	std::vector<Position> Optimize(std::vector<Position> p);
	bool validMove(Position, Position, bool, Point*,bool p=false);
	Point Drop(Position);

	Position posSize, negSize;
	double maxSize;

	Robot robot;

private:
	void ReadObjFile(const char* fileName);
	void BuildCollisionTree();

	Object staticScene;
	
	const double robotY;

	ozcollide::AABBTreePoly* collisionTree;


};

