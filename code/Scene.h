#pragma once
#include <vector>
#include <queue>
#include <algorithm>
#include <ozcollide/aabbtree_poly.h>
#include "octree.hpp"
#include "path.hpp"
#include "Vector.hpp"


struct Robot
{
	double xSide;
	double ySide;
	double zSide;
};

class Scene
{
public:
	Scene(const char* sceneFile);
	~Scene();

	const std::vector<Point> Points() const { return points; };
	const std::vector<std::array<int, 3>> Triangles() const { return triangles; };
	const double robotY;
	bool Collision(Position position);

	std::array<double, DIM_CONF> size;
	double maxSize(){ return *std::max_element(size.begin(),size.end()); }
	bool validMove(Position, Position);

private:
	void ReadObjFile(const char* fileName);
	void BuildCollisionTree();

	std::vector<Point> points;
	std::vector<std::array<int, 3>> triangles;
	Robot robot;

	ozcollide::AABBTreePoly* collisionTree;
};

