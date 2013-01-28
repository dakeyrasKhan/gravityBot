#pragma once
#include <vector>
#include <queue>
#include <algorithm>
#include <array>
#include <ozcollide/ozcollide.h>
#include <ozcollide/aabbtree_poly.h>
#include "octree.hpp"
#include "path.hpp"
#include "Vector.hpp"


class Scene
{
public:
	Scene(const char* sceneFile);
	~Scene();
	const std::vector<Point> Points() const { return points; };
	const std::vector<std::array<int, 3>> Triangles() const { return triangles; };

	std::array<double, DIM_CONF> size;
	double maxSize(){ return *std::max_element(size.begin(),size.end()); }
	bool collision(Position);
	bool validMove(Position, Position);

private:
	void ReadObjFile(const char* fileName);
	void BuildCollisionTree();

	std::vector<Point> points;
	std::vector<std::array<int, 3>> triangles;

	ozcollide::AABBTreePoly* collisionTree;
};

