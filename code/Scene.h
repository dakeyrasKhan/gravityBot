#pragma once
#include <vector>
#include <algorithm>
#include <array>
#include "octree.hpp"
#include "Vector.hpp"

class Scene
{
public:
	Scene();
	std::vector<Point> points;
	std::vector<std::array<int, 3>> triangles;
	std::array<double, DIM_CONF> size;
	double maxSize(){ return *std::max_element(size.begin(),size.end()); }
	bool collision(Position);
	bool validMove(Position, Position);
};

