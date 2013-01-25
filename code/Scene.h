#pragma once
#include <vector>
#include <array>
#include "Vector.hpp"

class Scene
{
public:
	Scene();
	std::vector<Point> points;
	std::vector<std::array<int, 3>> triangles;
};

