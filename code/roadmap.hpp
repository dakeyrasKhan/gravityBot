#pragma once
#include "octree.hpp"
#include "Scene.h"
#define NB_WAYPOINTS 1000*1000
#define NB_NEIGHB 100

class Roadmap{
public:
	Roadmap(Scene* scene);
private:
	Scene* scene;
	Octree tree;
	vector<Position> waypoints;
	vector<vector<int>> adjacency;
	vector<double> failRate;
};