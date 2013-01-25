#pragma once
#include "octree.hpp"
#include "Scene.h"
#include <queue>
#include <stack>
#define NB_WAYPOINTS 1000*1000
#define NB_NEIGHB 100

class Roadmap{
public:
	Roadmap(Scene* scene);
	Path getPath(Position,Position);
private:
	Scene* scene;
	Octree tree;
	vector<Position> waypoints;
	vector<vector<Node>> adjacency;
	vector<double> failRate;
};