#pragma once
#include "octree.hpp"
#include "Scene.h"
#include <queue>
#include <stack>
#define NB_WAYPOINTS 1000*1000
#define NB_NEIGHB 100

class Roadmap{
public:
	Roadmap(Scene*);
	void explore(int,int);
	Path getPath(Position,Position);
private:
	Scene* scene;
	Octree tree;
	vector<int> classes;
	vector<Position> waypoints;
	vector<vector<Node>> adjacency;
	vector<double> failRate;
};

class FullRoadmap{
public:
	FullRoadmap(Scene* scene):with(scene),without(scene){};
	//void connexify();
private:
	Roadmap with;
	Roadmap without;
};