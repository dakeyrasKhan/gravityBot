#pragma once
#include "octree.hpp"
#include "Scene.h"
#include <queue>
#include <stack>
#define NB_WAYPOINTS 10000
#define NB_NEIGHB 50
#define NB_DROP 5

class Roadmap{
public:
	Roadmap(Scene*);
	void explore(int,int);
	Path getPath(Position,Position,bool,Point*);

private:
	void addNode(FullNode);
	Scene* scene;
	Octree tree;
	int nbClasses;
	vector<int> classes;
	vector<FullNode> waypoints;
	vector<vector<FullNode>> adjacency;
	vector<double> failRate;
	vector<Path> drop;
};