#pragma once
#include "octree.hpp"
#include "Scene.hpp"
#include "path.hpp"
#include <queue>
#include <stack>
#define NB_WAYPOINTS 100
#define NB_DROP 1

class Roadmap{
public:
	Roadmap(Scene*);
	void explore(int,int);
	Path getPath(FullNode,FullNode,Point*);

private:
	void addNode(FullNode,int);
	Scene* scene;
	Octree tree;
	int nbClasses;
	vector<int> classes;
	vector<FullNode> waypoints;
	vector<vector<FullNode>> adjacency;
	vector<double> failRate;
	vector<Path> drop;
};