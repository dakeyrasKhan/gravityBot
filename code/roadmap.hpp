#pragma once
#include "octree.hpp"
#include "Scene.hpp"
#include "path.hpp"
#include <queue>
#include <stack>
#define NB_WAYPOINTS 500
#define NB_NEIGHB 10
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