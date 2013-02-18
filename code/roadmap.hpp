#pragma once
#include "octree.hpp"
#include "Scene.hpp"
#include "path.hpp"
#include <queue>
#include <stack>
#define NB_WAYPOINTS 200
#define NB_DROP 1

class Roadmap{
public:
	Roadmap(Scene*);
	void explore(int,int);
	Path getPath(FullNode,FullNode,Point*);
	Path OkCatch(Position start,Position end,Point* ball);
private:
	void addNode(FullNode,int);
	Scene* scene;
	Octree tree;
	int nbClasses;
	bool isDrop[2*NB_WAYPOINTS+1][2*NB_WAYPOINTS+1];
	vector<int> classes;
	vector<FullNode> waypoints;
	vector<vector<FullNode>> adjacency;
	vector<double> failRate;
	vector<Path> drop;
};