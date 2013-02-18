#pragma once
#include "QuadTree.hpp"
#include "Scene.hpp"
#include "path.hpp"
#include <queue>
#include <stack>

#define NB_WAYPOINTS 600

#define NB_DROP 1

struct Adj{
	FullNode node;
	Path drop;
	Adj(){};
	Adj(FullNode n):node(n){};
	Adj(FullNode n,Path p):node(n),drop(p){};
};

class Roadmap{
public:
	Roadmap(){};
	Roadmap(Scene*);
	void explore(int,int);
	Path getPath(FullNode,FullNode,Point*,bool);
	Path OkCatch(Position start,Position end,Point* ball);
	std::vector<FullNode> waypoints;
private:
	void addNode(FullNode,int);
	Scene* scene;
	QuadTree tree;
	int nbClasses;
	std::vector<int> classes;

	std::vector<std::vector<Adj>> adjacency;
	std::vector<double> failRate;
};