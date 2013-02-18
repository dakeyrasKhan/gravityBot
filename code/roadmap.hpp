#pragma once
#include "octree.hpp"
#include "Scene.hpp"
#include "path.hpp"
#include <queue>
#include <stack>
<<<<<<< HEAD
#define NB_WAYPOINTS 200
=======
#define NB_WAYPOINTS 100
>>>>>>> 4f4505b3de2d2d9ce97a5629b8b580ce3a70d09c
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
	vector<FullNode> waypoints;
private:
	void addNode(FullNode,int);
	Scene* scene;
	Octree tree;
	int nbClasses;
	vector<int> classes;

	vector<vector<Adj>> adjacency;
	vector<double> failRate;
};