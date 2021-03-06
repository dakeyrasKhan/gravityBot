#pragma once
#include "QuadTree.hpp"
#include "Scene.hpp"
#include "path.hpp"
#include <queue>
#include <stack>
#include <random>
#include <ctime>

#define NB_DENSIFY 20
#define NB_WAYPOINTS_WITH 300
#define NB_WAYPOINTS_WITHOUT 1000

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
	Roadmap() : tree(0, 0, 1) {};
	Roadmap(Scene*);
	void explore(int,int);
	Path getPath(FullNode,FullNode,Point*,bool);
	Path OkCatch(Position start,Position end,Point* ball);
	std::vector<FullNode> waypoints;
	std::vector<double> failRate;
private:
	void addNode(FullNode,int);
	Scene* scene;
	QuadTree tree;
	int nbClasses;
	std::vector<int> classes;

	std::mt19937 rng;
	double GetGaussianValue(const double m, const double sigma);
	Position GetGaussianPosition(const Position p, const double sigma);

	std::vector<std::vector<Adj>> adjacency;
	
};