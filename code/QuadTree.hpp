#pragma once
#include "Vector.hpp"
#include <vector>

const int NB_NEIGHBOURS = 5;

class FullNode{
public:
	FullNode(){}
	FullNode(Position p,int i,bool w):pos(p),id(i),with(w){}
	Position pos;
	Position setBall(Point *b){
		Position res=pos;
		res[BALL_X]=(*b)[X];
		res[BALL_Y]=(*b)[Y];
		res[BALL_Z]=(*b)[Z];
		return res;
	};
	int id;
	bool with;
};

class NodeComp{
public:
	NodeComp(FullNode n, double d):node(n),dist(d){}
	FullNode node;
	double dist;
	bool inline operator<(const NodeComp& other) const {
		return dist>other.dist;
	};
};


class QuadTree
{
public:
	QuadTree(const Coord& center, const double size);
	QuadTree(const double centerX, const double centerZ, const double size);
	~QuadTree();

	void AddValue(const FullNode&);
	void AddNeighbors(const Coord& p, const int nNeighbors, 
		const bool withBall, std::vector<FullNode>& neighbors) const;

private:
	static const int maxCluster = 5;

	void Split();
	int FindSon(const Coord& coord) const;
	double FindCubeSize(const Coord& p, const int nNeighbors, const bool withBall) const;
	void AddNeighborsInCube(const Coord& p, const double cubeSize, const bool withBall, 
		std::vector<FullNode>& neighbors) const;
	int CountInCube(const Coord& p, const double cubeSize, const bool with) const;
	static bool IsInCube(const Coord& p, const Coord& c, const double cubeSize);

	QuadTree* sons[4];
	double size;
	Coord center;

	int cWith, cWithout;

	std::vector<FullNode> values;
};

