#pragma once
#include <vector>
#include "Vector.hpp"
#include "octree.hpp"

class Path
{
public:
	Path(){};
	Path(std::vector<FullNode>& w) : waypoints(w), current(1), currentPos(w[0]) { };
	FullNode getNextPosition(Vector speed, double deltaT);
	bool isDone(){ return current==waypoints.size()-1; };
	void add(FullNode a){ waypoints.push_back(a);};
	bool empty(){ return waypoints.empty();};
	std::vector<FullNode> waypoints;
private:

	FullNode currentPos;
	int current;
};