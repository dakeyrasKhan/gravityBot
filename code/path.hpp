#include <vector>
#include "Vector.hpp"

class Path
{
public:
	Path(){};
	Path(std::vector<Position>& w) : waypoints(w), current(1), currentPos(w[0]) { };
	Position getNextPosition(Vector speed, double deltaT);
	bool isDone(){ return current==waypoints.size()-1; };
	void add(Position a){ waypoints.push_back(a);};
	bool empty(){ return waypoints.empty();};
private:
	std::vector<Position> waypoints;
	Position currentPos;
	int current;
};