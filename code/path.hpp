#include <vector>
#include "Vector.hpp"

class Path{

public:
	Path(std::vector<Position>& w) : waypoints(w), current(1), currentPos(w[0]) { };
	Position getNextPosition(Vector speed, double deltaT);
	bool isDone(){ return current==waypoints.size()-1; }

private:
	std::vector<Position> waypoints;
	Position currentPos;
	int current;
};