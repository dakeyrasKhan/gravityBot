#include "path.hpp"

FullNode Path::getNextPosition(Vector speed, double deltaT)
{
	if(isDone())
		return currentPos;
	
	Vector diff = waypoints[current+1].pos - currentPos.pos;
	Vector direction = diff.Normalize();
	Vector toAdd = speed*deltaT*direction;

	if(toAdd.Norm() > diff.Norm())
	{
		current++;
		currentPos.pos = waypoints[current].pos;
	}
	else
		currentPos.pos += toAdd;	

	return currentPos;
}
