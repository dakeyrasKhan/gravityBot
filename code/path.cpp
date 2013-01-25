#include "path.hpp"

Position Path::getNextPosition(Vector speed, double deltaT)
{
	if(isDone())
		return currentPos;
	
	Vector diff = waypoints[current+1] - currentPos;
	Vector direction = diff.Normalize();
	Vector toAdd = speed*deltaT*direction;

	if(toAdd.Norm() > diff.Norm())
	{
		current++;
		currentPos= waypoints[current];
	}
	else
		currentPos += toAdd;	

	return currentPos;
}
