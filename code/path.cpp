#include "path.hpp"

Position Path::getNextPosition(Vector speed, double deltaT)
{
	if(isDone())
		return currentPos;
	
	Vector diff = waypoints[current+1] - currentPos;
	Vector direction = normalize(diff);
	Vector toAdd = speed*deltaT*direction;

	if(norm(toAdd) > norm(diff))
	{
		current++;
		currentPos= waypoints[current];
	}
	else
		currentPos += toAdd;	

	return currentPos;
}
