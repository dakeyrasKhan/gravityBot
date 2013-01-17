
#include "path.hpp"

Position Path::getNextPosition(Vector speed, double deltaT){
	if(this.isDone())
		return this.currentPos;
	
	Vector diff = this.waypoints[this.current+1] - this.currentPos;
	Vector direction = diff.normalize();
	Vector toAdd = speed*deltaT*direction;
	if(toAdd.norm()>diff.norm()){
		this.current++;
		this.currentPos=this.waypoints[this.current];
	}
	else{
		this.currentPos+=toAdd;	
	}	
	return this.currentPos;
}