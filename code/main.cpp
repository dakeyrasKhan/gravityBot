#include <iostream>
#include "Display.hpp"
#include "Scene.hpp"
#include "octree.hpp"
#include "roadmap.hpp"
#include <random>


int main(int argc, char * argv[])
{
	srand(424242);

	Point robotSize;
	robotSize[0] = 1; robotSize[1] = .5; robotSize[2] = 1.5;
	Scene scene("../scenes/scene0.obj", robotSize);
	Display display(&argc, argv, &scene);
	
	std::vector<Position> trajectory;
	bool withRoadmap=true;
	bool optimize=false;
	Position start;
	start[0]=3;start[1]=1;start[2]=-1;start[3]=-3;start[4]=1;
	Position end;
	end[0]=5;end[1]=0;end[2]=0;end[3]=-3;end[4]=-4;

	if(withRoadmap)
	{
		std::cout<<"creating roadmap"<<std::endl;
		Roadmap roadmap(&scene);
		std::cout<<"roadmap created"<<std::endl;	
		Path p = roadmap.getPath(start, end, false,NULL);
		if(optimize)
			trajectory = scene.Optimize(p.waypoints);
		else{
			trajectory = p.waypoints;
		}
	}
	else
	{
		trajectory.push_back(start);
		trajectory.push_back(end);
	}

	display.SetTrajectory(trajectory);

	/*Position p;
	p[ROBOT_ROT] = 1.12507; 
	p[ROBOT_ARM0] = 0.5;
	p[ROBOT_ARM1] = 0.5;
	p[ROBOT_X] = -0.333333;
	p[ROBOT_Z] = -0.333333;
	display.SetTrajectory(p);*/

	display.MainLoop();

	return 0;
}
