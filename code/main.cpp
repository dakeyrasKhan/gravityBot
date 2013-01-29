#include <iostream>
#include "Display.hpp"
#include "Scene.hpp"
#include "octree.hpp"
#include "roadmap.hpp"
#include <random>


int main(int argc, char * argv[])
{
	srand(424242);
	Scene scene("../scenes/scene0.obj");
	std::vector<Position> trajectory;
	bool withRoadmap=true;

	Position start;
	start[0]=6;start[1]=0;start[2]=0;
	Position end;
	end[0]=5;end[1]=-3;end[2]=-4;

	if(withRoadmap){
		std::cout<<"creating roadmap"<<std::endl;
		Roadmap roadmap(&scene);
		std::cout<<"roadmap created"<<std::endl;	
		Path p = roadmap.getPath(start, end, false,NULL);

		trajectory = scene.Optimize(p.waypoints);

	}
	else{
		trajectory.push_back(start);
		trajectory.push_back(end);
	}

	
	/*Position pos;
	std::vector<Position> trajectory;

	pos[0] = 0; pos[1] = 0; pos[2] = 0;
	trajectory.push_back(pos);
	pos[0] = 6.20878; pos[1] = 3; pos[2] = -2;
	trajectory.push_back(pos);

	display.SetTrajectory(trajectory);
	return 0;*/
	// = p.waypoints;
		


	Display display(&argc, argv, &scene);
	display.SetTrajectory(trajectory);


	display.MainLoop();

	return 0;
}
