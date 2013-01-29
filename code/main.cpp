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
	bool optimize=false;
	Position start;
	start[0]=3;start[1]=-3;start[2]=1;
	Position end;
	end[0]=5;end[1]=-3;end[2]=-4;

	if(withRoadmap){
		std::cout<<"creating roadmap"<<std::endl;
		Roadmap roadmap(&scene);
		std::cout<<"roadmap created"<<std::endl;	
		Path p = roadmap.getPath(start, end, false,NULL);
		if(optimize)
			trajectory = scene.Optimize(p.waypoints);
		else{
			Position s=p.waypoints[0];
			Position e=p.waypoints[1];
			Position collision((s+2.*e)/3.);
			trajectory.push_back(collision);
			std::cout<<"BLAH : "<<scene.Collision(collision,false,NULL)<<std::endl;
			//trajectory.push_back(p.waypoints[1]);
		}
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
