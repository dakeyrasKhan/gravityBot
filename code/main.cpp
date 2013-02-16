#include <iostream>
#include "Display.hpp"
#include "Scene.hpp"
#include "octree.hpp"
#include "roadmap.hpp"
#include <random>
#include <ctime>


int main(int argc, char * argv[])
{
	srand(time(NULL));

	Point robotSize;
	robotSize[0] = 1; robotSize[1] = .5; robotSize[2] = 1.5;
	Scene scene("../scenes/scene1.obj", robotSize);
	Display display(&argc, argv, &scene);
	
	std::vector<Position> trajectory;
	bool withRoadmap=true;
	bool optimize=false;
	Position start;
	start[0]=4.29731;
	start[1]=1.4429;
	start[2]=0.683601;
	start[3]=3.04187;
	start[4]=-2.60427;
	start[5]=3.78442;
	start[6]=0.2;
	start[7]=-0.919267;

	//start=Random(scene.NegSize(),scene.PosSize(),true,scene.robot);
	/*start[0]=Pi/4.;start[1]=Pi/4.;start[2]=Pi/4.;
	start[3]=0;start[4]=0;
	start = scene.robot.CorrectBallPos(start);
	Point posDrop = scene.Drop(start);
	start[5]=posDrop[X];start[6]=posDrop[Y];start[7]=posDrop[Z];
	start[0]=-Pi/4.;

	//Position r = scene.robot.RandomCatch(posDrop);
	
	Position end;
	end[0]=0;end[1]=0;end[2]=0;end[3]=0;end[4]=0;
	end = scene.robot.CorrectBallPos(end);
	
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
	}*/

	std::cout<<"COLLISION ? "<<scene.Collision(start,BALL_ON_ARM)<<std::endl;	
	trajectory.push_back(start);
	display.SetTrajectory(trajectory);

	display.MainLoop();

	return 0;
}
