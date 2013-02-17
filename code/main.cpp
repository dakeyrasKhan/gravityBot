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
	bool withRoadmap=false;
	bool optimize=false;

	Position start,end;
	start[0]=5.44703;
	start[1]=0.994772;
	start[2]=-0.175478;
	start[3]=1;
	start[4]=-4;
	start[5]=-0.0519309;
	start[6]=1.92743;
	start[7]=-2.83547;
	end[0]=3.44801;
	end[1]=0.977036;
	end[2]=1.87892;
	end[3]=-1.11081;
	end[4]=-3.17048;
	end[5]=-0.0519309;
	end[6]=0.2;
	end[7]=-2.83547;
	
	//start=Random(scene.NegSize(),scene.PosSize(),true,scene.robot);
	/*start[0]=-Pi/4.;start[1]=Pi/4.;start[2]=Pi/4.;
	start[3]=0;start[4]=0;
	start = scene.robot.CorrectBallPos(start);*/
	/*Point posDrop;
	try{
		posDrop = scene.Drop(start);
	}
	catch(NoDropPointException e){
		std::cout<<"oops"<<std::endl;
	}
	start[5]=posDrop[X];start[6]=posDrop[Y];start[7]=posDrop[Z];
	start[0]=-Pi/4.;*/
	//Position r = scene.robot.RandomCatch(posDrop);
	/*
	end[0]=0;end[1]=0;end[2]=0;end[3]=0;end[4]=0;
	end = scene.robot.CorrectBallPos(end);*/
	
	if(withRoadmap)
	{
		std::cout<<"creating roadmap"<<std::endl;
		Roadmap roadmap(&scene);
		std::cout<<"roadmap created"<<std::endl;	
		Path p = roadmap.getPath(FullNode(start,-1,true),FullNode(end,-1,true),NULL);
		if(optimize)
			trajectory = scene.Optimize(p.waypoints);
		else{
			for(auto wp : p.waypoints)
				trajectory.push_back(wp.pos);
		}
	}
	else
	{
		trajectory.push_back(start);
		//trajectory.push_back(end);
	}

	std::vector<bool> ballOnArm;
	for(auto x : trajectory)
		ballOnArm.push_back(true);

	display.SetTrajectory(trajectory, ballOnArm);

	display.MainLoop();

	return 0;
}
