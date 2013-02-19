#include <iostream>
#include "Display.hpp"
#include "Scene.hpp"
#include "QuadTree.hpp"
#include "roadmap.hpp"
#include <random>
#include <ctime>


int main(int argc, char * argv[])
{
	srand(time(NULL));

	Point robotSize;
	robotSize[0] = 1; robotSize[1] = .5; robotSize[2] = 1.5;

	Scene scene("../scenes/scene5.obj", robotSize);
	Display display(&argc, argv, &scene);
	
	std::vector<Position> trajectory;
	bool withRoadmap=true;
	bool optimize=false;

	Position start,end;
	
	start[0]=0;
	start[1]=0;
	start[2]=2;
	start[3]=-3;
	start[4]=-3;
	start=scene.robot.CorrectBallPos(start);

	end[0]=0;
	end[1]=0;
	end[2]=2;
	end[3]=3;
	end[4]=3;
	end=scene.robot.CorrectBallPos(end);

	
	//start=Random(scene.NegSize(),scene.PosSize(),true,scene.robot);
	start[0] = -Pi/4.;start[1]=Pi/4.;start[2]=Pi/4.;
	start[3] = 5;start[4]=5;
	start = scene.robot.CorrectBallPos(start);
	Point posDrop;
	try{
		posDrop = scene.Drop(start);
	}
	catch(NoDropPointException e){
		std::cout<<"oops"<<std::endl;
	}
	start.setBall(&posDrop);
	//start[0]=-Pi/4.;
	//Position r = scene.robot.RandomCatch(posDrop);
	
	end[0]=0;end[1]=Pi/4.;end[2]=Pi/4.;end[3]=4;end[4]=-5;
	end = scene.robot.CorrectBallPos(end);
	try{
		posDrop = scene.Drop(end);
	}
	catch(NoDropPointException e){
		std::cout<<"oops"<<std::endl;
	}

	end.setBall(&posDrop);
	//end[3]=7;end[4]=-5;
	Roadmap roadmap;
	display.roadmap=NULL;
	std::vector<bool> ballOnArm;
	if(withRoadmap)
	{
		std::cout<<"creating roadmap"<<std::endl;
		roadmap=Roadmap(&scene);
		display.roadmap=&roadmap;
		std::cout<<"roadmap created with "<<roadmap.waypoints.size()<<" waypoints"<<std::endl;	
		//return 0;
		Path p = roadmap.getPath(FullNode(start,-1,false),FullNode(end,-1,false),NULL,true);

		if(optimize)
			trajectory = scene.Optimize(p.waypoints);
		else{
			Point lastBallPos=start.getBall();
			for(unsigned int i=0;i<p.waypoints.size();i++){
				FullNode wp=p.waypoints[i];
				ballOnArm.push_back(wp.with);
				if(wp.with)
					trajectory.push_back(scene.robot.CorrectBallPos(wp.pos));
				else{
					if(i!=0 && !p.waypoints[i-1].with)
						trajectory.push_back(wp.pos.setBall(&lastBallPos));
					else if(i!=0){
						lastBallPos=wp.pos.getBall();
						trajectory.push_back(wp.pos);
					}
					else
						trajectory.push_back(wp.pos);
				}
			}
		}
	}
	if(!withRoadmap || trajectory.empty()){
		std::cout << "empty trajectory" << std::endl;
		trajectory.push_back(start);
		ballOnArm.push_back(true);
		trajectory.push_back(end);
		ballOnArm.push_back(true);
	}

	//std::cout << scene.ValidMove(start, end, TRANSPORTING_BALL) << std::endl;
	display.SetTrajectory(trajectory, ballOnArm);


	display.MainLoop();

	return 0;
}
