#include <iostream>
#include "Display.hpp"
#include "Scene.hpp"
#include "octree.hpp"
#include <random>


int main(int argc, char * argv[])
{
	srand(42);
	Scene scene("../scenes/scene0.obj");
	Display display(&argc, argv, &scene);

	Position pos;
	std::vector<Position> trajectory;

	pos[0] = 0; pos[1] = 0; pos[2] = 0;
	trajectory.push_back(pos);
	pos[0] = 2; pos[1] = 10; pos[2] = 10;
	trajectory.push_back(pos);

	display.SetTrajectory(trajectory);
	display.MainLoop();

	return 0;
}
