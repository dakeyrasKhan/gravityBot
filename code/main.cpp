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
	pos[0] = 0; pos[1] = -1; pos[2] = -2;
	display.SetTrajectory(pos);
	std::cout << scene.Collision(pos, false, nullptr);
	display.MainLoop();

	return 0;
}
