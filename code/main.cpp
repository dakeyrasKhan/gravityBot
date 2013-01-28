#include <iostream>
#include "Display.hpp"
#include "Scene.hpp"
#include "octree.hpp"
#include <random>

using namespace std;


int main(int argc, char * argv[])
{
	srand(42);
	Scene scene("../scenes/scene0.obj");
	Display display(&argc, argv, &scene);

	std::cout << mod(42.42, Pi);
	display.MainLoop();

	return 0;
}
