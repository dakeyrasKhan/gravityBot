#include <iostream>
#include "Display.h"
#include "Vector.hpp"
#include "octree.hpp"


int main(int argc, char * argv[])
{
	Display display(&argc, argv);
	display.MainLoop();

	return 0;
}