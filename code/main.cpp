#include <iostream>
#include "Display.h"
#include "Scene.h"
#include "octree.hpp"
#include <random>

using namespace std;


int main(int argc, char * argv[])
{
	srand(42);
	Scene scene("../scenes/test.obj");
	Display display(&argc, argv, &scene);

	std::cout << mod(42.42, Pi);
	display.MainLoop();

	return 0;
}


/*Octree tree;
	for(int i=0;i<3;i++){
		tree.size[i]=8;
		tree.center[i]=4;
	}
	tree.depth=0;

	Position layer1[8]=
		{{3,3,3,1,0,0,0,0,0,0},
		 {3,3,5,1,0,0,0,0,0,0},
		 {3,5,3,1,0,0,0,0,0,0},
		 {3,5,5,1,0,0,0,0,0,0},
		 {5,3,3,1,0,0,0,0,0,0},
		 {5,3,5,1,0,0,0,0,0,0},
		 {5,5,3,1,0,0,0,0,0,0},
		 {5,5,5,1,0,0,0,0,0,0}};

	Position layer2[8]=
		{{1,1,1,2,0,0,0,0,0,0},
		 {1,1,7,2,0,0,0,0,0,0},
		 {1,7,1,2,0,0,0,0,0,0},
		 {1,7,7,2,0,0,0,0,0,0},
		 {7,1,1,2,0,0,0,0,0,0},
		 {7,1,7,2,0,0,0,0,0,0},
		 {7,7,1,2,0,0,0,0,0,0},
		 {7,7,7,2,0,0,0,0,0,0}};

	for(int i=0;i<8;i++){
		addPos(layer1[i],&tree);
		addPos(layer2[i],&tree);
	}
	Point centerP={4,4,4};
	vector<Position> bli;
	addToVect(centerP,findSizeCube(0,8,centerP,10,&tree),&tree,&bli);
	for(auto elem : bli)
		printf("%lf %lf %lf\n",elem[0],elem[1],elem[2]);*/
