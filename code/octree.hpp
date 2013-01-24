#pragma once
#include "Vector.hpp"
#include <vector>
using namespace std;

#define MAX_DEPTH 10
enum{X,Y,Z};

const int sonFinder[2][2][2]={{{0,6},{2,4}},{{1,7},{3,5}}};
const double centerCalc[8][3]={{-1,-1,-1},{1,-1,-1},{-1,1,-1},{1,1,-1},
							{-1,1,1},{1,1,1},{-1,-1,1},{1,-1,1}};

class Octree {
public:
	vector<Position> values;
	int depth;
	Point center;
	Octree* son[8];
	double size[3];	
};
