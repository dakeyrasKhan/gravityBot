#pragma once
#include "Vector.hpp"
#include <vector>
using namespace std;

#define MAX_DEPTH 8
const double EPS (1./double(1<<MAX_DEPTH));
enum{X,Y,Z};

const int sonFinder[2][2][2]={{{0,6},{2,4}},{{1,7},{3,5}}};
const double centerCalc[8][3]={{-1,-1,-1},{1,-1,-1},{-1,1,-1},{1,1,-1},
							{-1,1,1},{1,1,1},{-1,-1,1},{1,-1,1}};
const int MAXCLUSTER = 10;

struct Node{
	Position pos;
	int id;
};

class Octree {
public:
	Octree(){
		for(int i=0;i<8;i++)
			son[i]=NULL;
		brokenDown = false;
	};
	~Octree(){
		for(int i=0;i<8;i++)
			if(son[i]!=NULL)
				delete son[i];
	}
	vector<Node> values;
	int depth;
	Point center;
	Octree* son[8];
	double size[3];
	bool brokenDown;
};


int findSon(Position, Octree*);
bool excluded(Point,double, Octree*);
bool included(Point,double, Octree*);
Point calculateCenter(int, Octree*);
void addPos(Position, Octree*);
int countInCube(Point, double, Octree*);
void addToVect(Point, double, Octree*, vector<Position>*);
double findSizeCube(double, double, Point, int, Octree*);
void findNeighbours(Point,double,Octree*,vector<Position>*);