#pragma once
#include "Vector.hpp"
#include <vector>
using namespace std;

const int sonFinder[2][2][2]={{{0,6},{2,4}},{{1,7},{3,5}}};
const double centerCalc[8][3]={{-1,-1,-1},{1,-1,-1},{-1,1,-1},{1,1,-1},
							{-1,1,1},{1,1,1},{-1,-1,1},{1,-1,1}};
const int MAXCLUSTER = 5;
const int NB_NEIGHBOURS = 5;

class FullNode{
public:
	FullNode(){}
	FullNode(Position p,int i,bool w):pos(p),id(i),with(w){}
	Position pos;
	Position setBall(Point *b){
		Position res=pos;
		res[BALL_X]=(*b)[X];
		res[BALL_Y]=(*b)[Y];
		res[BALL_Z]=(*b)[Z];
		return res;
	};
	int id;
	bool with;
};
class NodeComp{
public:
	NodeComp(FullNode n, double d):node(n),dist(d){}
	FullNode node;
	double dist;
	bool inline operator<(const NodeComp& other) const {
		return dist>other.dist;
	};
};

class Octree {
public:
	Octree(){
		for(int i=0;i<8;i++)
			son[i]=NULL;
		brokenDown = false;
		nbWith=nbWithout=0;
		depth=0;
	};
	Octree(Point neg, Point pos){
		for(unsigned int i=0;i<neg.size();i++){
			size[i]=pos[i]-neg[i];
			center[i]=(pos[i]+neg[i])/2.;
		}
		for(int i=0;i<8;i++)
			son[i]=NULL;
		brokenDown = false;
		nbWith=nbWithout=0;
		depth=0;
	};
	~Octree(){
		for(int i=0;i<8;i++)
			if(son[i]!=NULL)
				delete son[i];
	};
	vector<FullNode> values;
	int nbWith;
	int nbWithout;
	int depth;
	Point center;
	Octree* son[8];
	double size[3];
	bool brokenDown;
};


int findSon(FullNode, Octree*);
bool excluded(Point,double, Octree*);
bool included(Point,double, Octree*);
Point calculateCenter(int, Octree*);
void addPos(FullNode, Octree*);

int countInCube(Point, double, bool, Octree*);
void addToVect(Point, double, bool, Octree*, vector<FullNode>*);
double findSizeCube(double, double, Point, int, bool, Octree*);
void findNeighbours(Point,double,bool,Octree*,vector<FullNode>*);
