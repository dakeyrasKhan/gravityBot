#include "octree.hpp"

int findSon(Position pos,Octree *tree){
	Point point = pos.toPoint();
	Point center = tree->center;
	int x = point[X]>center[X];
	int y = point[Y]>center[Y];
	int z = point[Z]>center[Z];
	return sonFinder[x][y][z];
}

bool excluded(Point center,double size, Octree *tree){
	bool res = false;

	for(int i=0;i<3;i++){
		double beginT = tree->center[i]-tree->size[i]/2.;
		double endT = tree->center[i]+tree->size[i]/2.;
		double begin = center[i]-size/2.;
		double end = center[i]+size/2.;
		res = res || ( endT<=begin || end<=beginT);
	}

	return res;
}

bool included(Point center,double size, Octree *tree){
	bool res = false;

	for(int i=0;i<3;i++){
		double beginT = tree->center[i]-tree->size[i]/2.;
		double endT = tree->center[i]+tree->size[i]/2.;
		double begin = center[i]-size/2.;
		double end = center[i]+size/2.;
		res = res && ( beginT>=begin || endT<=end);
	}

	return res;
}

Point calculateCenter(int son, Octree* tree){
	Point result = tree->center;
	for(int i=0;i<3;i++)
		result[i] += centerCalc[son][i]*0.25*tree->size[i];
	return result;
}

void addPos(Position pos, Octree* tree){
	if(tree->depth==MAX_DEPTH){
		tree->values.push_back(pos);
		return;
	}
	int son = findSon(pos,tree);

	if(tree->son[son]==NULL){
		tree->son[son]= new Octree;
		for(int i=0;i<3;i++)
			tree->son[son]->size[i]=tree->size[i]/2.;
		tree->son[son]->center=calculateCenter(son,tree);
		tree->son[son]->depth=tree->depth+1;
	}
	addPos(pos,tree->son[son]);
	tree->values.push_back(pos);
}

int countInCube(Point center, double size, Octree* tree){
	if(excluded(center,size,tree))
		return 0;
	if(included(center,size,tree) || tree->depth==MAX_DEPTH)
		return tree->values.size();

	int total=0;
	for(int i=0;i<8;i++)
		total+=countInCube(center,size,tree->son[i]);
	
	return total;
}

void addToVect(Point center, double size, Octree *tree, vector<Position>* values){
	
	if(excluded(center,size,tree))
		return;
	
	if(included(center,size,tree) || tree->depth==MAX_DEPTH)
		for(int i=0;i<tree->values.size();i++)
			values->push_back(tree->values[i]);

	for(int i=0;i<8;i++)
		addToVect(center,size,tree->son[i],values);
}

double findSizeCube(double begin, double end, Point center, int nbValues, Octree *tree){
	if(countInCube(center,begin,tree)==countInCube(center,end,tree))
		return end;
	double mid = (end+begin)/2.;
	int count = countInCube(center,mid,tree);

	if(count<=nbValues)
		return findSizeCube(mid,end,center,nbValues,tree);

	return findSizeCube(begin,mid,center,nbValues,tree);
}