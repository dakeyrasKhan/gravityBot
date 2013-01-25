#include "octree.hpp"

Point toPoint(Position v){
	Point p;
	for(int i=0;i<p.size();i++)
		p[i]=v[i];
	return p;
}

int findSon(Position pos, Octree *tree)
{
	Point point = pos.ToPoint();
	Point center = tree->center;
	int x = point[X] > center[X];
	int y = point[Y] > center[Y];
	int z = point[Z] > center[Z];
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

void addSon(Position pos, Octree* tree){
	int son = findSon(pos,tree);
	if(tree->son[son]==NULL){
		tree->son[son]= new Octree;
		for(int i=0;i<3;i++)
			tree->son[son]->size[i]=tree->size[i]/2.;
		tree->son[son]->center=calculateCenter(son,tree);
		tree->son[son]->depth=tree->depth+1;
		
	}
	addPos(pos,tree->son[son]);
}

void addPos(Position pos, Octree* tree){
	
	tree->values.push_back(pos);
	
	if(tree->values.size()<=MAXCLUSTER || tree->depth==MAX_DEPTH)
		return;
	
	if(tree->values.size()>MAXCLUSTER && tree->brokenDown == false){
		tree->brokenDown=true;
		for(auto elem : tree->values)
			addSon(elem,tree);
	}
	else
		addSon(pos,tree);
}

int countInCube(Point center, double size, Octree* tree){
	if(tree==NULL || excluded(center,size,tree)){
		return 0;
	}
	if(included(center,size,tree) || tree->depth==MAX_DEPTH || 
	   tree->brokenDown==false){
		return tree->values.size();
	}

	int total=0;
	for(int i=0;i<8;i++)
		total+=countInCube(center,size,tree->son[i]);
	return total;
}

void addToVect(Point center, double size, Octree* tree, vector<Position>* values){
	if(tree==NULL || excluded(center,size,tree)){
		return;
	}
	if(included(center,size,tree) || tree->depth==MAX_DEPTH || tree->brokenDown==false)
	   	values->insert(values->end(),tree->values.begin(), tree->values.end());
	

	for(int i=0;i<8;i++)
		addToVect(center,size,tree->son[i],values);
}

double findSizeCube(double begin, double end, Point center, int nbValues, Octree *tree){
	
	int beginC = countInCube(center,begin,tree);
	int endC = countInCube(center,end,tree);
	if(beginC>=endC || abs(end-begin)<=EPS*tree->size[0]){
		return end;
	}
	double mid = (end+begin)/2.;
	int count = countInCube(center,mid,tree);
	if(count<=nbValues)
		return findSizeCube(mid,end,center,nbValues,tree);

	return findSizeCube(begin,mid,center,nbValues,tree);
}