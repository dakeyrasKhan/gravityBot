#include "QuadTree.hpp"


QuadTree::QuadTree(const Coord& center, const double size) : 
	size(size), sons(nullptr), center(center), cWith(0), cWithout(0)
{

}

QuadTree::QuadTree(const double centerX, const double centerZ, const double size) : 
	size(size), sons(nullptr), cWith(0), cWithout(0)
{
	center[0] = centerX;
	center[1] = centerZ;
}


QuadTree::~QuadTree()
{
	if(sons != nullptr)
		delete[] sons;
}


void QuadTree::AddValue(const FullNode& value)
{
	if(value.with)
		cWith++;
	else
		cWithout++;

	if(sons == nullptr)
	{
		values.push_back(value);
		if(values.size() > maxCluster)
			Split();
	}
	else
		sons[FindSon(value.pos.ToCoord())].AddValue(value);
}


int QuadTree::FindSon(const Coord& coord) const
{
	return(coord[0] > center[0] ? 2 : 0) + (coord[1] > center[1] ? 1 : 0);
}


void QuadTree::Split()
{
	sons = new QuadTree[4];
	double newSize = size/2;

	sons[0] = QuadTree(center[0]-newSize/2, center[1]-newSize/2, newSize);
	sons[1] = QuadTree(center[0]-newSize/2, center[1]+newSize/2, newSize);
	sons[2] = QuadTree(center[0]+newSize/2, center[1]-newSize/2, newSize);
	sons[3] = QuadTree(center[0]+newSize/2, center[1]+newSize/2, newSize);

	for(auto x : values)
		sons[FindSon(x.pos.ToCoord())].AddValue(x);

	values.clear();
}

void QuadTree::AddNeighbors(const Coord& p, 
							const int nNeighbors, 
							const bool withBall, 
							std::vector<FullNode>& neighbors) const
{
	double cubeSize = FindCubeSize(p, nNeighbors, withBall);
	AddNeighborsInCube(p, cubeSize, withBall, neighbors);
}

double QuadTree::FindCubeSize(const Coord& p, const int nNeighbors, const bool withBall) const
{
	double begin = 0;
	double end = size;
	while(abs(end-begin) > EPS*size)
	{
		double mid = (end+begin)/2.;
		int count = CountInCube(p, mid, withBall);

		if(count == nNeighbors)
			return mid;
		else if(count < nNeighbors)
			begin = mid;
		else
			end = mid;
	}

	return end;
}

void QuadTree::AddNeighborsInCube(const Coord& p, const double cubeSize, const bool withBall, 
								  std::vector<FullNode>& neighbors) const
{
	if(IsInCube(p, center, size+cubeSize))
	{
		if(sons == nullptr)
		{
			for(auto x : values)
				if(x.with == withBall && IsInCube(x.pos.ToCoord(), p, cubeSize))
					neighbors.push_back(x);
		}
		else
			for(int i=0; i<4; i++)
				sons[i].AddNeighborsInCube(p, cubeSize, withBall, neighbors);
	}
}

bool QuadTree::IsInCube(const Coord& p, const Coord& c, const double cubeSize)
{
	Coord x = p-c;
	return std::max(abs(x[0]), abs(x[1])) < cubeSize/2;
}

int QuadTree::CountInCube(const Coord& p, const double cubeSize, const bool with) const
{
	if(IsInCube(p, center, size+cubeSize))
	{
		if(sons == nullptr)
		{
			int c=0;
			for(auto x : values)
				if(x.with == with && IsInCube(x.pos.ToCoord(), p, cubeSize))
					c++;
			return c;
		}
		else if(IsInCube(center, p, cubeSize-size))
			return with ? cWith : cWithout;
		else
		{
			int c=0;
			for(int i=0; i<4; i++)
				c += sons[i].CountInCube(p, cubeSize, with);
			return c;
		}
	}
	else
		return 0;
}