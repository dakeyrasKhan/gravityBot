#include "Scene.h"

bool Scene::collision(Position pos)
{
	return false;
}

bool Scene::validMove(Position a, Position b)
{
	double length=Position((b-a)).Norm();
	if(length<=EPS)
		return true;

	Position mid = Position((a+b))/2.;
	if(collision(mid))
		return false;
	return validMove(a,mid) && validMove(mid,b);
}


Scene::Scene()
{
	std::array<double, 3> p[] =
	{
		{0.5, 0.5, 0.5},
		{0.5, -0.5, 0.5},
		{-0.5, 0.5, 0.5},
		{0.5, 1, 1},
	};

	std::array<int, 3> t[] =
	{
		{0, 1, 2},
		{0, 2, 3},
	};

	for(int i=0; i<4; i++)
		points.push_back(p[i]);
	for(int i=0; i<2; i++)
		triangles.push_back(t[i]);
}
