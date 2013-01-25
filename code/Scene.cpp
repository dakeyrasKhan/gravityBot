#include "Scene.h"

bool Scene::collision(Position pos){}

bool Scene::validMove(Position a, Position b){}


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
