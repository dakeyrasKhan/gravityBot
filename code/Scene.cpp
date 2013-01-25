#include "Scene.h"


Scene::Scene()
{
	std::array<double, 3> p[] =
	{
		{0.5, 0.5, 0.5},
		{0.5, -0.5, 0.5},
		{-0.5, 0.5, 0.5},
	};

	std::array<int, 3> t[] =
	{
		{0, 1, 2},
	};

	for(int i=0; i<3; i++)
		points.push_back(p[i]);
	for(int i=0; i<1; i++)
		triangles.push_back(t[i]);
}
