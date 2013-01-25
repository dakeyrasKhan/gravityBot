#pragma once
#include "Scene.h"
#include <GL/glut.h>

class Display
{
public:
	Display(int* argc, char* argv[], Scene* scene);
	void Render();
	void MainLoop() { glutMainLoop(); };

private:
	Scene* scene;
	static const int width = 800;
	static const int height = 800;
};

