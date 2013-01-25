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
};

