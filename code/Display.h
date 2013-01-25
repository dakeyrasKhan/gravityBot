#pragma once
#include <GL/glut.h>

class Display
{
public:
	Display(int* argc, char* argv[]);
	void Render();
	void MainLoop() { glutMainLoop(); };
};

