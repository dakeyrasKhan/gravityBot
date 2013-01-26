#pragma once
#include <chrono>
#include "Scene.h"
#include <GL/glut.h>

#define SPEED 20.0

class Display
{
public:
	Display(int* argc, char* argv[], Scene* scene);
	void Render();
	void MainLoop() { glutMainLoop(); };
	void KeyboardFunc(unsigned char key, bool down);

private:
	typedef std::chrono::high_resolution_clock clock;
	enum Key {Z, Q, S, D};

	Scene* scene;
	int width;
	int height;
	void DrawAxis();
	void DrawTriangles();
	void SetView(double timediff);

	std::array<bool, 4> keys;
	clock::time_point lastRender;
	clock::duration renderInterval;
	Point position, direction, up;

};

