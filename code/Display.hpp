#pragma once
#include <chrono>
#include "Scene.hpp"
#include "path.hpp"
#include <GL/glut.h>

#define CAM_SPEED 20.0
#define ROBOT_SPEED 1.0

class Display
{
public:
	Display(int* argc, char* argv[], Scene* scene);
	void Render();
	void MainLoop() { glutMainLoop(); };
	void KeyboardFunc(unsigned char key, bool down);
	void MouseFunc(int button, bool down, int x, int y);
	void MotionFunc(int x, int y);
	void SetTrajectory(const std::vector<Position>& trajectory);
	void SetTrajectory(const Position& position);

private:
	typedef std::chrono::high_resolution_clock clock;
	typedef std::array<int, 2> Pixel;
	enum Key {Z, Q, S, D, R, F, M_LEFT};

	Scene* scene;
	int width;
	int height;
	void DrawAxis();
	void DrawObject(const Object& object);
	void SetView(double timediff);

	std::array<bool, 7> keys;
	clock::time_point lastRender;
	clock::duration renderInterval;
	Point position, direction, up;
	Pixel mousePos;
	Pixel oldMousePos;

	std::vector<Position> trajectory;
	bool isTrajectoryEnded;
	clock::time_point lastWaypointTime;
	int lastWaypoint;
	Position UpdatePosition(clock::time_point time);
};

