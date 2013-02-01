#pragma once
#include <algorithm>
#include "Vector.hpp"
#include "Object.hpp"
#include "Robot.hpp"

#define NB_TRY 1000

class Scene
{
public:
	Scene(const char* sceneFile, const Point& robotSize);

	bool Collision(Position position, bool, Point*);
	const Object& StaticScene() const { return staticScene; };
	Object RobotObject(Position pos) const { return robot.GetObject(pos); };

	Position GetPosSize() const { return posSize; };
	Position GetNegSize() const { return negSize; };
	double GetMaxSize() const { return maxSize; };

	std::vector<Position> Optimize(std::vector<Position> p);
	bool validMove(Position, Position, bool, Point*,bool p=false);
	Point Drop(Position);

	Position posSize, negSize;
	double maxSize;

private:
	void ReadObjFile(const char* fileName);
	void BuildBaseScene();

	Object staticScene;
	Robot robot;
	std::vector<std::array<int, 3>> baseScene;

};

