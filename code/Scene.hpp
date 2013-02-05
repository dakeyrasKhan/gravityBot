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

	bool Collision(const Position& position) const;
	const Object& StaticScene() const { return staticScene; };
	Object RobotObject(Position pos) const { return robot.GetObject(pos); };

	Position GetPosSize() const { return posSize; };
	Position GetNegSize() const { return negSize; };
	double GetMaxSize() const { return maxSize; };

	std::vector<Position> Optimize(std::vector<Position> p);
	bool validMove(Position, Position, bool, Point*);
	Point Drop(Position);

	const double ballRadius;

	Position posSize, negSize;
	double maxSize;

private:
	void ReadObjFile(const char* fileName);
	void BuildBaseScene();
	bool IntersectTriangleSphere(const int triangle, const Point& sphereCenter) const;

	Object staticScene;
	Robot robot;
	std::vector<std::array<int, 3>> baseScene;

};

