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

	const Object& StaticScene() const { return staticScene; };
	Object RobotObject(Position pos) const { return robot.GetObject(pos); };

	Position PosSize() const { return posSize; };
	Position NegSize() const { return negSize; };
	double MaxSize() const { return maxSize; };

	bool Collision(const Position& position) const;
	std::vector<Position> Optimize(std::vector<Position> p);
	bool validMove(Position, Position, bool, Point*);
	Point Drop(Position);

	const double ballRadius;

private:
	void ReadObjFile(const char* fileName);
	void BuildBaseScene();

	bool IntersectTriangleSphere(const int triangle, const Point& sphereCenter) const;
	bool RobotCollision(const std::array<Box, 2>& baseBoxes, const std::array<Box, 2>& armsBoxes, 
		const Point& ballPos, const bool testBallArm1) const;
	bool GroundCollision(const std::array<Box, 2>& baseBoxes, const std::array<Box, 2>& armsBoxes, 
		const Point& ballPos, const bool testBall) const;

	Object staticScene;
	Robot robot;
	std::vector<std::array<int, 3>> baseScene;

	Position posSize, negSize;
	double maxSize;

};

