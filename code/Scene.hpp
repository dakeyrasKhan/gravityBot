#pragma once
#include <algorithm>
#include <exception>
#include "Vector.hpp"
#include "Object.hpp"
#include "Robot.hpp"
#include "Triangle.hpp"


#define NB_TRY 1000

#define BALL_ON_ARM 0x1
#define BALL_ON_FLOOR 0x2

class Scene
{
public:
	Scene(const char* sceneFile, const Point& robotSize);

	const Object& StaticScene() const { return staticScene; };
	Object RobotObject(Position pos) const { return robot.GetObject(pos); };

	Position PosSize() const { return posSize; };
	Position NegSize() const { return negSize; };
	double MaxSize() const { return maxSize; };

	bool Collision(const Position& position, const int ballStatus = 0) const;
	std::vector<Position> Optimize(std::vector<Position> p);
	bool ValidMove(const Position&, const Position&, const int ballStatus = 0) const;
	Point Drop(Position);
	Robot robot;
	
	const double ballRadius;

private:
	void ReadObjFile(const char* fileName);
	void BuildBaseScene();

	bool RobotCollision(const std::array<Box, 2>& baseBoxes, const std::array<Box, 2>& armsBoxes, 
		const Point& ballPos, const bool testBallArm1) const;
	bool GroundCollision(const std::array<Box, 2>& baseBoxes, const std::array<Box, 2>& armsBoxes, 
		const Point& ballPos, const bool testBall) const;

	Object staticScene;
	
	std::vector<std::array<int, 3>> noBaseGroundTriangles;
	std::vector<Triangle> groundTriangles;
	std::vector<Triangle> triangles;

	Position posSize, negSize;
	double maxSize;

};


class NoDropPointException : std::exception
{

};

