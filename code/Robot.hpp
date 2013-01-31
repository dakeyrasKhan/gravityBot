#pragma once
#include "Vector.hpp"
#include "Object.hpp"
#include "Box.hpp"

#define ROBOT_ROT	0
#define ROBOT_ARM0	1
#define ROBOT_ARM1	2
#define ROBOT_X		3
#define ROBOT_Z		4

class Robot
{
public:
	Robot(Point size);
	Object GetObject(const Position& pos) const;
	std::array<Box, 2> GetBaseBoxes(const Position& pos) const;
	std::array<Box, 2> GetArmsBoxes(const Position& pos) const;

	void printSize() const;

	const Point baseSize;
	const double yPos;
	const double armWidth;
	const double arm0Length;
	const double arm1Length;
	const double baseArmLength;

private:
	Object baseObject, arm0Object, arm1Object;
	static Object BuildBox(const Point& size);

	double baseArmY;
	double baseHeight;
};