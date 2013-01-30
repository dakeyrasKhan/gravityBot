#pragma once
#include <ozcollide/aabbtree_poly.h>
#include "Vector.hpp"
#include "Object.hpp"

#define ROBOT_ROT	0
#define ROBOT_ARM0 1
#define ROBOT_ARM1 2
#define ROBOT_X		3
#define ROBOT_Z		4

class Robot
{
public:
	Robot(Point size);
	Object GetObject(const Position& pos) const;
	ozcollide::OBB GetBox() const;
	void printSize() const;

	const Point baseSize;
	const double yPos;
	const double armWidth;
	const double arm0Length;
	const double arm1Length;
private:
	Object baseObject, arm0Object, arm1Object;
	static Object BuildBox(const Point& size);
};