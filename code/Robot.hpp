#pragma once
#include <ozcollide/aabbtree_poly.h>
#include "Vector.hpp"
#include "Object.hpp"

#define ROBOT_ROT	0
#define ROBOT_X		1
#define ROBOT_Z		2

class Robot
{
public:
	Robot(Point size);
	Object GetObject(const Position& pos) const;
	ozcollide::OBB GetBox() const;
	void printSize() const;

	const Point baseSize;
	const double yPos;
private:
	Object object;
};