#pragma once
#include <ozcollide/aabbtree_poly.h>
#include "Vector.hpp"
#include "Object.hpp"

#define ROBOT_Y 0.25
#define ROBOT_HEIGHT 0.5

class Robot
{
public:
	Robot() { };
	Robot(Point size);
	Object GetObject() const { return object; };
	ozcollide::OBB GetBox() const;
	void printSize() const;
private:
	Point size;
	Object object;
};