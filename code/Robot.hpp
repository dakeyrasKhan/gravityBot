#pragma once
#include <ozcollide/aabbtree_poly.h>
#include "Vector.hpp"
#include "Object.hpp"

class Robot
{
public:
	Robot() { };
	Robot(Point size) : size(size) { };
	Object GetObject() const;
	ozcollide::OBB GetBox() const;

private:
	Point size;
};