#pragma once
#include <ozcollide/aabbtree_poly.h>
#include "Vector.hpp"
#include "Object.hpp"

class Robot
{
public:
	Robot() { };
	Robot(Point size);
	Object GetObject() const { return object; };
	ozcollide::OBB GetBox() const;

private:
	Point size;
	Object object;
};