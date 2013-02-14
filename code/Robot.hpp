#pragma once
#include "Object.hpp"
#include "Box.hpp"
#include "Vector.hpp"

class Robot
{
public:
	Robot(Point size);
	Object GetObject(const Position& pos) const;
	std::array<Box, 2> GetBaseBoxes(const Position& pos) const;
	std::array<Box, 2> GetArmsBoxes(const Position& pos) const;

	double armLength(){return arm0Length+arm1Length;};
	double maxDist(){return sqrt(armLength()*armLength()-
								 baseArmLength*baseArmLength);};
	Position SetAngles(double d,Position p);
	Position RandomCatch(Point p);
	Position Catch(Position pos,Point p);
	Position CorrectBallPos(Position pos) const;

	void printSize() const;

	const Point baseSize;
	const double yPos;
	const double armWidth;
	const double arm0Length;
	const double arm1Length;
	const double baseArmLength;
	const double minSpace;

private:
	Object baseObject, arm0Object, arm1Object;
	static Object BuildBox(const Point& size);

	double baseArmY;
	double baseHeight;
};