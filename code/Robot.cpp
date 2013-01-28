#include "Robot.hpp"

ozcollide::OBB Robot::GetBox() const
{
	ozcollide::OBB box;
	box.extent = ozcollide::Vec3f(size[0], size[1], size[2]);
	return box;
}