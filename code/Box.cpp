#include "Box.hpp"

ozcollide::OBB Box::ToOzcollide() const
{
	ozcollide::OBB box;
	box.center = ozcollide::Vec3f(center[0], center[1], center[2]);
	box.extent = ozcollide::Vec3f(size[0]/2, size[1]/2, size[2]/2);

	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			box.matrix.m_[i][j] = rotation[i][j];

	return box;
}