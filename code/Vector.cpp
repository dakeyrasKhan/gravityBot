#include "Vector.hpp"
#include <random>

Position randomCatch(Point p){
	double length = (ARM_LENGTH-SPACE)*(double(rand())/double(RAND_MAX))+SPACE;
	double angle = 2.*Pi*(double(rand())/double(RAND_MAX));
	//Al-Kashi
	double angle1 = acos(-(ARM2*ARM2-ARM_LENGTH*ARM_LENGTH-ARM1*ARM1)/(2.*ARM_LENGTH*ARM1));
	double angle2 = acos(-(ARM_LENGTH*ARM_LENGTH-ARM2*ARM2-ARM1*ARM1)/(2.*ARM2*ARM1));
	double x = p[ROBOT_X]-cos(angle)*length;
	double z = p[ROBOT_Z]-sin(angle)*length;
	return Position(); //Position(0,x,z,angle,angle1,angle2)
}