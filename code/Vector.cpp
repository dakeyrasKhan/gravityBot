#include "Vector.hpp"
#include "Robot.hpp"
#include <random>

Position Random(const std::array<double, DIM_CONF>& neg,
							  const std::array<double, DIM_CONF>& pos,
							  bool with, const Robot& robot)
{
	Position toReturn;
	for(int i=0;i<DIM_CONF;i++)
		toReturn[i]=mod(rand(),pos[i]-neg[i])+neg[i];
	toReturn[5]=toReturn[6]=toReturn[7]=0;
	if(!with)
		return toReturn;

	return robot.CorrectBallPos(toReturn);
}

// n must be noramlized
Point SegmentPlaneClosestPoint(const Point& n, const Point& p, const Point& a0, const Point& a1)
{
	double origin = n|p;
	double nA0 = (n|a0) - origin;
	double nA1 = (n|a1) - origin;

	if(nA0*nA1 > 0)
		return abs(nA0) < abs(nA1) ? a0 : a1;
	else
	{
		Point dir = a1 - a0;
		return a0 + (nA0*dir.Normalize());
	}
}

double SegmentSegmentDistance(const Point& a0, const Point& a1, const Point& b0, const Point& b1)
{
	Point dirA = a1 - a0;
	Point dirB = b1 - b0;
	double length2A = dirA.Norm2();
	double length2B = dirB.Norm2();

	double dirAdirB = dirA|dirB;
	double delta = length2A*length2B - dirAdirB*dirAdirB;

	Point a0b0 = b0 - a0;
	double xB0overA = -(dirA|a0b0)/length2A;
    double xA0overB = +(dirB|a0b0)/length2B;

	double xA, xB; 

	if(abs(delta) < std::numeric_limits<double>::epsilon()) 
	{
        xA = 0;
        xB = xA0overB;
    }
    else
	{   
		xA = -length2B*(dirAdirB*xA0overB + length2A*xB0overA)/delta;
        xB = -length2A*(length2B*xA0overB + dirAdirB*xB0overA)/delta;

        if (xA < 0) 
		{
            xA = 0;
            xB = xA0overB;
        }
        else if(xA > 1) 
		{
            xA = 1;
            xB = xA0overB + dirAdirB/length2B;
        }
    }

    if(xB < 0) 
	{            
        xB = 0;
		xA = clamp(xB0overA, 0, 1);
    }
    else if(xB > 1) 
	{      
        xB = 1;
		xA = clamp(xB0overA + dirAdirB/length2A, 0, 1);
    }

	return Point(a0b0 - (xA*dirA) + (xB*dirB)).Norm();
}

