#include "Box.hpp"
#include <limits>
#include <cmath>
#include "Triangle.hpp"


bool Box::Intersect(Point p0, Point p1, Point p2) const
{
	// Transforme the triangle to have a unit cube centered on the origin
	p0 = ((p0 - center)*rotation)/size; 
	p1 = ((p1 - center)*rotation)/size; 
	p2 = ((p2 - center)*rotation)/size;

	// First compare all three vertexes with all face/edge/corne planes
	// If any vertex is inside the cube, return immediately!
	int mask0 = PlaneMask(p0);
	int mask1 = PlaneMask(p1);
	int mask2 = PlaneMask(p2);

	if(mask0 == 0 || mask1 == 0 || mask2 == 0)
		return true;

	// If all three vertexes were outside of one or more face/edge/corner planes,
	// return immediately with a trivial rejection!                  
	if((mask0 & mask1 & mask2) != 0) return false;

	// If vertex 0 and 1, as a pair, cannot be trivially rejected
	// by the above tests, then see if the v0-->v1 triangle edge 
	// intersects the cube.  Do the same for v0-->v2 and v1-->v2.
	// Pass to the intersection algorithm the "OR" of the outcode
	// bits, so that only those cube faces which are spanned by  
	// each triangle edge need be tested.                   
	if((mask0 & mask1) == 0 && CheckLine(p0, p1, mask0 | mask1)) 
		return true;
	if((mask0 & mask2) == 0 && CheckLine(p0, p2, mask0 | mask2)) 
		return true;
	if((mask1 & mask2) == 0 && CheckLine(p1, p2, mask1 | mask2)) 
		return true;

	// By now, we know that the triangle is not off to any side,     
	// and that its sides do not penetrate the cube.  We must now    
	// test for the cube intersecting the interior of the triangle.  
	// We do this by looking for intersections between the cube      
	// diagonals and the triangle... first finding the intersection   
	// of the four diagonals with the plane of the triangle, and     
	// then if that intersection is inside the cube, pursuing        
	// whether the intersection point is inside the triangle itself. 
	// The normal vector "norm" X,Y,Z components are the coefficients 
	// of the triangles AX + BY + CZ + D = 0 plane equation.  If we   
	// solve the plane equation for X=Y=Z (a diagonal), we get        
	// -D/(A+B+C) as a metric of the distance from cube center to the 
	// diagonal/plane intersection.  If this is between -0.5 and 0.5, 
	// the intersection is inside the cube.  If so, we continue by    
	// doing a point/triangle intersection.                           
	// Do this for all four diagonals.                                
	Point norm = (p0 - p1)^(p0 - p2);
	double d = norm | p0;

	Point hit;
	Triangle t(p0, p1, p2);
	hit[X] = hit[Y] = hit[Z] = d/(norm[X] + norm[Y] + norm[Z]);
	if(abs(hit[X]) <= 0.5 && t.IntersectPoint(hit))
		return true;

	hit[Z] = -(hit[X] = hit[Y] = d/(norm[X] + norm[Y] - norm[Z]));
	if(abs(hit[X]) <= 0.5 &&  t.IntersectPoint(hit))
		return true; 

	hit[Y] = -(hit[X] = hit[Z] = d/(norm[X] - norm[Y] + norm[Z]));
	if(abs(hit[X]) <= 0.5 &&  t.IntersectPoint(hit))
		return true;

	hit[Y] = hit[Z] = -(hit[X] = d / (norm[X] - norm[Y] - norm[Z]));
	if(abs(hit[X]) <= 0.5 &&  t.IntersectPoint(hit))
		return true;

	return false;
}

int Box::FacePlaneMask(const Point& p)
{
	int outcode = 0;

	// Which of the six face-planes is point P outside of
	if(p[X] >  .5) outcode |= 1 << 0;
	if(p[X] < -.5) outcode |= 1 << 1;
	if(p[Y] >  .5) outcode |= 1 << 2;
	if(p[Y] < -.5) outcode |= 1 << 3;
	if(p[Z] >  .5) outcode |= 1 << 4;
	if(p[Z] < -.5) outcode |= 1 << 5;

	return outcode;
}

int Box::PlaneMask(const Point& p)
{
	int outcode = 0;

	// Which of the six face-planes is point P outside of
	if(p[X] >  .5) outcode |= 1 << 0;
	if(p[X] < -.5) outcode |= 1 << 1;
	if(p[Y] >  .5) outcode |= 1 << 2;
	if(p[Y] < -.5) outcode |= 1 << 3;
	if(p[Z] >  .5) outcode |= 1 << 4;
	if(p[Z] < -.5) outcode |= 1 << 5;

	// Which of the twelve edge-planes is point P outside of
	if( p[X] + p[Y] > 1.0) outcode |= 1 << 8;
	if( p[X] - p[Y] > 1.0) outcode |= 1 << 9;
	if(-p[X] + p[Y] > 1.0) outcode |= 1 << 10;
	if(-p[X] - p[Y] > 1.0) outcode |= 1 << 11;
	if( p[X] + p[Z] > 1.0) outcode |= 1 << 12;
	if( p[X] - p[Z] > 1.0) outcode |= 1 << 13;
	if(-p[X] + p[Z] > 1.0) outcode |= 1 << 14;
	if(-p[X] - p[Z] > 1.0) outcode |= 1 << 15;
	if( p[Y] + p[Z] > 1.0) outcode |= 1 << 16;
	if( p[Y] - p[Z] > 1.0) outcode |= 1 << 17;
	if(-p[Y] + p[Z] > 1.0) outcode |= 1 << 18;
	if(-p[Y] - p[Z] > 1.0) outcode |= 1 << 19;

	// Which of the eight corner plane(s) is point P outside of
	if(( p[X] + p[Y] + p[Z]) > 1.5) outcode |= 1 << 24;
	if(( p[X] + p[Y] - p[Z]) > 1.5) outcode |= 1 << 25;
	if(( p[X] - p[Y] + p[Z]) > 1.5) outcode |= 1 << 26;
	if(( p[X] - p[Y] - p[Z]) > 1.5) outcode |= 1 << 27;
	if((-p[X] + p[Y] + p[Z]) > 1.5) outcode |= 1 << 28;
	if((-p[X] + p[Y] - p[Z]) > 1.5) outcode |= 1 << 29;
	if((-p[X] - p[Y] + p[Z]) > 1.5) outcode |= 1 << 30;
	if((-p[X] - p[Y] - p[Z]) > 1.5) outcode |= 1 << 31;

	return outcode;
}

// Test the point "alpha" of the way from P0 to P1
// See if it is on a face of the cube             
// Consider only faces in "mask"                  
int Box::CheckPoint(const Point& p0, const Point& p1, const double alpha, const int mask)
{
	return Box::FacePlaneMask(p0 + alpha*(p1 - p0)) & mask;
}

// Compute intersection of P0 --> P1 line segment with face planes 
// Then test intersection point to see if it is on cube face       
// Consider only face planes in "outcode_diff"                     
// Note: Zero bits in "outcode_diff" means face line is outside of 
bool Box::CheckLine(const Point& p0, const Point& p1, const int outcode_diff)
{
	if((0x01 & outcode_diff) != 0)
		if(CheckPoint(p0, p1, ( .5-p0[X])/(p1[X]-p0[X]), 0x3e) == 0)
			return true;

	if((0x02 & outcode_diff) != 0)
		if(CheckPoint(p0, p1, (-.5-p0[X])/(p1[X]-p0[X]), 0x3d) == 0)
			return true;

	if((0x04 & outcode_diff) != 0) 
		if(CheckPoint(p0, p1, ( .5-p0[Y])/(p1[Y]-p0[Y]), 0x3b) == 0)
			return true;

	if((0x08 & outcode_diff) != 0) 
		if(CheckPoint(p0, p1, (-.5-p0[Y])/(p1[Y]-p0[Y]), 0x37) == 0)
			return true;

	if((0x10 & outcode_diff) != 0) 
		if(CheckPoint(p0, p1, ( .5-p0[Z])/(p1[Z]-p0[Z]), 0x2f) == 0)
			return true;

	if((0x20 & outcode_diff) != 0) 
		if(CheckPoint(p0, p1, (-.5-p0[Z])/(p1[Z]-p0[Z]), 0x1f) == 0)
			return true;

	return false;
}

bool Box::IntersectBox(Box b) const
{
	// Intersect against the unit cube centered in the origin
	b.center = (b.center - center)*rotation;
	b.rotation = rotation.Transpose()*b.rotation;

	// Get the vertices of b
	std::array<double, 3>  vertices[] = 
	{
		{-0.5, -0.5, -0.5},
		{-0.5, -0.5, +0.5},
		{-0.5, +0.5, -0.5},
		{+0.5, -0.5, -0.5},
		{-0.5, +0.5, +0.5},
		{+0.5, -0.5, +0.5},
		{+0.5, +0.5, -0.5},
		{+0.5, +0.5, +0.5}
	};

	for(int i=0; i<8; i++)
		vertices[i] = b.rotation*(vertices[i]*b.size);

	// Test separation with box0 axis (3)
	for(int i=0; i<3; i++)
	{
		double min = +std::numeric_limits<double>::infinity();
		double max = -std::numeric_limits<double>::infinity();
		for(int j=0; j<8; j++)
		{
			min = std::min(min, vertices[j][i]);
			max = std::max(max, vertices[j][i]);
		}

		if(min > size[i]/2 - b.center[i] || max < - size[i]/2 - b.center[i])
			return false;
	}
		
	// Test separation with box1 axis (3)
	for(int i=1; i<=3; i++)
	{
		Point axis = vertices[i] - vertices[0];
		axis = axis.Normalize();
		
		double low = vertices[0] | axis;
		double hight = vertices[i] | axis;
		double shift = b.center | axis;

		axis = axis*size;

		// min == -max
		double max = std::max(std::max(abs(axis[0] + axis[1] + axis[2]), abs(axis[0] + axis[1] - axis[2])),
			std::max(abs(axis[0] - axis[1] + axis[2]), abs(-axis[0] + axis[1] + axis[2])))/2;

		if(max < low + shift || -max > hight + shift)
			return false;
	}

	// Test separation with box0^box1 axis (6)
	// Not needed for now since all boxes have 
	// at least one axis in common

	return true;
}

bool Box::IntersectSphere(Point p, const double radius) const
{
	p = (p - center)*rotation;

	for(int i=0; i<3; i++)
	{
		double s = size[i]/2;
		if(p[i] > s)
			p[i] -= s;
		else if(p[i] < -s)
			p[i] += s;
		else
			p[i] = 0;
	}

	return p.Norm2() < radius*radius;
}

bool Box::IntersectCylinder(Point c0, Point c1, const double radius) const
{
	c0 = (c0 - center)*rotation;
	c1 = (c1 - center)*rotation;

	// intersect with edges
	std::array<double, 3> e0 = {size[X]/2, 0, 0};
	std::array<double, 3> e1 = {0, size[Y]/2, 0};
	std::array<double, 3> e2 = {0, 0, size[Z]/2};

	if(SegmentSegmentDistance(c0, c1,  -e0 -e1 -e2, -e0 -e1 +e2) < radius)
		return true;
	if(SegmentSegmentDistance(c0, c1,  -e0 +e1 -e2, -e0 +e1 +e2) < radius)
		return true;
	if(SegmentSegmentDistance(c0, c1,  +e0 -e1 -e2, +e0 -e1 +e2) < radius)
		return true;
	if(SegmentSegmentDistance(c0, c1,  +e0 +e1 -e2, +e0 +e1 +e2) < radius)
		return true;

	if(SegmentSegmentDistance(c0, c1,  -e0 -e1 -e2, -e0 +e1 -e2) < radius)
		return true;
	if(SegmentSegmentDistance(c0, c1,  -e0 -e1 +e2, -e0 +e1 +e2) < radius)
		return true;
	if(SegmentSegmentDistance(c0, c1,  +e0 -e1 -e2, +e0 +e1 -e2) < radius)
		return true;
	if(SegmentSegmentDistance(c0, c1,  +e0 -e1 +e2, +e0 +e1 +e2) < radius)
		return true;

	if(SegmentSegmentDistance(c0, c1,  -e0 -e1 -e2, +e0 -e1 -e2) < radius)
		return true;				   			
	if(SegmentSegmentDistance(c0, c1,  -e0 -e1 +e2, +e0 -e1 +e2) < radius)
		return true;				   			
	if(SegmentSegmentDistance(c0, c1,  -e0 +e1 -e2, +e0 +e1 -e2) < radius)
		return true;				   			
	if(SegmentSegmentDistance(c0, c1,  -e0 +e1 +e2, +e0 +e1 +e2) < radius)
		return true;

	// intersect with planes
	std::array<double, 3> axis[] = 
	{
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1}
	};

	Point p;


	p = SegmentPlaneClosestPoint(axis[X], -e0, c0, c1);
	if(abs(p[X]+e0[X]) < radius && abs(p[Y]) < size[Y]/2 && abs(p[Z]) < size[Z]/2)
		return true;

	p = SegmentPlaneClosestPoint(axis[X], +e0, c0, c1);
	if(abs(p[X]-e0[X]) < radius &&  abs(p[Y]) < size[Y]/2 && abs(p[Z]) < size[Z]/2)
		return true;

	p = SegmentPlaneClosestPoint(axis[Y], -e1, c0, c1);
	if(abs(p[Y]+e1[Y]) < radius && abs(p[X]) < size[X]/2 && abs(p[Z]) < size[Z]/2)
		return true;

	p = SegmentPlaneClosestPoint(axis[Y], +e1, c0, c1);
	if(abs(p[Y]-e1[Y]) < radius && abs(p[X]) < size[X]/2 && abs(p[Z]) < size[Z]/2)
		return true;

	p = SegmentPlaneClosestPoint(axis[Z], -e2, c0, c1);
	if(abs(p[Z]+e2[Z]) < radius && abs(p[Y]) < size[Y]/2 && abs(p[X]) < size[X]/2)
		return true;

	p = SegmentPlaneClosestPoint(axis[Z], +e2, c0, c1);
	if(abs(p[Z]-e2[Z]) < radius && abs(p[Y]) < size[Y]/2 && abs(p[X]) < size[X]/2)
		return true;

	return false;
}

// trace = 1+2cos(alpha) so alpha = arccos((trace - 1)/2);
// For the axis :
// http://math.stackexchange.com/questions/178830/cross-product-technique-to-find-the-eigenspaces-of-a-3x3-matrix
// can be improved using the axis to get the exact angle so we can take the middle
// as the box to inflate
// by projecting on planes
Box Box::GetRotationBoundingBox(const Point& size, 
								const Matrix& start, 
								const Matrix& end,
								const double externRadius)
{
	Matrix rotation = end*start.Transpose();

	 // the angle of the rotation
	double alpha = std::acos((rotation.Trace()-1)/2);
	// the diameter of the circle the points are rotating on
	double d = size.Norm() + externRadius; 
	double inflate = alpha > Pi/2 ? d : sin(alpha)*d; // sin(alpha) > 0

	return Box(Point(0), size + inflate, start);
}

Object Box::GetObject() const
{
	Object object;
	std::array<double, 3> vertices[] =
	{
		{0, 0, 0},
		{size[0], 0, 0},
		{0, size[1], 0},
		{0, 0, size[2]},
		{size[0], size[1], 0},
		{size[0], 0, size[2]},
		{0, size[1], size[2]},
		{size[0], size[1], size[2]}
	};

	std::array<int, 3> faces[] = 
	{
		{0, 1, 2},
		{1, 4, 2},
		{0, 1, 3},
		{1, 5, 3},
		{0, 2, 3},
		{2, 6, 3},
		{1, 4, 5},
		{4, 7, 5},
		{3, 5, 6},
		{5, 7, 6},
		{2, 6, 4},
		{4, 6, 7}
	};

	for(int i=0; i<8; i++)
		object.points.push_back(vertices[i]);
	for(int i=0; i<12; i++)
		object.triangles.push_back(faces[i]);

	object.Translate(-(size/2));
	object.Rotate(rotation);
	object.Translate(center);

	return object;
}