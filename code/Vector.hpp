#pragma once
#include <array>
#include <cmath>
#include <limits>

#define MAX_DEPTH 15
const double Pi = 3.14159265359;
const double EPS = (1./double(1<<MAX_DEPTH));

#ifdef WIN32
const double INFINITY = std::numeric_limits<double>::infinity();
#endif

class Robot;
enum axis { X=0, Y=1, Z=2 };

#define DIM_CONF	8
#define NB_ROT		3
#define ARM_LENGTH  42.
#define ARM1		21.
#define ARM2		21.
#define SPACE		1.

#define ROBOT_ROT	0
#define ROBOT_ARM0	1
#define ROBOT_ARM1	2
#define ROBOT_X		3
#define ROBOT_Z		4
#define BALL_X		5
#define BALL_Y		6
#define BALL_Z		7

inline double mod(const double d0, const double d1)
{
	if(d0>0)
		return d0 - d1*floor(d0/d1);
	else
		return d1*ceil(d0/d1)-d0;
}

template<std::size_t L, int R>
class Array : public std::array<double, L>
{
public:
						Array(const std::array<double, L>& a) : std::array<double, L>(a) { };
						Array() { };
						Array(const double d) { for(auto& x : *this) x = d; };
	Array<3, 0>			ToPoint() const;
	Array<2, 0>			ToCoord() const;
	double				Norm() const;
	double				Norm2() const;
	Array<L, R>			Normalize() const;
	Array<L, R>			setBall(Array<3,0>* ball) {
		(*this)[BALL_X]=(*ball)[X];
		(*this)[BALL_Y]=(*ball)[Y];
		(*this)[BALL_Z]=(*ball)[Z];
		return (*this);
	};
	Array<3, 0>			getBall() const {
		Array<3, 0> res;
		res[X]=(*this)[BALL_X];
		res[Y]=(*this)[BALL_Y];
		res[Z]=(*this)[BALL_Z];
		return res;
	};
};

typedef Array<3, 0> Point;
typedef Array<DIM_CONF, NB_ROT> Vector;
typedef Array<DIM_CONF, NB_ROT> Position;
typedef Array<2, 0> Coord;

Position Random(const std::array<double, DIM_CONF>& neg,
							  const std::array<double, DIM_CONF>& pos,
							  bool with, const Robot& robot);
Position randomCatch(Point p,double,double);

template<std::size_t L, int R>
inline Point Array<L, R>::ToPoint() const
{
	Point p;
	p[X]=(*this)[ROBOT_X];
	p[Z]=(*this)[ROBOT_Z];
	p[Y]=0;
	return p;
}

template<std::size_t L, int R>
inline Coord Array<L, R>::ToCoord() const
{
	Coord c;
	c[0]=(*this)[ROBOT_X];
	c[1]=(*this)[ROBOT_Z];
	return c;
}

template<std::size_t L, int R>
double inline Array<L, R>::Norm2() const
{
	double d = 0;
	for(int i=0; i<R; i++)
	{
		double x = mod((*this)[i],2.*Pi);
		d += x*x;
	}
	for(int i=R; i<L; i++)
		d += (*this)[i]*(*this)[i];


	return d;
}

template<std::size_t L, int R>
double inline Array<L, R>::Norm() const
{
	return sqrt(Norm2());
}

template<std::size_t L, int R>
Array<L, R> inline Array<L, R>::Normalize() const
{
	Array<L, R> p = (*this);
	for(int i=0; i<R; i++)
		p[i] = p[i] - 2*Pi*floor(p[i]/(2*Pi));

	double d = 0;
	for(auto x : p)
		d += x*x;

	d = sqrt(d);

	for(auto& x : p)
		x = x/d;

	return p;
}

template<typename T, std::size_t L>
inline std::array<T, L> operator+(const std::array<T, L> v0, const std::array<T, L> v1)
{
	std::array<T, L> out;
	for(int i=0; i<L; i++)
		out[i] = v0[i]+v1[i];

	return out;
}

template<typename T, std::size_t L>

inline std::array<T, L> abs(const std::array<T, L> v)
{
	std::array<T, L> out;
	for(int i=0; i<L; i++)
		out[i] = abs(v[i]);

	return out;
}

template<typename T, std::size_t L>
inline std::array<T, L> operator+(const std::array<T, L> v, const double d)
{
	std::array<T, L> out;
	for(int i=0; i<L; i++)
		out[i] = v[i]+d;

	return out;
}

template<typename T, std::size_t L>
inline std::array<T, L> operator+(const std::array<T, L> v)
{
	return v;
}

template<typename T, std::size_t L>
inline std::array<T, L> operator-(const std::array<T, L> v0, const std::array<T, L> v1)
{
	std::array<T, L> out;
	for(int i=0; i<L; i++)
		out[i] = v0[i]-v1[i];

	return out;
}

template<typename T, std::size_t L>
inline std::array<T, L> operator-(const std::array<T, L> v)
{
	std::array<T, L> out;
	for(int i=0; i<L; i++)
		out[i] = -v[i];

	return out;
}

template<typename T, std::size_t L>
inline std::array<T, L> operator*(const T d, const std::array<T, L> v)
{
	std::array<T, L> out;
	for(int i=0; i<L; i++)
		out[i] = d*v[i];

	return out;
}

template<typename T, std::size_t L>
inline std::array<T, L> operator*(const std::array<T, L> v, const double d)
{
	return d*v;
}

template<typename T, std::size_t L>
inline std::array<T, L> operator/(const std::array<T, L> v, const double d)
{
	std::array<T, L> out;
	for(int i=0; i<L; i++)
		out[i] = v[i]/d;

	return out;
}

template<typename T, std::size_t L>
inline std::array<T, L> operator/(const std::array<T, L> v0, const std::array<T, L> v1)
{
	std::array<T, L> out;
	for(int i=0; i<L; i++)
		out[i] = v0[i]/v1[i];

	return out;
}

template<typename T, std::size_t L>
inline std::array<T, L> operator*(const std::array<T, L> v0, const std::array<T, L> v1)
{
	std::array<T, L> out;
	for(int i=0; i<L; i++)
		out[i] = v0[i]*v1[i];

	return out;
}

template<typename T, std::size_t L>
inline T operator|(const std::array<T, L> v0, const std::array<T, L> v1)
{
	double out = 0;
	for(int i=0; i<L; i++)
		out += v0[i]*v1[i];

	return out;
}

template<typename T, std::size_t L>
inline void operator+=(std::array<T, L>& v0, const std::array<T, L> v1)
{
	for(int i=0; i<L; i++)
		v0[i] += v1[i];
}

template<typename T, std::size_t L>
inline void operator-=(std::array<T, L>& v0, const std::array<T, L> v1)
{
	for(int i=0; i<L; i++)
		v0[i] -= v1[i];
}

template<typename T, std::size_t L>
inline void operator+=(std::array<T, L>& v0, const T v1)
{
	for(int i=0; i<L; i++)
		v0[i] += v1;
}

template<typename T, std::size_t L>
inline void operator-=(std::array<T, L>& v0, const T v1)
{
	for(int i=0; i<L; i++)
		v0[i] -= v1;
}

inline Point operator^(const Point& p0, const Point& p1)
{
	Point out;
	out[0] = p0[1]*p1[2] - p0[2]*p1[1];
	out[1] = p0[2]*p1[0] - p0[0]*p1[2];
	out[2] = p0[0]*p1[1] - p0[1]*p1[0];

	return out;
}

inline double clamp(const double x, const double a, const double b)
{
    return x < a ? a : (x > b ? b : x);
}

double SegmentSegmentDistance(const Point& a0, const Point& a1, const Point& b0, const Point& b1);
Point SegmentPlaneClosestPoint(const Point& n, const Point& p, const Point& a0, const Point& a1);
