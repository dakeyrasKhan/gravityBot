#pragma once
#include <array>
#include <cmath>

const double Pi = 3.14159265359;

inline double mod(const double d0, const double d1)
{
	return d0 - d1*floor(d0/d1);
}

template<std::size_t L, int R>
class Array : public std::array<double, L>
{
public:
				Array(const std::array<double, L>& a) : std::array<double, L>(a) { };
				Array() { };
	Array<3, 0>	ToPoint() const;
	double		Norm() const;
	Array<L, R>	Normalize() const;
};

typedef Array<3, 0> Point;
typedef Array<8, 3> Vector;
typedef Array<8, 3> Position;

template<std::size_t L, int R>
inline Point Array<L, R>::ToPoint() const
{
	Point p;
	for(int i=0; i<3; i++)
		p[i] = (*this)[i];
	return p;
}

template<std::size_t L, int R>
double inline Array<L, R>::Norm() const
{
	double d = 0;
	for(int i=0; i<R; i++)
	{
		double x = (*this)[i] - 2*Pi*floor((*this)[i]/(2*Pi));
		d += x*x;
	}
	for(int i=R; i<L; i++)
		d += (*this)[i]*(*this)[i];


	return sqrt(d);
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
inline std::array<T, L> operator-(const std::array<T, L> v0, const std::array<T, L> v1)
{
	std::array<T, L> out;
	for(int i=0; i<L; i++)
		out[i] = v0[i]-v1[i];

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
inline std::array<T, L> operator*(const std::array<T, L> v0, const std::array<T, L> v1)
{
	std::array<T, L> out;
	for(int i=0; i<L; i++)
		out[i] = v0[i]*v1[i];

	return out;
}

template<typename T, std::size_t L>
inline void operator+=(std::array<T, L>& v0, const std::array<T, L> v1)
{
	for(int i=0; i<L; i++)
		v0[i] += v1[i];
}

