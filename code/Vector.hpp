#pragma once
#include <array>
#include <cmath>
#define N_DIM 10
#define N_DIM_ANG 3
#define PI 3.14159265359

typedef std::array<double, N_DIM> Vector;
typedef std::array<double, 3> Point;

typedef Vector Position;

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

double inline norm(Vector p)
{
	for(int i=0; i<N_DIM_ANG; i++)
		p[i] = p[i] - 2*PI*floor(p[i]/(2*PI));

	double d = 0;
	for(auto x : p)
		d += x*x;

	return sqrt(d);
}

Vector inline normalize(Vector p)
{
	for(int i=0; i<N_DIM_ANG; i++)
		p[i] = p[i] - 2*PI*floor(p[i]/(2*PI));

	double d = 0;
	for(auto x : p)
		d += x*x;

	d = sqrt(d);

	for(auto& x : p)
		x = x/d;

	return p;
}
