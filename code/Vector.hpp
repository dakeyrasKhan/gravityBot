#pragma once
#include <array>
#include <cmath>

const double Pi = 3.14159265359;

template<std::size_t L, int R>
class Array : public std::array<double, L>
{
public:
	Array(const std::array<double, L>& a) : std::array<double, L>(a) { };
	Array() { };

	Array<3, 0> ToPoint();
};

typedef Array<03, 0> Point;
typedef Array<10, 3> Vector;
typedef Array<10, 3> Position;

template<std::size_t L, int R>
inline Point Array<L, R>::ToPoint()
{
	Point p;
	for(int i=0; i<3; i++)
		p[i] = (*this)[i];
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

template<std::size_t L, int R>
double inline norm(Array<L, R> p)
{
	for(int i=0; i<R; i++)
		p[i] = p[i] - 2*Pi*floor(p[i]/(2*Pi));

	double d = 0;
	for(auto x : p)
		d += x*x;

	return sqrt(d);
}

template<std::size_t L, int R>
Array<L, R> inline normalize(Array<L, R> p)
{
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
