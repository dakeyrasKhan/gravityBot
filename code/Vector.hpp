#include <array>

template<typename T, int L>
inline std::array<T, L> operator+(const std::array<T, L> v0, const std::array<T, L> v1)
{
	std::array<T, L> out;
	for(int i=0; i<L; i++)
		out[i] = v0[i]+v1[i];

	return out;
}

template<typename T, int L>
inline std::array<T, L> operator-(const std::array<T, L> v0, const std::array<T, L> v1)
{
	std::array<T, L> out;
	for(int i=0; i<L; i++)
		out[i] = v0[i]+v1[i];

	return out;
}