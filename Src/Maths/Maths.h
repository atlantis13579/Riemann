#pragma once

#include <cmath>
#include <limits>
#include <random>

namespace Maths
{
	#ifndef PI
	#define PI				(3.1415926536f)
	#endif // !PI

	#define PI_2			(6.2831853072f)
	#define PI_4			(12.566370614f)
	#define PI_OVER_2		(1.5707963268f)
	#define PI_OVER_3		(1.0471975512f)
	#define PI_OVER_4		(0.7853981634f)
	#define PI_OVER_6		(0.5235987756f)
	#define PI_OVER_8		(0.3926990817f)
	#define PI_OVER_180		(0.0174532925f)
	#define RAD_TO_DEG		(57.295779513f)
	#define DEG_TO_RAD		(PI_OVER_180)
	#define SQRT_2			(1.4142135624f)
	#define SQRT_3			(1.7320508076f)
	#define	INV_SQRT_2		(0.7071067812f)
	#define	INV_SQRT_3		(0.5773502692f)
	#define SMALL_NUMBER	(1e-3f)
	#define TINY_NUMBER		(1e-6f)

	inline float ToRadian(float degree)
	{
		return degree * DEG_TO_RAD;
	}

	inline float ToDegree(float radian)
	{
		return radian * RAD_TO_DEG;
	}


	inline float Random01()
	{
		return (float)rand() / RAND_MAX;
	}

	inline int RandomInt(int a, int b)
	{
		int p = a + (int)((b - a) * Random01());
		return p;
	}

	inline float RandomFloat(float a, float b)
	{
		float p = a + (b - a) * Random01();
		return p;
	}

	template<typename T>
	inline void RandomShuffe(T *v, int size)
	{
		std::random_device rd;
		std::mt19937 g(rd());
		std::shuffle(v, v + size, g);
	}

	struct SplitMix64RandGen
	{
		typedef unsigned long long uint64;
		uint64 m_state;
		explicit SplitMix64RandGen(uint64 seed)
		{
			m_state = seed;
		}
		explicit SplitMix64RandGen()
		{
			m_state = 0;
		}
		uint64 operator()()
		{
			uint64 z = (m_state += 0x9e3779b97f4a7c15);
			z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9;
			z = (z ^ (z >> 27)) * 0x94d049bb133111eb;
			return z ^ (z >> 31);
		}
	};

	struct Hash
	{
		typedef unsigned long long uint64;

		// the FNV-1a hash algorithm
		// https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
		static uint64 HashBytes(const unsigned char* data, int size, uint64 inSeed = 0xcbf29ce484222325UL)
		{
			uint64 hash = inSeed;
			for (int i = 0; i < size; ++i)
			{
				hash = hash ^ uint64(data[i]);
				hash = hash * 0x100000001b3UL;
			}
			return hash;
		}

		// A 64 bit hash function by Thomas Wang, Jan 1997
		// http://web.archive.org/web/20071223173210/http://www.concentric.net/~Ttwang/tech/inthash.htm
		static uint64 Hash64(uint64 val)
		{
			uint64 hash = val;
			hash = (~hash) + (hash << 21); // hash = (hash << 21) - hash - 1;
			hash = hash ^ (hash >> 24);
			hash = (hash + (hash << 3)) + (hash << 8); // hash * 265
			hash = hash ^ (hash >> 14);
			hash = (hash + (hash << 2)) + (hash << 4); // hash * 21
			hash = hash ^ (hash >> 28);
			hash = hash + (hash << 31);
			return hash;
		}

		uint64 HashStr(const char* str)
		{
			uint64 hash = 14695981039346656037UL;
			for (const char* c = str; *c != 0; ++c)
			{
				hash ^= *c;
				hash = hash * 1099511628211UL;
			}
			return hash;
		}

		template <typename T>
		static void HashCombine(size_t& seed, const T& t)
		{
			std::hash<T> hasher;
			seed ^= hasher(t) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		}

		template <typename T, typename... Ts>
		static void HashCombine(std::size_t& seed, const T& t, Ts... val)
		{
			std::hash<T> hasher;
			seed ^= hasher(t) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

			HashCombine(seed, val...);
		}
	};

	template<typename T>
	bool IsAscendingOrder(T* v, int size)
	{
		if (size <= 1)
		{
			return true;
		}
		for (int i = 1; i < size; ++i)
		{
			if (v[i - 1] > v[i])
			{
				return false;
			}
		}
		return true;
	}

	template<typename T>
	bool IsDescendingOrder(T* v, int size)
	{
		if (size <= 1)
		{
			return true;
		}
		for (int i = 1; i < size; ++i)
		{
			if (v[i - 1] < v[i])
			{
				return false;
			}
		}
		return true;
	}

	template<typename T>
	inline bool IsBetween(const T& val, const T& v0, const T& v1)
	{
		return v0 <= val && val <= v1;
	}

	inline float FloatDiff(float v1, float v2)
	{
		return fabsf(v1 - v2);
	}

	inline bool FloatEqual(float v1, float v2)
	{
		return fabsf(v1 - v2) < 1e-6;
	}

	template<typename T>
	inline void Swap(T& a, T& b)
	{
		T t = a;
		a = b;
		b = t;
	}

	template<typename T>
	inline T	Clamp(const T X, const T Min, const T Max)
	{
		return X < Min ? Min : X < Max ? X : Max;
	}

	template<typename T>
	inline int	Sign(const T x)
	{
		T eps = std::numeric_limits<T>::epsilon();
		return x > eps ? 1 : (x < -eps ? -1 : 0);
	}

	inline float ClampAngle(float angle)
	{
		angle = fmodf(angle, PI_2);
		if (angle < 0.0f)
			angle += PI_2;
		return angle;
	}

	inline float AngleDiff(float angle1, float angle2)
	{
		float diff = fmodf(angle1 - angle2, PI_2);
		return diff;
	}

	template<typename T>
	inline T	LinearInterp(const T v1, const T v2, float mix)
	{
		return v1 * (1.0f - mix) + v2 * mix;
	}

	inline bool	IsFloatInf(float x)
	{
		return x >= std::numeric_limits<float>::max() || x == std::numeric_limits<float>::infinity();
	}

	inline bool IsFloatNan(float f)
	{
		return f != f;
	}

	inline bool IsFloatValid(float f)
	{
		return (f == f) && (f != NAN) && (f != INFINITY) && (f != INFINITY);
	}

	template <typename T>
	inline T Abs(const T x)
	{
		return x >= 0 ? x : -x;
	}

	template <typename T>
	inline T Sqr(const T x)
	{
		return x * x;
	}

	template <typename T>
	inline T Cube(const T x)
	{
		return x * x * x;
	}

	// https://en.wikipedia.org/wiki/Kahan_summation_algorithm
	template <typename T>
	inline T KahanBabuskaSummation(const T* px, int n)
	{
		if (n <= 0)
		{
			return 0;
		}

		T sum = px[0];
		T err = 0;

		for (int i = 1; i < n; ++i)
		{
			const T k = px[i];
			const T m = sum + k;
			err += std::abs(sum) >= std::abs(k) ? sum - m + k : k - m + sum;
			sum = m;
		}
		return sum + err;
	}

	// http://www.matrix67.com/data/InvSqrt.pdf
	inline float InvSqrt(float x) {
		float xhalf = 0.5f * x;
		int i = *(int*)&x;
		i = 0x5f3759df - (i >> 1);
		x = *(float*)&i;
		x = x * (1.5f - xhalf * x * x);
		return x;
	}

	template <typename T>
	inline int SolveQuadratic(const T a, const T b, const T c, T roots[2])		// ax^2 + bx + c = 0
	{
		const T eps = (T)1e-6;
		if (std::fabs(a) < eps)
		{
			if (std::fabs(b) < eps)
				return 0;
			roots[0] = -c / b;
			return 1;
		}

		T discriminant = b * b - 4 * a * c;
		if (discriminant < 0)
		{
			return 0;
		}

		if (std::fabs(discriminant) < eps)
		{
			roots[0] = roots[1] = -b / (2 * a);
			return 1;
		}

		roots[0] = (-b - std::sqrt(discriminant)) / (2 * a);
		roots[1] = (-b + std::sqrt(discriminant)) / (2 * a);
		return 2;
	}

	template <typename T>
	inline T SolveQuadratic(const T a, const T b, const T c)		// ax^2 + bx + c = 0
	{
		const T eps = (T)1e-6;
		if (std::fabs(a) < eps)
		{
			if (std::fabs(b) < eps)
				return std::numeric_limits<T>::max();
			return -c / b;
		}

		T discriminant = b * b - 4 * a * c;
		if (discriminant < 0)
			return std::numeric_limits<T>::max();
		return (-b + std::sqrt(discriminant)) / (2 * a);
	}

	template<typename T>
	inline T SolveCubic(const T a, const T b, const T c)		// x^3 + ax^2 + bx + c = 0
	{
		const T eps = (T)1e-6;
		const T discriminant = a * a - 3 * b;
		if (discriminant <= eps)
			return -a / 3;

		T x = (T)1;
		T val = c + x * (b + x * (a + x));
		if (val < 0)
		{
			x = std::abs(c);
			float t = (T)1 + std::abs(b);
			if (t > x)
				x = t;
			t = (T)1 + std::abs(a);
			if (t > x)
				x = t;
		}

		// Newton's method to find root
		for (int i = 0; i < 16; ++i)
		{
			val = c + x * (b + x * (a + x));
			if (std::abs(val) <= eps)
				return x;

			float dev = b + 2 * x * a + 3 * x * x;
			x -= val / dev;
		}

		return x;
	}

	template<typename T>
	inline T SolveCubic(const T a, const T b, const T c, const T d)		// ax^3 + bx^2 + cx + d = 0
	{
		const T eps = (T)1e-6;
		if (std::abs(a) < eps)
			return SolveQuadratic(b, c, d);
		return SolveCubic(b / a, c / a, d / a);
	}

	// from http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
	inline int BitCount(unsigned int v)
	{
		unsigned int const w = v - ((v >> 1) & 0x55555555);
		unsigned int const x = (w & 0x33333333) + ((w >> 2) & 0x33333333);
		return (((x + (x >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
	}

	inline bool IsPowerOfTwo(int x)
	{
		return x != 0 && (x & (x - 1)) == 0;
	}

	template <typename T>
	inline T Epsilon()
	{
		return (T)0;
	}

	template <>
	inline double Epsilon()
	{
		static const double kEpsilon = 1e-8;
		return kEpsilon;
	}

	template <>
	inline float Epsilon()
	{
		static const float kEpsilon = 1e-6f;
		return kEpsilon;
	}

	template <typename T>
	inline constexpr T Epsilon(T a)
	{
		const T aa = std::abs(a) + (T)1;
		if (aa == std::numeric_limits<T>::infinity())
		{
			return Epsilon<T>();
		}
		else
		{
			return Epsilon<T>() * aa;
		}
	}

	template<typename T>
	T Max(const T& p)
	{
		return p;
	}

	template<typename T, typename ... Ts>
	T Max(const T& p0, Ts... args)
	{
		T p1 = Max(args...);
		return p0 > p1 ? p0 : p1;
	}

	template<typename T>
	T Min(const T& p)
	{
		return p;
	}

	template<typename T, typename ... Ts>
	T Min(const T& p0, const Ts & ... args)
	{
		T p1 = Min(args...);
		return p0 < p1 ? p0 : p1;
	}

	template<typename T>
	inline bool FuzzyEqual(T v1, T v2)
	{
		return v1 == v2;
	}

	template<>
	inline bool FuzzyEqual(float v1, float v2)
	{
		return fabsf(v1 - v2) < 1e-6f;
	}

	template<>
	inline bool FuzzyEqual(double v1, double v2)
	{
		return fabs(v1 - v2) < 1e-8;
	}


	inline bool FuzzyEqual(float v1, float v2, float eplison)
	{
		return fabsf(v1 - v2) < eplison;
	}

	template<typename T>
	inline bool FuzzyZero(T v1)
	{
		return v1 == 0;
	}

	template<>
	inline bool FuzzyZero(float v1)
	{
		return fabsf(v1) < 1e-6f;
	}

	template<>
	inline bool FuzzyZero(double v1)
	{
		return fabs(v1) < 1e-8;
	}
}