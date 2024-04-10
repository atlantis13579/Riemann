#pragma once

#define _USE_MATH_DEFINES

#undef max
#undef min 

#include <math.h>
#include <algorithm>

namespace Maths
{
	template <typename T>
	class TVector3
	{
	public:
		T x, y, z;

		inline TVector3<T>()
		{
		}

		explicit inline TVector3<T>(T k)
		{
			x = y = z = k;
		}

		explicit inline TVector3<T>(T _x, T _y, T _z)
		{
			x = _x;
			y = _y;
			z = _z;
		}

		inline TVector3<T>(const TVector3<T>& v)
		{
			x = v.x;
			y = v.y;
			z = v.z;
		}

		template<typename N>
		inline TVector3<T>(const TVector3<N>& v)
		{
			x = (T)v.x;
			y = (T)v.y;
			z = (T)v.z;
		}

		inline void	SetZero()
		{
			x = y = z = 0.0f;
		}

		inline T Dot(const TVector3<T>& v) const
		{
			return x * v.x + y * v.y + z * v.z;
		}

		inline TVector3<T> Cross(const TVector3<T>& v) const
		{
			return TVector3<T>(y * v.z - v.y * z, z * v.x - v.z * x, x * v.y - v.x * y);
		}

		TVector3<T> Unit() const
		{
			T m = Length();
			return TVector3<T>(x / m, y / m, z / m);
		}

		TVector3<T> SafeUnit() const
		{
			T m = Length();
			if (m <= std::numeric_limits<T>::epsilon())
			{
				return TVector3<T>::Zero();
			}
			return TVector3<T>(x / m, y / m, z / m);
		}

		T Normalize()
		{
			T m = Length();
			x /= m;
			y /= m;
			z /= m;
			return m;
		}

		T SafeNormalize()
		{
			T m = Length();
			if (m <= std::numeric_limits<T>::epsilon())
			{
				x = y = z = (T)0;
				return (T)0;
			}
			x /= m;
			y /= m;
			z /= m;
			return m;
		}

		inline T    Length() const
		{
			return sqrtf(x * x + y * y + z * z);
		}

		inline T    SquareLength() const
		{
			return x * x + y * y + z * z;
		}

		inline TVector3<T>& operator=(const TVector3<T>& v)
		{
			x = v.x;
			y = v.y;
			z = v.z;
			return *this;
		}

		inline TVector3<T> operator+(const TVector3<T>& v) const
		{
			return TVector3<T>(x + v.x, y + v.y, z + v.z);
		}

		inline TVector3<T> operator-(const TVector3<T>& v) const
		{
			return TVector3<T>(x - v.x, y - v.y, z - v.z);
		}

		inline TVector3<T> operator*(const TVector3<T>& v) const
		{
			return TVector3<T>(x * v.x, y * v.y, z * v.z);
		}

		inline TVector3<T> operator/(const TVector3<T>& v) const
		{
			return TVector3<T>(x / v.x, y / v.y, z / v.z);
		}

		inline TVector3<T> operator+(T k) const
		{
			return TVector3<T>(x + k, y + k, z + k);
		}

		inline TVector3<T> operator-(T k) const
		{
			return TVector3<T>(x - k, y - k, z - k);
		}

		inline TVector3<T> operator*(T k) const
		{
			return TVector3<T>(x * k, y * k, z * k);
		}

		inline TVector3<T> operator/(T k) const
		{
			return TVector3<T>(x / k, y / k, z / k);
		}

		inline TVector3<T> operator-() const
		{
			return TVector3<T>(-x, -y, -z);
		}

		inline void	 operator+= (const TVector3<T>& v)
		{
			x += v.x;
			y += v.y;
			z += v.z;
		}

		inline void	 operator-= (const TVector3<T>& v)
		{
			x -= v.x;
			y -= v.y;
			z -= v.z;
		}

		inline void	 operator*= (const TVector3<T>& v)
		{
			x *= v.x;
			y *= v.y;
			z *= v.z;
		}

		inline void	 operator/= (const TVector3<T>& v)
		{
			x /= v.x;
			y /= v.y;
			z /= v.z;
		}

		inline void	 operator+= (T k)
		{
			x += k;
			y += k;
			z += k;
		}

		inline void	 operator-= (T k)
		{
			x -= k;
			y -= k;
			z -= k;
		}

		inline void	 operator*= (T k)
		{
			x *= k;
			y *= k;
			z *= k;
		}

		inline void	 operator/= (T k)
		{
			x /= k;
			y /= k;
			z /= k;
		}

		bool		operator> (T k) const
		{
			return x > k && y > k && z > k;
		}

		bool		operator>= (T k) const
		{
			return x >= k && y >= k && z >= k;
		}

		bool		operator< (T k) const
		{
			return x < k&& y < k&& z < k;
		}

		bool		operator<= (T k) const
		{
			return x <= k && y <= k && z <= k;
		}

		bool		operator> (const TVector3<T>& rhs) const
		{
			return x > rhs.x && y > rhs.y && z > rhs.z;
		}

		bool		operator>= (const TVector3<T>& rhs) const
		{
			return x >= rhs.x && y >= rhs.y && z >= rhs.z;
		}

		bool		operator< (const TVector3<T>& rhs) const
		{
			return x < rhs.x&& y < rhs.y&& z < rhs.z;
		}

		bool		operator<= (const TVector3<T>& rhs) const
		{
			return x <= rhs.x && y <= rhs.y && z <= rhs.z;
		}

		bool		operator== (const TVector3<T>& rhs) const
		{
			return x == rhs.x && y == rhs.y && z == rhs.z;
		}

		bool		operator!= (const TVector3<T>& rhs) const
		{
			return x != rhs.x || y != rhs.y || z != rhs.z;
		}

		inline const T* Data() const
		{
			return &x;
		}

		inline T* Data()
		{
			return &x;
		}

		inline T	operator[] (int i) const
		{
			const T* p = (const T*)this;
			return p[i];
		}

		inline T& operator[] (int i)
		{
			T* p = (T*)this;
			return p[i];
		}

		inline bool Contains(const T v) const
		{
			return x == v || y == v || z == v;
		}

		bool		IsZero() const
		{
			return SquareLength() <= std::numeric_limits<T>::epsilon();
		}

		bool		ParallelTo(const TVector3<T>& v) const
		{
			const TVector3<T>& n = Cross(v);
			T sqr = n.SquareLength();
			return sqr < std::numeric_limits<T>::epsilon();
		}

		int LargestAxis() const
		{
			if (y > x)
			{
				return z > y ? 2 : 1;
			}
			return z > x ? 2 : 0;
		}

		TVector3<T> Project(const TVector3<T>& to) const
		{
			if (to.SquareLength() < std::numeric_limits<T>::epsilon())
			{
				return TVector3<T>::Zero();
			}
			T n = Dot(to);
			T d = to.Dot(to);
			return to * (n / d);
		}

		void DecomposeOrthonormalFrame(TVector3<T>& v0, TVector3<T>& v1, TVector3<T>& v2) const
		{
			v0 = Unit();
			v1 = v0.Cross(TVector3<T>::UnitX());
			if (v1.SquareLength() < 1e-3f)
			{
				v1 = v0.Cross(TVector3<T>::UnitY());
			}
			v2 = v0.Cross(v1);
			return;
		}

		// Duff et al method, from https://graphics.pixar.com/library/OrthonormalB/paper.pdf
		inline void GetPerpVectors(TVector3<T>& OutPerp1, TVector3<T>& OutPerp2) const
		{
			if (z < (T)0)
			{
				T A = (T)1 / ((T)1 - z);
				T B = x * y * A;
				OutPerp1.x = (T)1 - x * x * A;
				OutPerp1.y = -B;
				OutPerp1.z = x;
				OutPerp2.x = B;
				OutPerp2.y = y * y * A - (T)1;
				OutPerp2.z = -y;
			}
			else
			{
				T A = (T)1 / ((T)1 + z);
				T B = -x * y * A;
				OutPerp1.x = (T)1 - x * x * A;
				OutPerp1.y = B;
				OutPerp1.z = -x;
				OutPerp2.x = B;
				OutPerp2.y = (T)1 - y * y * A;
				OutPerp2.z = -y;
			}
		}

		// Duff et al method, from https://graphics.pixar.com/library/OrthonormalB/paper.pdf
		inline TVector3<T> GetPerpVector() const
		{
			TVector3<T> OutPerp1;
			if (z < (T)0)
			{
				T A = (T)1 / ((T)1 - z);
				T B = x * y * A;
				OutPerp1.x = (T)1 - x * x * A;
				OutPerp1.y = -B;
				OutPerp1.z = x;
			}
			else
			{
				T A = (T)1 / ((T)1 + z);
				T B = -x * y * A;
				OutPerp1.x = (T)1 - x * x * A;
				OutPerp1.y = B;
				OutPerp1.z = -x;
			}
			return OutPerp1;
		}

		TVector3<T> Abs() const
		{
			return TVector3<T>(std::abs(x), std::abs(y), std::abs(z));
		}

		TVector3<T> Min(const TVector3<T>& rhs) const
		{
			return TVector3<T>(std::min(x, rhs.x), std::min(y, rhs.y), std::min(z, rhs.z));
		}

		TVector3<T> Max(const TVector3<T>& rhs) const
		{
			return TVector3<T>(std::max(x, rhs.x), std::max(y, rhs.y), std::max(z, rhs.z));
		}

		static TVector3<T> Lerp(const TVector3<T>& start, const TVector3<T>& end, float t)
		{
			return TVector3<T>(
				start.x * (1.0f - t) + end.x * t,
				start.y * (1.0f - t) + end.y * t,
				start.z * (1.0f - t) + end.z * t);
		}

		static TVector3<T> UnitLerp(const TVector3<T>& start, const TVector3<T>& end, float t)
		{
			return Lerp(start, end, t).Unit();
		}

		static TVector3<T> Slerp(const TVector3<T>& start, const TVector3<T>& end, float t)
		{
			T innerp = start.Dot(end);
			innerp = std::min(std::max(-1.0f, innerp), 1.0f);
			float angle = acosf(innerp) * t;
			TVector3<T> base = (end - start * innerp).Unit();
			return start * cosf(angle) + base * sinf(angle);
		}

		constexpr static TVector3<T> InfMin()
		{
			return TVector3<T>(-std::numeric_limits<T>::max(), -std::numeric_limits<T>::max(), -std::numeric_limits<T>::max());
		}

		constexpr static TVector3<T> InfMax()
		{
			return TVector3<T>(std::numeric_limits<T>::max(), std::numeric_limits<T>::max(), std::numeric_limits<T>::max());
		}

		constexpr static TVector3<T> Zero()
		{
			return TVector3<T>(0, 0, 0);
		}

		constexpr static TVector3<T> One()
		{
			return TVector3<T>((T)1, (T)1, (T)1);
		}

		constexpr static TVector3<T> UnitX()
		{
			return TVector3<T>((T)1, 0, 0);
		}

		constexpr static TVector3<T> UnitY()
		{
			return TVector3<T>(0, (T)1, 0);
		}

		constexpr static TVector3<T> UnitZ()
		{
			return TVector3<T>(0, 0, (T)1);
		}

		static TVector3<T> Random()
		{
			return TVector3<T>((T)rand() / RAND_MAX, (T)rand() / RAND_MAX, (T)rand() / RAND_MAX);
		}
	};

	template <typename T>
	inline TVector3<T> operator* (T s, const TVector3<T>& vv)
	{
		return vv * s;
	}

	template <typename T>
	inline T DotProduct(const TVector3<T>& A, const TVector3<T>& B)
	{
		return A.Dot(B);
	}

	template <typename T>
	inline TVector3<T> CrossProduct(const TVector3<T>& A, const TVector3<T>& B)
	{
		return A.Cross(B);
	}

	template <typename T>
	inline TVector3<T> UnitCrossProduct(const TVector3<T>& A, const TVector3<T>& B)
	{
		TVector3<T> C = A.Cross(B);
		return C.Unit();
	}

	template <typename T>
	inline T Determinant(const TVector3<T>& A, const TVector3<T>& B, const TVector3<T>& C)
	{
		return (A.y * B.z * C.x + A.z * B.x * C.y -
			A.x * B.z * C.y - A.y * B.x * C.z +
			A.x * B.y * C.z - A.z * B.y * C.x);
	}

	// A dot (B x C) 
	// This one is equivalent to Determinant, but seems more accuracy in float precision
	template <typename T>
	inline T ScalarTripleProduct(const TVector3<T>& A, const TVector3<T>& B, const TVector3<T>& C)
	{
		return A.Dot(B.Cross(C));
	}

	// A x (B x C)
	template <typename T>
	inline T VectorTripleProduct(const TVector3<T>& A, const TVector3<T>& B, const TVector3<T>& C)
	{
		return (A.Dot(C)) * B - (A.Dot(B)) * C;
	}

	typedef TVector3<float> Vector3;
	typedef TVector3<int>	Vector3i;

	static_assert(sizeof(Vector3) == 12, "sizeof Vector3 is not valid");
	static_assert(sizeof(Vector3i) == 12, "sizeof Vector3 is not valid");
}

using Vector3 = Maths::Vector3;
using Vector3i = Maths::Vector3i;