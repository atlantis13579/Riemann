#pragma once

namespace Maths
{
	template <typename T>
	class TIndex3
	{
	public:
		T a, b, c;

		inline TIndex3<T>()
		{
		}

		explicit inline TIndex3<T>(T _a, T _b, T _c)
		{
			a = _a;
			b = _b;
			c = _c;
		}

		inline TIndex3<T>(const TIndex3<T>& v)
		{
			a = v.a;
			b = v.b;
			c = v.c;
		}

		inline TIndex3<T>& operator=(const TIndex3<T>& v)
		{
			a = v.a;
			b = v.b;
			c = v.c;
			return *this;
		}

		bool		operator== (const TIndex3<T>& rhs) const
		{
			return a == rhs.a && b == rhs.b && c == rhs.c;
		}

		bool		operator!= (const TIndex3<T>& rhs) const
		{
			return a != rhs.a || b != rhs.b || c != rhs.c;
		}

		inline const T* Data() const
		{
			return &a;
		}

		inline T* Data()
		{
			return &a;
		}

		inline const T&	operator[] (int i) const
		{
			const T* p = (const T*)this;
			return p[i];
		}

		inline T& operator[] (int i)
		{
			T* p = (T*)this;
			return p[i];
		}

		int IndexOf(const T v) const
		{
			return (a == v) ? 0 : ((b == v) ? 1 : (c == v ? 2 : -1));
		}

		inline bool Contains(const T v) const
		{
			return a == v || b == v || c == v;
		}

		constexpr static TIndex3<T> Zero()
		{
			return TIndex3<T>(0, 0, 0);
		}
	};

	typedef TIndex3<int>	Index3;
	static_assert(sizeof(Index3) == 12, "sizeof Index3i is not valid");
}

using Index3 = Maths::Index3;