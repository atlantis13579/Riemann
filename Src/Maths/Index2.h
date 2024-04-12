#pragma once

namespace Maths
{
	template <typename T>
	class TIndex2
	{
	public:
		T a, b;

		inline TIndex2<T>()
		{
		}

		explicit inline TIndex2<T>(T _a, T _b)
		{
			a = _a;
			b = _b;
		}

		inline TIndex2<T>(const TIndex2<T>& v)
		{
			a = v.a;
			b = v.b;
		}

		inline TIndex2<T>& operator=(const TIndex2<T>& v)
		{
			a = v.a;
			b = v.b;
			return *this;
		}

		bool		operator== (const TIndex2<T>& rhs) const
		{
			return a == rhs.a && b == rhs.b;
		}

		bool		operator!= (const TIndex2<T>& rhs) const
		{
			return a != rhs.a || b != rhs.b;
		}

		inline const T* Data() const
		{
			return &a;
		}

		inline T* Data()
		{
			return &a;
		}

		inline const T& operator[] (int i) const
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
			return (a == v) ? 0 : ((b == v) ? 1 : -1);
		}

		inline bool Contains(const T v) const
		{
			return a == v || b == v;
		}

		constexpr static TIndex2<T> Zero()
		{
			return TIndex2<T>(0, 0);
		}
	};

	typedef TIndex2<int>	Index2;
	static_assert(sizeof(Index2) == 8, "sizeof Index2i is not valid");
}

using Index2 = Maths::Index2;