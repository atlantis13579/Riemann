#pragma once

#include <assert.h>
#include <codecvt>
#include <vector>

#define UNUSED(_x)			(void)(_x)

#ifdef __clang__
#define OFFSETOF(_st, _m)    ((size_t)&(((_st *)0)->_m))
#elif __GNUC__
#define OFFSETOF(_st, _m)	(__builtin_offsetof(_st, _m))
#elif _MSC_VER
#define OFFSETOF(_st, _m)      offsetof(_st, _m)
#else
#define OFFSETOF(_st, _m)	((size_t)&(((_st *)0)->_m))
#endif

#define SAFE_DELETE(_p)			if (_p) { delete _p; _p = nullptr; }
#define SAFE_DELETE_ARRAY(_p)	if (_p) { delete []_p; _p = nullptr; }

#ifdef _DEBUG
#define DEBUG_CODE(_expr)       _expr;
#else
#define DEBUG_CODE(_expr)
#endif //_DEBUG

inline void AssertBreakPoint()
{
	int _nbp = 0;
	(void)_nbp;
}

#define	ASSERT(_expr)		if(!(_expr)) AssertBreakPoint();
#define	ASSERT_TRUE(_expr)	ASSERT(_expr)
#define	ASSERT_FALSE(_expr)	ASSERT(!(_expr))

template<size_t Width, class T>
inline T Align(T value)
{
	return (value + (Width - 1)) & (~(Width - 1));
}

inline void* AlignMemory(void* Memory, int Width)
{
	return (void*)((((size_t)Memory + Width - 1) / Width) * Width);
}

inline bool IsAlign(void* Memory, int Width)
{
	return (((intptr_t)Memory) & (Width - 1)) == 0;
}

template<typename T, typename ADT>
std::vector<T> ToVector(const ADT& Enumerables)
{
	std::vector<T> Result;
	for (const T& i : Enumerables)
	{
		Result.push_back(i);
	}
	return Result;
}

template<typename T, typename Pred>
void VectorRemove(std::vector<T>& vec, Pred p)
{
	auto it = std::remove_if(vec.begin(), vec.end(), p);
	vec.resize(it - vec.begin());
}

template<typename T>
void VectorRemove(std::vector<T>& vec, const T& val)
{
	for (size_t i = 0; i < vec.size(); ++i)
	{
		if (vec[i] == val)
		{
			vec[i] = vec.back();
			vec.pop_back();
			return;
		}
	}
}

template<typename K, typename V, typename ADT>
const V* MapFind(const ADT& map, const K& key)
{
	auto it = map.find(key);
	if (it != map.end())
	{
		return &it->second;
	}
	return nullptr;
}

template<typename K, typename V, typename ADT>
V* MapFind(ADT& map, const K& key)
{
	auto it = map.find(key);
	if (it != map.end())
	{
		return &it->second;
	}
	return nullptr;
}

// Branchless binary search
template <class ForwardIt, class T, class Compare>
ForwardIt BranchlessLowerBound(ForwardIt first, ForwardIt last, const T& value, Compare comp)
{
	auto length = last - first;
	while (length > 0)
	{
		auto half = length / 2;
		// multiplication (by 1) is needed for GCC to generate CMOW
		first += comp(first[half], value) * (length - half);
		length = half;
	}
	return first;
}
