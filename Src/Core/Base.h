#pragma once

#include <assert.h>
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
	int __nbp = 0;
	(void)__nbp;
}

#define	ASSERT(_expr)		if(!(_expr)) AssertBreakPoint();
#define	ASSERT_TRUE(_expr)	ASSERT(_expr)
#define	ASSERT_FALSE(_expr)	ASSERT(!(_expr))

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