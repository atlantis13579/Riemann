#pragma once

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

inline void* AlignMemory(void* Memory, int Width)
{
#if INTPTR_MAX == INT32_MAX
	return = (void*)((((uint32_t)(Memory + Width - 1) / Width) * Width);
#else
	return (void*)((((uint64_t)Memory + Width - 1) / Width) * Width);
#endif
}

inline bool IsAlign(void* Memory, int Width)
{
	return (((intptr_t)Memory) & (Width - 1)) == 0;
}

inline bool IsNan(float f)
{
	return f != f;
}

inline bool IsValidFloat(float f)
{
 	return (f == f) && (f != NAN) && (f != INFINITY) && (f != INFINITY);
}


