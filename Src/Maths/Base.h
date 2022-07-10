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
