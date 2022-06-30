#pragma once

#define UNUSED(_x)			(void)(_x)

#ifdef __GNUC__
#define offsetof(_st, _m)	(__builtin_offsetof(_st, _m))
#elif _MSC_VER

#else
#define offsetof(_st, _m)	((size_t)&(((_st *)0)->_m))
#endif
