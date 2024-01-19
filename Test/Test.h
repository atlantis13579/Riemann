#pragma once

#include <string>

inline void BreakPoint()
{
	int __nbp = 0;
	(void)__nbp;
}

inline void BreakPoint(const char *src_path, long line_no)
{
	int __nbp = 0;
    (void)__nbp;
}

#define EXPECT(_expr)		\
	if (!(_expr)) {			\
		printf("Test FAILED at %s Line %d\n", __FILE__, __LINE__);	\
		BreakPoint(__FILE__, __LINE__);		\
	}

#define EXPECT1(_expr, _expect)		\
	if (!(_expr)) {			\
		printf("Test FAILED at %s Line %d, expect = %s\n", __FILE__, __LINE__, std::to_string(_expect).c_str());	\
		BreakPoint(__FILE__, __LINE__);		\
	}

#define EXPECT2(_expr, _val1, _val2)		\
	if (!(_expr)) {			\
		printf("Test FAILED at %s Line %d, val1 = %s, val2 = %s\n", __FILE__, __LINE__, std::to_string(_val1).c_str(), std::to_string(_val2).c_str());	\
		BreakPoint(__FILE__, __LINE__);		\
	}
