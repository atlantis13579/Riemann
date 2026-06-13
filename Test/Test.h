#pragma once

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

inline void BreakPoint()
{
	int __nbp = 0;
	(void)__nbp;
}

inline void BreakPoint(const char *src_path, long line_no)
{
	(void)src_path;
	(void)line_no;
	int __nbp = 0;
    (void)__nbp;
}

inline int& TestFailureCount()
{
	static int count = 0;
	return count;
}

inline void TestResetFailures()
{
	TestFailureCount() = 0;
}

inline bool TestFileExists(const std::string& fileName)
{
	std::ifstream file(fileName.c_str(), std::ios::in | std::ios::binary);
	return !!file;
}

inline std::string TestDataPath(const char* fileName)
{
	const char* directories[] =
	{
		"Contents/TestData/",
		"../Contents/TestData/",
		"../../Contents/TestData/",
		"../../../Contents/TestData/",
		"../../../../Contents/TestData/",
	};

	for (const char* directory : directories)
	{
		const std::string path = std::string(directory) + fileName;
		if (TestFileExists(path))
		{
			return path;
		}
	}

	for (const char* directory : directories)
	{
		if (TestFileExists(std::string(directory) + "bunny.obj"))
		{
			return std::string(directory) + fileName;
		}
	}

	return std::string("../Contents/TestData/") + fileName;
}

inline void TestRecordFailure(const char* expr, const char* file, int line)
{
	++TestFailureCount();
	std::printf("Test FAILED at %s Line %d: %s\n", file, line, expr);
	BreakPoint(file, line);
}

#define EXPECT(_expr)		\
	do {	\
		if (!(_expr)) {			\
			TestRecordFailure(#_expr, __FILE__, __LINE__);	\
		}	\
	} while (0)

#define EXPECT_TRUE(_expr)				EXPECT(_expr)
#define EXPECT_FALSE(_expr)				EXPECT(!(_expr))
#define EXPECT_SAME(_v1, _v2)		EXPECT((_v1 - _v2).SquareLength() < 1e-6f)
#define EXPECT_NEAR(_v1, _v2, _eps)	EXPECT(std::fabs((_v1) - (_v2)) <= (_eps))

#define EXPECT1(_expr, _expect)		\
	do {	\
		if (!(_expr)) {			\
			++TestFailureCount(); \
			printf("Test FAILED at %s Line %d, expect = %s\n", __FILE__, __LINE__, std::to_string(_expect).c_str());	\
			BreakPoint(__FILE__, __LINE__);		\
		}	\
	} while (0)

#define EXPECT2(_expr, _val1, _val2)		\
	do {	\
		if (!(_expr)) {			\
			++TestFailureCount(); \
			printf("Test FAILED at %s Line %d, val1 = %s, val2 = %s\n", __FILE__, __LINE__, std::to_string(_val1).c_str(), std::to_string(_val2).c_str());	\
			BreakPoint(__FILE__, __LINE__);		\
		}	\
	} while (0)
