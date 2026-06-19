#pragma once

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <sys/stat.h>

#if defined(_WIN32)
#include <direct.h>
#endif

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

inline bool TestDirectoryExists(const std::string& directoryName)
{
	struct stat info;
	if (stat(directoryName.c_str(), &info) != 0)
	{
		return false;
	}
#if defined(_WIN32)
	return (info.st_mode & _S_IFDIR) != 0;
#else
	return S_ISDIR(info.st_mode);
#endif
}

inline bool TestCreateDirectory(const std::string& directoryName)
{
	if (TestDirectoryExists(directoryName))
	{
		return true;
	}
#if defined(_WIN32)
	return _mkdir(directoryName.c_str()) == 0 || TestDirectoryExists(directoryName);
#else
	return mkdir(directoryName.c_str(), 0755) == 0 || TestDirectoryExists(directoryName);
#endif
}

inline const std::string& TestContentsPath()
{
	static const std::string contentsPath = []() -> std::string
	{
		const char* directories[] =
		{
			"Contents/",
			"../Contents/",
			"../../Contents/",
			"../../../Contents/",
			"../../../../Contents/",
		};

		for (const char* directory : directories)
		{
			if (TestFileExists(std::string(directory) + "TestData/bunny.obj"))
			{
				return directory;
			}
		}

		return "../Contents/";
	}();

	return contentsPath;
}

inline std::string TestDataPath(const char* fileName)
{
	return TestContentsPath() + "TestData/" + fileName;
}

inline const std::string& TestRepositoryPath()
{
	static const std::string repositoryPath = []() -> std::string
	{
		const char* directories[] =
		{
			"",
			"../",
			"../../",
			"../../../",
			"../../../../",
		};

		for (const char* directory : directories)
		{
			if (TestDirectoryExists(std::string(directory) + "Src") &&
				TestDirectoryExists(std::string(directory) + "Test") &&
				TestDirectoryExists(std::string(directory) + "Contents"))
			{
				return directory;
			}
		}

		return "../";
	}();

	return repositoryPath;
}

inline std::string TestOutputPath(const char* fileName)
{
	const std::string directory = TestRepositoryPath() + "TestOutput/";
	TestCreateDirectory(directory);
	return directory + fileName;
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
