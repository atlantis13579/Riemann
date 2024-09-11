#pragma once

#include <stdarg.h>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

class Logger
{
public:
	static void OutputDebugWindow(const char* format, ...)
	{
		va_list args;
		va_start(args, format);
		std::string str = stringPrintfEx(format, args);
		va_end(args);
#ifdef _WIN32
		OutputDebugStringA(str.c_str());
#else
		(void)str.c_str();
#endif
	}

	static std::string stringPrintf(const char* format, ...)
	{
		va_list args;
		va_start(args, format);
		char sss[1024];
		vsnprintf(sss, sizeof(sss), format, args);
		std::string str = std::string(sss);
		va_end(args);
		return str;
	}

	static std::string stringPrintfEx(const char* format, va_list args) {
		char sss[1024];
		vsnprintf(sss, sizeof(sss), format, args);
		std::string str = sss;
		return str;
	}

	void Log(const char* file, long line_no, const char* format, ...)
	{
		
	}
};

#define LOG(_format, ...)		Logger::Log(__FILE__, __LINE__, _format, __VA_ARGS__)
