#pragma once

#include <chrono>

class Profiler
{
public:
	void Begin()
	{
		tp_begin_ = std::chrono::steady_clock::now();
	}

	float End()
	{
		auto tp_end = std::chrono::steady_clock::now();
		auto diff = std::chrono::duration_cast<std::chrono::microseconds>(tp_end - tp_begin_).count();
		return (float)diff;
	}

private:
	std::chrono::steady_clock::time_point tp_begin_;
};
