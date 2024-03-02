#pragma once

#include <chrono>

namespace Riemann
{
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

class WorldTime
{
public:
	WorldTime()
	{
	}

	void	Advanced(double dt)
	{
		if (seconds < 0)
		{
			InitStartTime();		// init
		}
		milliseconds += dt;
		if (milliseconds >= 1.0)
		{
			seconds += (int)milliseconds;
			milliseconds = fmod(milliseconds, 1.0);
			if (fabs(milliseconds < 1e-6))
			{
				milliseconds = 0.0;
			}
		}
	}

	void	InitStartTime()
	{
		seconds = 0;
		milliseconds = 0.0;
		start_time = std::chrono::steady_clock::now();
	}

	double	GetTimeSeconds() const
	{
		const double time = (double)seconds + milliseconds;
		return time;
	}

	double	GetSysTimeSeconds() const
	{
		std::chrono::steady_clock::time_point curr = std::chrono::steady_clock::now();
		const double time_ms = (double)std::chrono::duration_cast<std::chrono::milliseconds>(curr - start_time).count();
		return time_ms * 0.001;
	}

private:
	std::chrono::steady_clock::time_point	start_time;
	int		seconds = -1;
	double	milliseconds = 0.0;
};
}