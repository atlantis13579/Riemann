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

class WorldTime
{
public:
	WorldTime()
	{
		InitStartTime();
	}

	void	Advanced(double dt)
	{
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
		const double time = (double)std::chrono::duration_cast<std::chrono::seconds>(curr - start_time).count();
		return time;
	}

private:
	std::chrono::steady_clock::time_point	start_time;
	int		seconds;
	double	milliseconds;
};
