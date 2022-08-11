#pragma once

#include <mutex>

class Semaphore
{
public:
	inline			Semaphore()
	{

	}
	inline			~Semaphore()
	{

	}
	inline void		Signal(int v)
	{
		lock.lock();
		value += v;
		if (v > 1)
			cv.notify_all();
		else
			cv.notify_one();
		lock.unlock();
	}
	inline void		Wait(int v)
	{
		std::unique_lock<std::mutex> ulock(lock);
		value -= v;
		cv.wait(ulock, [this]() { return value >= 0; });
	}

	std::mutex				lock;
	std::condition_variable	cv;
	int						value = 0;
};
