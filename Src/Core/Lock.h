#pragma once

#include <atomic>
#include <mutex>

namespace Riemann
{
	class SpinLock
	{
	public:
		void lock()
		{
			while (locked.test_and_set(std::memory_order_acquire))
			{
				;
			}
		}
		void unlock()
		{
			locked.clear(std::memory_order_release);
		}

	private:
		std::atomic_flag locked = ATOMIC_FLAG_INIT;
	};

	class Semaphore
	{
	public:
		inline			Semaphore()
		{
			value = 0;
		}

		inline void		Signal(int v)
		{
			if (v <= 0)
				return;
			std::unique_lock<std::mutex> ulock(lock);
			value += v;
			if (v > 1)
				cv.notify_all();
			else
				cv.notify_one();
		}

		inline void		Wait(int v)
		{
			if (v <= 0)
				return;
			std::unique_lock<std::mutex> ulock(lock);
			while (value < v)
			{
				cv.wait(ulock);
			}
			value -= v;
		}

		std::mutex				lock;
		std::condition_variable	cv;
		int						value;
	};

}