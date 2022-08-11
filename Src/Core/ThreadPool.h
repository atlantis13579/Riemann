#pragma once

#include <functional>
#include <thread>
#include <vector>

using ThreadFunction = std::function<void(int)>;

class ThreadPool
{
public:
	ThreadPool()
	{
		
	}
	~ThreadPool()
	{
		Stop();
	}
	
	int GetMaxHardwareConcurrency() const
	{
		return (int)std::thread::hardware_concurrency();
	}

	void WaitUntilAllThreadExit()
	{
		if (mThreads.empty())
			return;

		for (size_t i = 0; i < mThreads.size(); ++i)
		{
			std::thread& t = mThreads[i];
			if (t.joinable())
				t.join();
		}

		mThreads.clear();
	}
	
	void Start(int num_threads, ThreadFunction main)
	{
		if (num_threads < 0)
		{
			num_threads = GetMaxHardwareConcurrency();
		}
		mThreads.resize(num_threads);
		
		for (int i = 0; i < num_threads; ++i)
		{
			mThreads[i] = std::thread(main, i + 1);
		}
	}
	
	void Stop()
	{
		WaitUntilAllThreadExit();
	}
	
private:
	std::vector<std::thread> mThreads;
};
