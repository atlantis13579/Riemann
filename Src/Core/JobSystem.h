#pragma once

#include <atomic>
#include <functional>
#include "ThreadPool.h"

using JobFunction = std::function<void()>;

class JobSystem
{
public:
	JobSystem()
	{
		
	}
	~JobSystem()
	{
	}
	
	void Execute(int num_threads)
	{
		mThreads.Start(num_threads, [this] (int id) { ExecuteMain(id); } );
	}
	
	void ExecuteMain(int id)
	{
	}
	
private:
	ThreadPool	mThreads;
};

class Job
{
public:
	Job(const char *name, JobSystem *system, JobFunction& func)
	{
		mJobName = name;
		mSystem = system;
		mJobFunction = func;
	}
	
	void AddDependency(int i)
	{
		mDependencies.fetch_add(i, std::memory_order_relaxed);
	}

	bool RemoveDependency(int i)
	{
		int old_val = mDependencies.fetch_sub(i, std::memory_order_release);
		int new_val = old_val - i;
		return new_val == 0;
	}
	
	void Execute()
	{
		
	}
	
private:
	const char 			*mJobName;
	JobSystem*			mSystem;
	JobFunction 		mJobFunction;
	std::atomic<int>	mDependencies;
};
