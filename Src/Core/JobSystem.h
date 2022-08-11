#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include "Graph.h"
#include "ThreadPool.h"
#include "RingBuffer.h"

class JobSystem;
using JobFunction = std::function<void()>;

#define MAX_JOB_QUEUE_SIZE	(1024)

class Job
{
private:
	Job()
	{
		AddRefCount();
	}

public:
	static Job*	Create(const char* name, JobFunction func)
	{
		Job* job = new Job();
		job->mJobName = std::string(name);
		job->mJobFunction = func;
		return job;
	}

	void	AddChild(Job* job)
	{
		mChildJobs.push_back(job);
		job->AddDependency();
	}

	void	AddRefCount()
	{
		mRefCount.fetch_add(1, std::memory_order_relaxed);
	}

	void	Release()
	{
		if (mRefCount.fetch_sub(1, std::memory_order_release) == 1)
		{
			std::atomic_thread_fence(std::memory_order_acquire);
			delete this;
		}
	}

	void	AddDependency()
	{
		mDependencies.fetch_add(1, std::memory_order_relaxed);
	}

	bool	RemoveDependency()
	{
		if (mDependencies.fetch_sub(1, std::memory_order_release) == 1)
		{
			return true;
		}
		return false;
	}

	int		GetDependencies() const
	{
		return mDependencies;
	}

	void	Execute()
	{
		mJobFunction();
	}

private:
	std::string			mJobName;
	JobFunction 		mJobFunction;
	std::atomic<int>	mDependencies;
	std::atomic<int>	mRefCount;

public:
	std::vector<Job*>	mChildJobs;
};

struct JobGraph : public Graph<Job*>
{
public:
	void CreateParallelJobs(const std::vector<Job*>& _jobs)
	{
		nodes = _jobs;
	}

	void CreateSequentialJobs(const std::vector<Job*>& _jobs)
	{
		nodes = _jobs;
		for (int i = 1; i < (int)_jobs.size(); ++i)
		{
			edges.emplace_back(i - 1, i);
		}
	}
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
		if (v <= 0) return;
		std::lock_guard<std::mutex> ulock(lock);
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
		value -= v;
		cv.wait(ulock, [this]() { return value >= 0; });
	}

	std::mutex				lock;
	std::condition_variable	cv;
	int						value;
};

class JobSystem
{
public:
	JobSystem()
	{
		mTerminate = false;
	}
	~JobSystem()
	{
		Terminate();
		mWorkers.WaitUntilAllThreadExit();
	}

	bool ExecuteGraph(int num_workers, const JobGraph& graph)
	{
		if (!SolveGraphDependencys(graph))
		{
			return false;
		}
		return Execute(num_workers, false);
	}

	bool ExecuteGraphAsync(int num_workers, const JobGraph& graph)
	{
		if (!SolveGraphDependencys(graph))
		{
			return false;
		}
		return Execute(num_workers, true);
	}

	void Terminate()
	{
		mTerminate = true;
		mSemaphore.Signal(MAX_JOB_QUEUE_SIZE);
	}

private:
	bool Execute(int num_workers, bool Async)
	{
		mWorkers.Start(num_workers, [this](int id) { ThreadMain(id); });

		if (!Async)
		{
			WaitOneExecution();
		}
		return true;
	}

	void WaitOneExecution()
	{
		while (!mTerminate && mJobsReminding > 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(16));
			continue;
		}
		return;
	}

	void ThreadMain(int id)
	{
		while (!mTerminate)
		{
			mSemaphore.Wait(1);

			Job* job = mActiveQueue.Pop();
			if (job)
			{
				job->Execute();
				RemoveDependencys(job);
				job->Release();

				--mJobsReminding;
			}
		}
	}

	bool SolveGraphDependencys(const JobGraph& g)
	{
		mJobsReminding += (int)g.nodes.size();
		mActiveQueue.Clear();
		for (size_t i = 0; i < g.edges.size(); ++i)
		{
			auto p = g.edges[i];
			g.nodes[p.first]->AddChild(g.nodes[p.second]);
		}
		int count = 0;
		for (size_t i = 0; i < g.nodes.size(); ++i)
		{
			Job *p = g.nodes[i];
			if (p->GetDependencies() == 0)
			{
				mActiveQueue.Push(p);
				count++;
			}
		}
		mSemaphore.Signal(count);
		return true;
	}

	void RemoveDependencys(Job *job)
	{
		int count = 0;
		const std::vector<Job*>& ChildJobs = job->mChildJobs;
		for (size_t i = 0; i < ChildJobs.size(); ++i)
		{
			Job* job = ChildJobs[i];
			if (job->RemoveDependency())
			{
				mActiveQueue.Push(job);
				count++;
			}
		}
		mSemaphore.Signal(count);
	}
	
private:
	ThreadPool										mWorkers;
	ThreadSafeRingBuffer<Job*, MAX_JOB_QUEUE_SIZE>	mActiveQueue;
	std::atomic<bool>								mTerminate;
	std::atomic<int>								mJobsReminding;
	Semaphore										mSemaphore;
};

