#pragma once

#include <atomic>
#include <functional>
#include <string>
#include "Graph.h"
#include "ThreadPool.h"
#include "Semaphore.h"
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
		WaitExecution();
	}

	void WaitExecution()
	{
		mWorkers.WaitUntilAllThreadExit();
	}

	bool ExecuteGraph(int num_threads, const JobGraph &graph)
	{
		if (!SolveGraphDependencys(graph))
		{
			return false;
		}
		Execute(num_threads);
		WaitExecution();
		return true;
	}

	bool ExecuteGraphAsync(int num_threads, const JobGraph& graph)
	{
		if (!SolveGraphDependencys(graph))
		{
			return false;
		}
		Execute(num_threads);
		return true;
	}
	
	void Terminate()
	{
		mTerminate = true;
	}
	
private:
	void Execute(int num_threads)
	{
		mWorkers.Start(num_threads, [this](int id) { ExecuteMain(id); });
	}

	void ExecuteMain(int id)
	{
		while (!mTerminate)
		{
			Job *job = mActiveQueue.Pop();
			if (job == nullptr)
			{
				if (mJobsRemindingSingleExecution <= 0)
					break;
				continue;
			}

			job->Execute();
			RemoveDependencys(job);
			job->Release();

			--mJobsRemindingSingleExecution;
		}

		return;
	}

	bool SolveGraphDependencys(const JobGraph& g)
	{
		mJobsRemindingSingleExecution += (int)g.nodes.size();
		mActiveQueue.Clear();
		for (size_t i = 0; i < g.edges.size(); ++i)
		{
			auto p = g.edges[i];
			g.nodes[p.first]->AddChild(g.nodes[p.second]);
		}
		for (size_t i = 0; i < g.nodes.size(); ++i)
		{
			Job *p = g.nodes[i];
			if (p->GetDependencies() == 0)
			{
				mActiveQueue.Push(p);
			}
		}
		return true;
	}

	void RemoveDependencys(Job *job)
	{
		std::vector<Job*>& ChildJobs = job->mChildJobs;
		for (size_t i = 0; i < ChildJobs.size(); ++i)
		{
			Job* job = ChildJobs[i];
			if (job->RemoveDependency())
			{
				mActiveQueue.Push(job);
			}
		}
	}
	
private:
	ThreadPool										mWorkers;
	ThreadSafeRingBuffer<Job*, MAX_JOB_QUEUE_SIZE>	mActiveQueue;
	std::atomic<bool>								mTerminate;
	std::atomic<int>								mJobsRemindingSingleExecution;
};

