#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <set>
#include "Graph.h"
#include "ThreadPool.h"
#include "RingBuffer.h"

class JobSystem;
using JobFunction = std::function<void()>;

#define MAX_JOB_QUEUE_SIZE	(1024)

class Job
{
public:
	enum class Job_status
	{
		WaitDependency,
		Queueing,
		Running,
		Finished,
	};
	
	static Job*	Create(const char* name, JobFunction func)
	{
		Job* job = new Job();
		job->mJobName = std::string(name);
		job->mJobFunction = func;
		job->mFreeOnRelease = true;
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
			if (mFreeOnRelease) delete this;
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
	Job()
	{
		AddRefCount();
		mDependencies = 0;
	}

private:
	std::string			mJobName;
	JobFunction 		mJobFunction;
	std::atomic<int>	mDependencies;
	std::atomic<int>	mRefCount;
	std::atomic<int>	mStatus;
	bool				mFreeOnRelease;

public:
	std::vector<Job*>	mChildJobs;
};

struct JobGraph : public Graph<Job*>
{
public:
	void AddJob(const char* name, JobFunction func)
	{
		Job* job = Job::Create(name, func);
		AddNode(job);
	}

	void AddDependency(int s, int e)
	{
		AddEdge(s, e);
	}

	void BuildDependencys()
	{
		for (size_t i = 0; i < edges.size(); ++i)
		{
			auto p = edges[i];
			nodes[p.first]->AddChild(nodes[p.second]);
		}
	}

	bool HasCycleDFS() const
	{
		struct DFSStack
		{
			size_t i;
			Job* p;
		};

		std::vector<DFSStack> stack;
		stack.resize(nodes.size());
		for (size_t i = 0; i < nodes.size(); ++i)
		{
			Job* p = nodes[i];
			if (p->GetDependencies() != 0)
				continue;

			std::set<Job*> visited;
			int depth = 0;
			stack[depth].i = 0;
			stack[depth].p = p;

			while (depth >= 0)
			{
				size_t i0 = stack[depth].i;
				Job* p0 = stack[depth].p;

				if (i0 >= p0->mChildJobs.size())
				{
					--depth;
					visited.erase(p0);
					if (depth >= 0)
						stack[depth].i += 1;
					continue;
				}
		
				if (p0->mChildJobs[i0]->mChildJobs.empty())
				{
					stack[depth].i += 1;
					continue;
				}
				
				Job* next = p0->mChildJobs[i0];
				++depth;
				stack[depth].i = 0;
				stack[depth].p = next;

				if (visited.find(next) != visited.end())
					return true;
				visited.insert(next);

				continue;
			}
		}
		return false;
	}

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

	void CreateWorkers(int num_workers)
	{
		mWorkers.Stop();
		mWorkers.Start(num_workers, [this](int id) { JobThreadMain(id); });
	}

	bool ExecuteGraph(int num_workers, const JobGraph& graph)
	{
		if (!Execute(graph))
		{
			return false;
		}
		WaitOneExecution();
		return true;
	}

	bool ExecuteGraphAsync(int num_workers, const JobGraph& graph)
	{
		if (!Execute(graph))
		{
			return false;
		}
		return true;
	}

	void Terminate()
	{
		mTerminate = true;
		mSemaphore.Signal(MAX_JOB_QUEUE_SIZE);
	}

private:
	void WaitOneExecution()
	{
		while (!mTerminate && mJobsReminding > 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}
	}

	void JobThreadMain(int id)
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

	bool Execute(const JobGraph& g)
	{
		mJobsReminding += (int)g.nodes.size();
		mActiveQueue.Clear();
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

