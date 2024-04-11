#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include "Graph.h"
#include "ThreadPool.h"
#include "RingBuffer.h"

namespace Riemann
{
	class JobSystem;
	using JobFunction = std::function<void()>;

	#define MAX_JOB_QUEUE_SIZE	(1024)

	class Job
	{
	public:
		enum Job_status
		{
			Idle,
			WaitDependency,
			Queueing,
			Running,
			Finished,
		};

		Job(const char* name, const JobFunction& func)
		{
			AddRefCount();
			mJobName = std::string(name);
			mJobFunction = func;
			mStatus = Job_status::Idle;
			mDependencies = 0;
			mFreeOnRelease = false;
		}

		static Job* Create(const char* name, const JobFunction& func)
		{
			Job* job = new Job(name, func);
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
			mStatus = Job_status::Running;
			mJobFunction();
			mStatus = Job_status::Finished;
		}

		const std::string& GetName() const
		{
			return mJobName;
		}

	private:
		std::string			mJobName;
		JobFunction 		mJobFunction;
		std::atomic<int>	mDependencies;
		std::atomic<int>	mRefCount;
		bool				mFreeOnRelease;

	public:
		std::atomic<int>	mStatus;
		std::vector<Job*>	mChildJobs;
	};

	struct JobGraph : public Graph<Job*>
	{
	public:
		void AddJob(const char* name, const JobFunction& func)
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
				nodes[p.n0]->AddChild(nodes[p.n1]);
			}
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

		bool ExecuteGraph(const JobGraph& graph)
		{
			if (!Execute(graph))
			{
				return false;
			}
			WaitOneExecution();
			return true;
		}

		bool ExecuteGraphAsync(const JobGraph& graph)
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

				Job* job = mActiveQueue.pop();
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
			mActiveQueue.clear();
			int count = 0;
			for (size_t i = 0; i < g.nodes.size(); ++i)
			{
				Job* job = g.nodes[i];
				if (job->GetDependencies() == 0)
				{
					job->mStatus = Job::Job_status::Queueing;
					mActiveQueue.push(job);
					count++;
				}
				else
				{
					job->mStatus = Job::Job_status::WaitDependency;
				}
			}
			mSemaphore.Signal(count);
			return true;
		}

		void RemoveDependencys(Job* job)
		{
			int count = 0;
			const std::vector<Job*>& ChildJobs = job->mChildJobs;
			for (size_t i = 0; i < ChildJobs.size(); ++i)
			{
				Job* job = ChildJobs[i];
				if (job->RemoveDependency())
				{
					job->mStatus = Job::Job_status::Queueing;
					mActiveQueue.push(job);
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

}