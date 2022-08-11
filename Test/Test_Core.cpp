
#include <assert.h>
#include <string>
#include <chrono>
#include "Test.h"
#include "../Src/Core/JobSystem.h"
#include "../Src/Core/ThreadPool.h"
#include "../Src/Core/RingBuffer.h"

void thread_main(int id)
{
	printf("tid = %i\n", id);
}

void TestThreadPool()
{
	printf("Running TestThreadPool\n");
	ThreadPool pool;
	pool.Start(-1, thread_main);

	ThreadSafeRingBuffer<int*, 100> buf;
	return;
}

void job_func(int id)
{
	printf("tid = %i\n", id);
}

void TestJob()
{
	printf("Running TestJob\n");

	JobSystem system;
	system.CreateWorkers(-1);
	JobGraph graph;

	for (int i = 0; i < 12; ++i)
	{
		graph.AddJob(std::to_string(i).c_str(), [i] {
			if (i == 6)
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			else
				std::this_thread::sleep_for(std::chrono::milliseconds(200));
			auto tid = std::this_thread::get_id();
			printf("tid: %d, job : %d\n", *(int*)&tid, i);
		});
	}
	graph.AddEdge(0, 4);
	graph.AddEdge(0, 5);
	graph.AddEdge(0, 6);
	graph.AddEdge(1, 4);
	graph.AddEdge(1, 5);
	graph.AddEdge(1, 6);
	graph.AddEdge(2, 4);
	graph.AddEdge(2, 5);
	graph.AddEdge(2, 6);
	graph.AddEdge(3, 4);
	graph.AddEdge(3, 5);
	graph.AddEdge(3, 6);
	graph.AddEdge(4, 7);
	graph.AddEdge(5, 9);
	graph.AddEdge(6, 8);
	graph.AddEdge(7, 9);
	graph.AddEdge(7, 10);
	graph.AddEdge(9, 11);
	graph.AddEdge(10, 11);

	system.ExecuteGraph(2, graph);
	return;
}

void TestCores()
{
	// TestThreadPool();
	TestJob();
}
