
#include <assert.h>
#include <string>
#include <chrono>
#include "Test.h"
#include "../Src/Core/Graph.h"
#include "../Src/Core/BitSet.h"
#include "../Src/Core/JobSystem.h"
#include "../Src/Core/ThreadPool.h"
#include "../Src/Core/RingBuffer.h"

using namespace Riemann;

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
	system.CreateWorkers(2);
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
	graph.AddDependency(0, 4);
	graph.AddDependency(0, 5);
	graph.AddDependency(0, 6);
	graph.AddDependency(1, 4);
	graph.AddDependency(1, 5);
	graph.AddDependency(1, 6);
	graph.AddDependency(2, 4);
	graph.AddDependency(2, 5);
	graph.AddDependency(2, 6);
	graph.AddDependency(3, 4);
	graph.AddDependency(3, 5);
	graph.AddDependency(3, 6);
	graph.AddDependency(4, 7);
	graph.AddDependency(5, 9);
	graph.AddDependency(6, 8);
	graph.AddDependency(7, 9);
	graph.AddDependency(7, 10);
	graph.AddDependency(9, 11);
	graph.AddDependency(10, 11);
	graph.BuildDependencys();

	EXPECT(!graph.HasCycle());

	system.ExecuteGraph(graph);
	return;
}

void TestGraph()
{
	GraphNE<int, const char*> gr;
	for (int i = 0; i < 9; ++i)
		gr.nodes.push_back(i);
	gr.edges.emplace_back(1, 2, "a");
	gr.edges.emplace_back(3, 2, "b");
	gr.edges.emplace_back(3, 1, "c");
	gr.edges.emplace_back(5, 4, "d");
	gr.edges.emplace_back(2, 4, "e");
	gr.edges.emplace_back(2, 6, "f");

	std::vector<std::vector<int>> islands;
	gr.BuildNodeIslands({false, false, true, false, false,  false, false, false, false}, &islands);
	
	EXPECT(islands.size() == 3);
	EXPECT(islands[0].size() == 3);		// 1, 2, 3
	EXPECT(islands[1].size() == 3);		// 2, 4, 5
	EXPECT(islands[2].size() == 2);		// 2, 6
	for (size_t i = 0; i < islands[0].size(); ++i)
	{
		EXPECT(islands[0][i] == 1 || islands[0][i] == 2 || islands[0][i] == 3);
	}
	for (size_t i = 0; i < islands[1].size(); ++i)
	{
		EXPECT(islands[1][i] == 2 || islands[1][i] == 4 || islands[1][i] == 5);
	}
	for (size_t i = 0; i < islands[2].size(); ++i)
	{
		EXPECT(islands[2][i] == 2 || islands[2][i] == 6);
	}

	std::vector<std::vector<const char*>> islands2;
	gr.BuildEdgeIslands({ false, false, true, false, false,  false, false, false, false }, &islands2);
	EXPECT(islands2.size() == 3);
	EXPECT(islands2[0].size() == 3);	// a, b, c
	EXPECT(islands2[1].size() == 2);	// d, e
	EXPECT(islands2[2].size() == 1);	// f
	return;
}

void TestBitSet()
{
	bit_set affected(2614);
	for (int i = 0; i <= 2614; ++i)
	{
		affected.set(i, true);
	}
	std::vector<uint32_t> str = affected.to_vector();
	return;
}

void TestCores()
{
	TestBitSet();
	// TestThreadPool();
	// TestJob();
	TestGraph();
}
