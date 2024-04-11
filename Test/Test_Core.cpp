
#include <assert.h>
#include <string>
#include <chrono>
#include "Test.h"
#include "../Src/Core/Graph.h"
#include "../Src/Core/BitSet.h"
#include "../Src/Core/DynamicArray.h"
#include "../Src/Core/ListSet.h"
#include "../Src/Core/JobSystem.h"
#include "../Src/Core/ThreadPool.h"
#include "../Src/Core/RingBuffer.h"
#include "../Src/Maths/Vector3.h"

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
	BitSet affected(2614);
	for (int i = 0; i <= 2614; ++i)
	{
		affected.Set(i, true);
	}
	affected.Clear();

	affected.Resize(100);
	affected[0] = true;
	affected[10] = true;
	affected[20] = true;
	affected[64] = true;
	affected[63] = true;
	affected[70] = true;
	affected[99] = true;

	std::string str = affected.ToString();

	std::vector<uint32_t> v;
	for (uint32_t i : affected)
	{
		v.push_back(i);
	}

	EXPECT(v.size() == 7);

	return;
}

void TestDynamicArray()
{
	DynamicArray<int> arr1(100);
	DynamicArray<int> arr2(100, 2);
	DynamicArray<int> arr3 = {1, 2, 3};
	EXPECT(arr1.GetSize() == 100);
	EXPECT(arr2[3] == 2);
	EXPECT(arr3[2] == 3);

	DynamicArray<Vector3> arr4;
	arr4.Emplace(1.0f, 2.0f, 3.0f);

	return;
}

void TestListSet()
{
	ListSet<int> s;
	s.SetSize(5);
	EXPECT(s.GetSize() == 5);
	EXPECT(s[0].GetSize() == 0);
	const int set_size = 100;
	for (int i = 0; i < set_size; ++i)
	{
		s[0].Add(i);
		s[1].Add(i);
	}
	EXPECT(s[0].GetSize() == set_size);
	EXPECT(s[1].GetSize() == set_size);
	for (int i = 0; i < set_size; ++i)
	{
		EXPECT(s[0][i] == i);
		EXPECT(s[1][i] == i);
	}
	s[0][1] = 3;
	EXPECT(s[0][1] == 3);

	for (int i = 0; i < s.GetBlockSize() * 2; ++i)
	{
		s[4].Add(i);
	}
	s[4].RemoveAt(s[4].GetSize() - 1);
	s[4].Clear();
	EXPECT(s[4].GetSize() == 0);

	for (int i = 0; i < 20; ++i)
	{
		EXPECT(s[1].RemoveAt(3));
		EXPECT(s[1].RemoveAt(30));
		EXPECT(s[1].RemoveAt(20, false));
	}
	EXPECT(s[1].GetSize() == set_size - 20 * 3);


	for (int i = 0; i < set_size; ++i)
	{
		s[2].Add(i);
	}
	EXPECT(s[2].GetSize() == set_size);

	for (int i = 0; i < 7; ++i)
	{
		s[4].Add(i);
		s[4].RemoveAt(0);
	}
	EXPECT(s[4].GetSize() == 0);

	std::vector<int> v;
	for (auto list : s)
	{
		v.push_back(list.GetSize());
	}
	EXPECT(v[0] == 100);

	s[0].Clear();
	s[1].Clear();
	s[2].Clear();
	s[3].Clear();
	s[4].Clear();

	auto s0 = s[0];
	for (int i = 0; i < 10; ++i)
		s0.Add(i);
	s0.RemoveAt(0, false);
	s0.RemoveAt(1, true);
	s0.RemoveAt(1, true);

	v = s0.ToVector();
	s0 = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

	int sum = 0;
	for (auto it = s0.begin(); it != s0.end(); ++it)
	{
		sum += *it;
	}
	for (auto i : s0)
	{
		sum += i;
	}
	EXPECT(sum == 110);


	return;
}

void TestCores()
{
	TestBitSet();
	// TestThreadPool();
	// TestJob();
	// TestGraph();
	// TestDynamicArray();
	TestListSet();
}
