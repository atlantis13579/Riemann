
#include <assert.h>
#include <string>
#include <chrono>
#include <algorithm>
#include <limits>
#include <map>
#include <vector>
#include "Test.h"

#include "../Src/Fluid/FluidSimulation.h"
#include "../Src/Fluid/FluidBackend.h"
#include "../Src/Fluid/AerodynamicsSimulation.h"
#include "../Src/Fluid/AerodynamicsRigidCoupling.h"
#include "../Src/Fluid/FluidRigidCoupling.h"
#include "../Src/Fluid/MarchingCubes.h"
#include "../Viewer/MeshRenderer.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Core/Base.h"
#include "../Src/Core/Ref.h"
#include "../Src/Core/Graph.h"
#include "../Src/Core/BitSet.h"
#include "../Src/Core/DynamicArray.h"
#include "../Src/Core/ListSet.h"
#include "../Src/Core/StaticArray.h"
#include "../Src/Core/StaticList.h"
#include "../Src/Core/StaticPool.h"
#include "../Src/Core/StaticStack.h"
#include "../Src/Core/BatchList.h"
#include "../Src/Core/File.h"
#include "../Src/Core/JobSystem.h"
#include "../Src/Core/ThreadPool.h"
#include "../Src/Core/RingBuffer.h"
#include "../Src/Core/PriorityQueue.h"
#include "../Src/Core/SmallSet.h"
#include "../Src/Core/SmallMap.h"
#include "../Src/Core/LinearMap.h"
#include "../Src/Core/LinearSet.h"
#include "../Src/Maths/Maths.h"
#include "../Src/Maths/Vector3.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/RigidBody.h"

using namespace Riemann;

struct TestListNode
{
	int value = 0;
	TestListNode* prev = nullptr;
	TestListNode* next = nullptr;
};

struct TestPriorityNode
{
	int value = 0;

	bool operator<(const TestPriorityNode& rhs) const
	{
		return value < rhs.value;
	}
};

struct TestRefObject : public RefCount<TestRefObject>
{
	~TestRefObject()
	{
		destruct_count++;
	}

	static int destruct_count;
};

int TestRefObject::destruct_count = 0;

struct TestRingItem
{
	int value = 0;
};

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
	printf("Running TestGraph\n");

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

	Graph<int> sparse;
	for (int i = 0; i < 3; ++i)
	{
		sparse.AddNode(i);
	}
	sparse.AddEdge(2, 1);
	sparse.AddEdge(0, 2);
	std::vector<int> entry;
	std::vector<int> adjs;
	sparse.BuildSparseAdjacencyMatrix(entry, adjs);
	EXPECT(sparse.edges[0].n0 == 2 && sparse.edges[0].n1 == 1);
	EXPECT(sparse.edges[1].n0 == 0 && sparse.edges[1].n1 == 2);

	int j0 = 0;
	int j1 = 0;
	EXPECT(Graph<int>::GetAdjacencyList(entry, (int)adjs.size(), 0, &j0, &j1));
	EXPECT(j0 == j1 && adjs[j0] == 2);
	EXPECT(Graph<int>::GetAdjacencyList(entry, (int)adjs.size(), 1, &j0, &j1));
	EXPECT(j0 == j1 && adjs[j0] == 2);
	EXPECT(Graph<int>::GetAdjacencyList(entry, (int)adjs.size(), 2, &j0, &j1));
	EXPECT(j1 - j0 + 1 == 2);
	EXPECT((adjs[j0] == 0 || adjs[j0 + 1] == 0));
	EXPECT((adjs[j0] == 1 || adjs[j0 + 1] == 1));
	return;
}

void TestBitSet()
{
	printf("Running TestBitSet\n");

	BitSet affected(2614);
	for (int i = 0; i <= 2614; ++i)
	{
		affected.set(i, true);
	}
	affected.clear();

	affected.resize(100);
	affected[0] = true;
	affected[10] = true;
	affected[20] = true;
	affected[64] = true;
	affected[63] = true;
	affected[70] = true;
	affected[99] = true;

	std::string str = affected.to_string();

	std::vector<uint32_t> v;
	for (uint32_t i : affected)
	{
		v.push_back(i);
	}

	EXPECT(v.size() == 7);

	return;
}

void TestBitSetOperations()
{
	printf("Running TestBitSetOperations\n");

	BitSet a(130);
	a.set(0, true);
	a.set(64, true);
	a.set(129, true);

	BitSet b(200);
	b.set(129, true);
	b.set(199, true);

	BitSet u = a + b;
	std::vector<uint32_t> uv = u.to_vector();
	EXPECT(u.size() == 200);
	EXPECT((uv == std::vector<uint32_t>{0, 64, 129, 199}));

	BitSet i = a & b;
	EXPECT((i.to_vector() == std::vector<uint32_t>{129}));

	BitSet d = a - b;
	EXPECT((d.to_vector() == std::vector<uint32_t>{0, 64}));

	a += b;
	EXPECT(a.size() == 200);
	EXPECT(a.get(199));

	BitSet c(3);
	c.set(1, true);
	EXPECT(((~c).to_vector() == std::vector<uint32_t>{0, 2}));

	c.set_safe(10, true);
	EXPECT(c.size() == 11);
	EXPECT(c.get_safe(10));
	EXPECT(!c.get_safe(100));

	BitSet shrink(100);
	shrink.set(99, true);
	shrink.resize(10);
	shrink.resize(100);
	EXPECT(!shrink.get(99));

	BitSet masked(3);
	masked.set(1, true);
	BitSet complement = ~masked;
	complement.resize(65);
	EXPECT((complement.to_vector() == std::vector<uint32_t>{0, 2}));
}

void TestDynamicArray()
{
	printf("Running TestDynamicArray\n");

	DynamicArray<int> a = { 1, 3 };
	a.insert_at(1, 2);
	EXPECT((std::vector<int>(a.begin(), a.end()) == std::vector<int>{1, 2, 3}));

	a.insert_at(1, 9, false);
	EXPECT(a.size() == 4);
	EXPECT(a[1] == 9);

	a.remove_at(1, false);
	EXPECT(a.size() == 3);
	EXPECT((std::find(a.begin(), a.end(), 9) == a.end()));

	DynamicArray<int> b = { 4, 5 };
	a.append(b);
	EXPECT((std::vector<int>(a.begin(), a.end()) == std::vector<int>{1, 2, 3, 4, 5}));

	size_t removed = a.remove_if([](int v) { return (v % 2) == 0; });
	EXPECT(removed == 2);
	EXPECT((std::vector<int>(a.begin(), a.end()) == std::vector<int>{1, 3, 5}));
}

void TestStaticContainers()
{
	printf("Running TestStaticContainers\n");

	StaticArray<int, 3> array;
	int* slot = array.add();
	EXPECT(slot != nullptr);
	*slot = 1;
	EXPECT(array.add(2));
	EXPECT(array.emplace(3));
	EXPECT(!array.add(4));
	EXPECT(array.add() == nullptr);
	EXPECT(array.size() == 3);
	EXPECT(!array.empty());

	EXPECT(array.remove_at(1));
	EXPECT(array.size() == 2);
	EXPECT(array.insert_at(1, 9));
	EXPECT((array[0] == 1 && array[1] == 9 && array[2] == 3));

	int sum = 0;
	for (int v : array)
	{
		sum += v;
	}
	EXPECT(sum == 13);

	StaticStack<int, 3> stack;
	EXPECT(stack.empty());
	stack.push(1);
	stack.push(2);
	StaticStack<int, 3> saved;
	saved.restore(stack);
	stack.push(3);
	EXPECT(stack.full());
	EXPECT(stack.pop() == 3);
	EXPECT(stack.pop() == 2);
	stack.restore(saved);
	EXPECT(stack.depth() == 2);
	EXPECT(stack.top() == 2);

	StaticPool<int, 2> pool;
	int* p0 = pool.get();
	int* p1 = pool.get();
	int* p2 = pool.get();
	EXPECT(p0 != nullptr);
	EXPECT(p1 != nullptr);
	EXPECT(p2 == nullptr);
	EXPECT(pool.size() == 2);
}

void TestListAndBatchContainers()
{
	printf("Running TestListAndBatchContainers\n");

	TestListNode n1;
	TestListNode n2;
	n1.value = 1;
	n2.value = 2;

	List<TestListNode> list;
	EXPECT(list.empty());
	list.append(&n1);
	list.append(&n2);
	EXPECT(list.size() == 2);
	EXPECT(list.back() == &n2);
	list.remove(&n2);
	EXPECT(list.size() == 1);
	EXPECT(list.back() == &n1);

	StaticList<TestListNode, 2> static_list;
	EXPECT(static_list.size() == 2);
	TestListNode* s0 = static_list.pop();
	TestListNode* s1 = static_list.pop();
	TestListNode* s2 = static_list.pop();
	EXPECT(s0 != nullptr);
	EXPECT(s1 != nullptr);
	EXPECT(s2 == nullptr);
	EXPECT(static_list.empty());
	static_list.append(s0);
	EXPECT(static_list.size() == 1);

	BatchList<TestListNode> batch;
	batch.init(1, 2);
	TestListNode* b0 = batch.allocate();
	TestListNode* b1 = batch.allocate();
	TestListNode* b2 = batch.allocate();
	EXPECT(b0 != nullptr);
	EXPECT(b1 != nullptr);
	EXPECT(b2 != nullptr);
	EXPECT(batch.size() == 3);
	batch.free(b1);
	EXPECT(batch.size() == 2);
	TestListNode* b3 = batch.allocate();
	EXPECT(b3 == b1);
	EXPECT(batch.size() == 3);
	batch.clear();
	EXPECT(batch.empty());
}

void TestListSet()
{
	printf("Running TestListSet\n");

	ListSet<int> s;
	s.resize(5);
	EXPECT(s.size() == 5);
	EXPECT(s[0].size() == 0);
	const int set_size = 100;
	for (int i = 0; i < set_size; ++i)
	{
		s[0].push_back(i);
		s[1].push_back(i);
	}
	EXPECT(s[0].size() == set_size);
	EXPECT(s[1].size() == set_size);
	for (int i = 0; i < set_size; ++i)
	{
		EXPECT(s[0][i] == i);
		EXPECT(s[1][i] == i);
	}
	s[0][1] = 3;
	EXPECT(s[0][1] == 3);

	for (int i = 0; i < s.GetBlockSize() * 2; ++i)
	{
		s[4].push_back(i);
	}
	s[4].remove_at(s[4].size() - 1);
	s[4].clear();
	EXPECT(s[4].size() == 0);

	for (int i = 0; i < 20; ++i)
	{
		EXPECT(s[1].remove_at(3));
		EXPECT(s[1].remove_at(30));
		EXPECT(s[1].remove_at(20, false));
	}
	EXPECT(s[1].size() == set_size - 20 * 3);


	for (int i = 0; i < set_size; ++i)
	{
		s[2].push_back(i);
	}
	EXPECT(s[2].size() == set_size);

	for (int i = 0; i < 7; ++i)
	{
		s[4].push_back(i);
		s[4].remove_at(0);
	}
	EXPECT(s[4].size() == 0);

	std::vector<int> v;
	for (auto list : s)
	{
		v.push_back(list.size());
	}
	EXPECT(v[0] == 100);

	s[0].clear();
	s[1].clear();
	s[2].clear();
	s[3].clear();
	s[4].clear();

	auto s0 = s[0];
	for (int i = 0; i < 10; ++i)
		s0.push_back(i);
	s0.remove_at(0, false);
	s0.remove_at(1, true);
	s0.remove_at(1, true);

	v = s0.to_vector();
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

	ListSet<int, 1> realloc_s;
	realloc_s.resize(2);
	const int realloc_count = 64;
	for (int i = 0; i < realloc_count; ++i)
	{
		realloc_s[0].push_back(i);
	}

	std::vector<int> iterated;
	for (int value : realloc_s[0])
	{
		iterated.push_back(value);
		realloc_s[1].push_back(value + 1000);
	}
	EXPECT(iterated.size() == realloc_count);
	for (int i = 0; i < realloc_count; ++i)
	{
		EXPECT(iterated[i] == i);
	}


	return;
}

void TestRingBuffer()
{
	printf("Running TestRingBuffer\n");

	RingBuffer<int, 10> buffer; 
	EXPECT(buffer.empty());
	EXPECT(buffer.pop() == 0);
	for (int i = 1; i <= 20; ++i)
		buffer.push(i, false);
	EXPECT(buffer.full());
	int sum = 0;
	for (int v : buffer)
		sum += v;
	EXPECT(sum == 55);
	for (int i = 1; i <= 15; ++i)
		buffer.push(i, true);
	EXPECT(buffer.full());
	EXPECT(buffer.size() == 10);
	sum = 0;
	for (int v : buffer)
		sum += v;
	EXPECT(sum == 105);
	for (int i = 1; i <= 6; ++i)
		EXPECT(buffer.pop() == i + 5);
	EXPECT(buffer.size() == 4);
	sum = 0;
	for (int v : buffer)
		sum += v;
	EXPECT(sum == 54);
	buffer.clear();
	buffer.push(1, false);
	buffer.push(2, false);
	buffer.pop();
	buffer.pop();
	buffer.pop();
	buffer.push(3, false);
	buffer.push(4, false);
	EXPECT(buffer.size() == 2);
	return;
}

void TestPriorityQueue()
{
	printf("Running TestPriorityQueue\n");

	PriorityQueue<int> q;
	EXPECT(q.top() == 0);
	EXPECT(q.pop() == 0);

	std::vector<int> v;
	for (int i = 1; i <= 100; ++i)
	{
		v.push_back(i);
	}
	Maths::RandomShuffe(v.data(), (int)v.size());

	for (size_t i = 0; i < v.size(); ++i)
	{
		q.push(v[i]);
	}

	q.remove(35);
	q.remove(11);
	q.remove(82);
	q.remove(89);
	q.remove(23);

	v.clear();

	while (!q.empty())
	{
		v.push_back(q.pop());
	}

	EXPECT(Maths::IsAscendingOrder(v.data(), (int)v.size()));

	q.push(2);
	q.push(1);
	q.remove(1);
	EXPECT(q.size() == 1);
	EXPECT(q.pop() == 2);
	q.push(3);
	q.remove(3);
	EXPECT(q.empty());

	TestPriorityNode n0{ 3 };
	TestPriorityNode n1{ 1 };
	PriorityPool<TestPriorityNode> pool;
	pool.push(&n0);
	pool.push(&n1);
	EXPECT(pool.top() == &n1);
	EXPECT(pool.pop() == &n1);
	pool.remove(&n0);
	EXPECT(pool.empty());

	TestPriorityNode nodes[4] = { {10}, {1}, {20}, {30} };
	pool.push(&nodes[1]);
	pool.push(&nodes[0]);
	pool.push(&nodes[2]);
	pool.push(&nodes[3]);
	nodes[0].value = 100;
	pool.update(&nodes[0]);
	EXPECT(pool.pop() == &nodes[1]);
	EXPECT(pool.pop() == &nodes[2]);
	EXPECT(pool.pop() == &nodes[3]);
	EXPECT(pool.pop() == &nodes[0]);
}

void TestMapAndSetCapacity()
{
	printf("Running TestMapAndSetCapacity\n");

	SmallSet<int, 2> small_set;
	small_set.insert(1);
	small_set.insert(2);
	small_set.insert(3);
	EXPECT(small_set.size() == 3);
	EXPECT(small_set.contains(1));
	EXPECT(small_set.contains(2));
	EXPECT(small_set.contains(3));

	LinearSet<int, 2> linear_set;
	EXPECT(linear_set.insert(1));
	EXPECT(linear_set.insert(2));
	EXPECT(!linear_set.insert(3));
	EXPECT(linear_set.size() == 2);

	SmallMap<int, int, 2> small_map;
	small_map.insert(1, 10);
	small_map.insert(2, 20);
	small_map.insert(3, 30);
	EXPECT(small_map.size() == 3);
	EXPECT(small_map[1] == 10);
	EXPECT(small_map[2] == 20);
	EXPECT(small_map[3] == 30);
	const SmallMap<int, int, 2>& const_small_map = small_map;
	EXPECT(const_small_map[7] == 0);

	LinearMap<int, int, 2> linear_map;
	EXPECT(linear_map.insert(1, 10));
	EXPECT(linear_map.insert(2, 20));
	EXPECT(!linear_map.insert(3, 30));
	EXPECT(linear_map.size() == 2);
	const LinearMap<int, int, 2>& const_linear_map = linear_map;
	EXPECT(const_linear_map[1] == 10);
	EXPECT(const_linear_map[7] == 0);
}

void TestFileAndBaseUtilities()
{
	printf("Running TestFileAndBaseUtilities\n");

	EXPECT(Align<8>(5u) == 8u);
	char storage[32];
	void* aligned = AlignMemory(storage + 1, 8);
	EXPECT(IsAlign(aligned, 8));

	std::vector<int> src = { 1, 2, 3, 4 };
	VectorRemove(src, 3);
	EXPECT((src == std::vector<int>{1, 2, 4}));
	VectorRemove(src, [](int v) { return v % 2 == 0; });
	EXPECT((src == std::vector<int>{1}));

	std::map<int, int> m;
	m[4] = 40;
	int* found_value = MapFind<int, int>(m, 4);
	int* missing_value = MapFind<int, int>(m, 5);
	EXPECT(found_value != nullptr);
	EXPECT(*found_value == 40);
	EXPECT(missing_value == nullptr);

	int sorted[] = { 1, 3, 5, 7 };
	int* lb = BranchlessLowerBound(sorted, sorted + 4, 4, [](int lhs, int rhs) { return lhs < rhs; });
	EXPECT(lb == sorted + 2);

	const char* path = "codex_core_file_test.bin";
	const unsigned char data[] = { 1, 2, 3, 4, 5 };
	{
		FileWriter writer(path);
		EXPECT(writer.IsLoaded());
		EXPECT(writer.Write(data, sizeof(data)) == sizeof(data));
		EXPECT(writer.Tell() == sizeof(data));
		writer.Close();
		writer.Close();
	}
	{
		FileReader reader(path);
		EXPECT(reader.IsLoaded());
		EXPECT(reader.GetFileSize() == sizeof(data));
		unsigned char read_data[sizeof(data)] = {};
		EXPECT(reader.Read(read_data, sizeof(read_data)) == sizeof(read_data));
		EXPECT(std::equal(read_data, read_data + sizeof(read_data), data));
		reader.Close();
		reader.Close();
	}
	{
		MemoryFile memory(path);
		EXPECT(memory.GetSize() == sizeof(data));
		EXPECT(std::equal(memory.GetData(), memory.GetData() + memory.GetSize(), data));

		MemoryFileAligned<16> aligned_memory(path, 1);
		EXPECT(aligned_memory.GetSize() == sizeof(data) - 1);
		EXPECT(IsAlign(aligned_memory.GetData(), 16));
		EXPECT(aligned_memory.GetData()[0] == data[1]);
	}
	std::remove(path);

	TestRefObject::destruct_count = 0;
	{
		Ref<TestRefObject> ref(new TestRefObject());
		EXPECT(ref->GetRefcount() == 1);
		{
			Ref<TestRefObject> ref_copy = ref;
			EXPECT(ref->GetRefcount() == 2);
			ConstRef<TestRefObject> const_ref(ref);
			EXPECT(ref->GetRefcount() == 3);
		}
		EXPECT(ref->GetRefcount() == 1);
	}
	EXPECT(TestRefObject::destruct_count == 1);

	ThreadSafeRingBuffer<TestRingItem, 2> safe_buffer;
	TestRingItem empty_item = safe_buffer.pop();
	EXPECT(empty_item.value == 0);
	TestRingItem queued_item;
	queued_item.value = 7;
	EXPECT(safe_buffer.push(queued_item, false));
	EXPECT(safe_buffer.pop().value == 7);
}

void TestSmallSet()
{
	printf("Running TestPriorityQueue\n");

	SmallSet<int> s;
	s.insert(1);
	s.insert(2);
	s.insert(3);
	s.insert(1);
	s.erase(1);
	s.insert(2);

	EXPECT(!s.contains(1));
	EXPECT(s.contains(2));
	EXPECT(s.contains(3));
	EXPECT(s.count(1) == 0);
	EXPECT(s.count(2) == 1);
	EXPECT(s.count(3) == 1);

	int sum = 0;
	for (int i : s)
	{
		sum += i;
	}
	EXPECT(sum == 5);
}

void TestSmallMap()
{
	printf("Running TestSmallMap\n");

	SmallMap<int, int> m;
	m[1] = 10;
	m[1] = 11;
	m[2] = 20;
	m[3] = 30;
	m.erase(2);

	EXPECT(m.haskey(1));
	EXPECT(!m.haskey(2));
	EXPECT(m.haskey(3));
	EXPECT(m.count(1) == 1);
	EXPECT(m.count(2) == 0);
	EXPECT(m.count(3) == 1);
	EXPECT(m[1] == 11);
	EXPECT(m[3] == 30);

	int sum = 0;
	for (auto it : m)
	{
		sum += it.key;
	}
	EXPECT(sum == 4);
}

void TestLinearSet()
{
	printf("Running TestLinearSet\n");

    int sum = 0;
    
	LinearSet<int> s;
    for (int i : s)
    {
        (void)i;
        sum += 1;
    }
    EXPECT(sum == 0);
    
	s.insert(1);
	s.insert(2);
	s.insert(3);
	s.insert(1);
	s.erase(1);
	s.insert(2);

	EXPECT(!s.contains(1));
	EXPECT(s.contains(2));
	EXPECT(s.contains(3));
	EXPECT(s.count(1) == 0);
	EXPECT(s.count(2) == 1);
	EXPECT(s.count(3) == 1);

	sum = 0;
	for (int i : s)
	{
		sum += i;
	}
	EXPECT(sum == 5);
}

void TestLinearMap()
{
	printf("Running TestLinearMap\n");

	LinearMap<int, int> m;
	m[1] = 10;
	m[1] = 11;
	m[2] = 20;
	m[3] = 30;
	m.erase(2);

	EXPECT(m.haskey(1));
	EXPECT(!m.haskey(2));
	EXPECT(m.haskey(3));
	EXPECT(m.count(1) == 1);
	EXPECT(m.count(2) == 0);
	EXPECT(m.count(3) == 1);
	EXPECT(m[1] == 11);
	EXPECT(m[3] == 30);

	int sum = 0;
	for (auto it : m)
	{
		sum += it.key;
	}
	EXPECT(sum == 4);
}

void TestFluidSimulation()
{
	printf("Running TestFluidSimulation\n");
	{
		std::vector<Vector3> velocity(1025);
		std::vector<Vector3> delta(1025);
		for (size_t index = 0; index < velocity.size(); ++index)
		{
			const float value = (float)index * 0.013f;
			velocity[index] = Vector3(value, -0.5f * value, 0.25f * value);
			delta[index] = Vector3(0.1f - value * 0.01f, 0.2f, -0.3f);
		}
		std::vector<Vector3> reference(velocity.size());
		Riemann::FluidBackendResult backendResult;
		EXPECT(Riemann::ApplyFluidVelocityDelta(velocity.data(), delta.data(), velocity.size(),
			reference.data(), Riemann::FluidBackend::CPU, false, &backendResult));
		const Riemann::FluidBackend backends[] = { Riemann::FluidBackend::CPU,
			Riemann::FluidBackend::SIMD, Riemann::FluidBackend::CUDA,
			Riemann::FluidBackend::ComputeShader };
		for (Riemann::FluidBackend backend : backends)
		{
			std::vector<Vector3> output(velocity.size());
			const bool available = Riemann::IsFluidBackendAvailable(backend);
			EXPECT(Riemann::ApplyFluidVelocityDelta(velocity.data(), delta.data(), velocity.size(),
				output.data(), backend, true, &backendResult));
			EXPECT(backendResult.RequestedBackend == backend);
			EXPECT(backendResult.Backend == (available ? backend : Riemann::FluidBackend::CPU));
			EXPECT(backendResult.UsedFallback == !available);
			for (size_t index = 0; index < output.size(); ++index)
				EXPECT((output[index] - reference[index]).SquareLength() < 1.0e-10f);
			if (!available)
				EXPECT_FALSE(Riemann::ApplyFluidVelocityDelta(velocity.data(), delta.data(),
					velocity.size(), output.data(), backend, false, nullptr));
		}
	}

	Riemann::FluidSimulationSettings settings;
	settings.BoundsMin = Vector3(0.0f, 0.0f, 0.0f);
	settings.BoundsMax = Vector3(1.0f, 1.0f, 1.0f);
	settings.Resolution = Vector3i(10, 10, 10);
	settings.Gravity = Vector3::Zero();
	settings.ParticleSpacing = 0.1f;
	settings.SmoothingRadius = 0.2f;
	settings.MaxTimeStep = 1.0f / 240.0f;
	settings.MaxSubsteps = 64;

	{
		Riemann::FluidSimulationSettings invalid = settings;
		Riemann::FluidSimulation simulation;
		invalid.RestDensity = std::numeric_limits<float>::quiet_NaN();
		EXPECT_FALSE(simulation.Initialize(invalid));
		invalid = settings;
		invalid.Method = (Riemann::FluidSimulationMethod)255;
		EXPECT_FALSE(simulation.Initialize(invalid));
		invalid = settings;
		invalid.Resolution = Vector3i(2048, 2048, 2048);
		EXPECT_FALSE(simulation.Initialize(invalid));
		invalid = settings;
		invalid.MaxTimeStep = std::numeric_limits<float>::denorm_min();
		EXPECT_FALSE(simulation.Initialize(invalid));
		invalid = settings;
		invalid.ParticleSpacing = 2.0f;
		invalid.SmoothingRadius = 2.0f;
		EXPECT_FALSE(simulation.Initialize(invalid));
		invalid = settings;
		invalid.ParticleSpacing = 1.0e-6f;
		invalid.SmoothingRadius = 2.0e-6f;
		EXPECT_FALSE(simulation.Initialize(invalid));
	}

	settings.Method = Riemann::FluidSimulationMethod::Eulerian;
	Riemann::FluidSimulation eulerian;
	EXPECT(eulerian.Initialize(settings));
	EXPECT(eulerian.GetMethod() == Riemann::FluidSimulationMethod::Eulerian);
	eulerian.AddFluidBox(Vector3(0.2f, 0.2f, 0.2f), Vector3(0.8f, 0.8f, 0.8f),
		Vector3(0.1f, 0.0f, 0.0f));
	const float initialEulerianMass = eulerian.GetTotalMass();
	EXPECT(initialEulerianMass > 0.0f);
	EXPECT(eulerian.SampleDensity(Vector3(0.5f, 0.5f, 0.5f)) > 0.5f * settings.RestDensity);
	const float nan = std::numeric_limits<float>::quiet_NaN();
	EXPECT(eulerian.SampleDensity(Vector3(nan, 0.5f, 0.5f)) == 0.0f);
	EXPECT(eulerian.SampleVelocity(Vector3(0.5f, nan, 0.5f)) == Vector3::Zero());
	eulerian.Step(1.0f / 60.0f);
	EXPECT(std::isfinite(eulerian.GetTotalMass()));
	EXPECT_NEAR(eulerian.GetTotalMass(), initialEulerianMass, 0.001f * initialEulerianMass);
	for (const Vector3& velocity : eulerian.GetGridVelocity())
	{
		EXPECT(std::isfinite(velocity.x) && std::isfinite(velocity.y) && std::isfinite(velocity.z));
	}
	for (int frame = 0; frame < 120; ++frame)
	{
		eulerian.Step(1.0f / 120.0f);
	}
	EXPECT_NEAR(eulerian.GetTotalMass(), initialEulerianMass, 0.005f * initialEulerianMass);
	for (float density : eulerian.GetGridDensity())
	{
		EXPECT(density >= 0.0f && density <= settings.RestDensity + 1.0e-3f);
	}

	Riemann::FluidSimulation projection(settings);
	projection.AddFluidBox(Vector3(0.1f, 0.1f, 0.1f), Vector3(0.5f, 0.9f, 0.9f),
		Vector3(1.0f, 0.0f, 0.0f));
	projection.AddFluidBox(Vector3(0.5f, 0.1f, 0.1f), Vector3(0.9f, 0.9f, 0.9f),
		Vector3(-1.0f, 0.0f, 0.0f));
	const float divergenceBeforeProjection = projection.GetMaxDivergence();
	EXPECT(divergenceBeforeProjection > 1.0f);
	projection.Step(settings.MaxTimeStep);
	EXPECT(projection.GetMaxDivergence() < 0.35f * divergenceBeforeProjection);

	const float massBeforeConversion = eulerian.GetTotalMass();
	EXPECT(eulerian.SetMethod(Riemann::FluidSimulationMethod::Lagrangian, true));
	EXPECT(eulerian.GetMethod() == Riemann::FluidSimulationMethod::Lagrangian);
	EXPECT(!eulerian.GetParticles().empty());
	const float convertedMass = eulerian.GetTotalMass();
	EXPECT_NEAR(convertedMass, massBeforeConversion, 1.0e-4f * massBeforeConversion);
	eulerian.Step(1.0f / 120.0f);
	EXPECT_NEAR(eulerian.GetTotalMass(), convertedMass, 1.0e-3f);
	EXPECT(eulerian.SetMethod(Riemann::FluidSimulationMethod::Eulerian, true));
	EXPECT_NEAR(eulerian.GetTotalMass(), convertedMass, 1.0e-4f * convertedMass);

	settings.Method = Riemann::FluidSimulationMethod::Lagrangian;
	settings.Gravity = Vector3(0.0f, -9.81f, 0.0f);
	Riemann::FluidSimulation lagrangian(settings);
	lagrangian.AddFluidSphere(Vector3(0.5f, 0.65f, 0.5f), 0.25f);
	EXPECT(lagrangian.GetParticles().size() > 8);
	const float initialParticleMass = lagrangian.GetTotalMass();
	lagrangian.Step(1.0f / 30.0f);
	EXPECT_NEAR(lagrangian.GetTotalMass(), initialParticleMass, 1.0e-3f);
	std::vector<float> particleDensityField;
	EXPECT(lagrangian.BuildDensityField(&particleDensityField));
	EXPECT(particleDensityField.size() == 1000);
	EXPECT(*std::max_element(particleDensityField.begin(), particleDensityField.end()) > 0.0f);
	for (const Riemann::FluidParticle& particle : lagrangian.GetParticles())
	{
		EXPECT(particle.Position.x >= settings.BoundsMin.x && particle.Position.x <= settings.BoundsMax.x);
		EXPECT(particle.Position.y >= settings.BoundsMin.y && particle.Position.y <= settings.BoundsMax.y);
		EXPECT(particle.Position.z >= settings.BoundsMin.z && particle.Position.z <= settings.BoundsMax.z);
		EXPECT(std::isfinite(particle.Density) && particle.Density > 0.0f);
		EXPECT(std::isfinite(particle.Velocity.x) && std::isfinite(particle.Velocity.y)
			&& std::isfinite(particle.Velocity.z));
	}

	Riemann::FluidSimulation boundarySimulation(settings);
	boundarySimulation.AddFluidBox(Vector3(0.45f, 0.45f, 0.45f), Vector3(0.55f, 0.55f, 0.55f),
		Vector3(30.0f, -25.0f, 20.0f));
	boundarySimulation.Step(0.25f);
	const float particleRadius = 0.5f * settings.ParticleSpacing;
	for (const Riemann::FluidParticle& particle : boundarySimulation.GetParticles())
	{
		EXPECT(particle.Position.x >= settings.BoundsMin.x + particleRadius);
		EXPECT(particle.Position.y >= settings.BoundsMin.y + particleRadius);
		EXPECT(particle.Position.z >= settings.BoundsMin.z + particleRadius);
		EXPECT(particle.Position.x <= settings.BoundsMax.x - particleRadius);
		EXPECT(particle.Position.y <= settings.BoundsMax.y - particleRadius);
		EXPECT(particle.Position.z <= settings.BoundsMax.z - particleRadius);
	}

	auto GridMomentum = [](const Riemann::FluidSimulation& simulation) -> Vector3
	{
		const Vector3 cellSize = simulation.GetCellSize();
		const double cellVolume = (double)cellSize.x * (double)cellSize.y * (double)cellSize.z;
		const std::vector<float>& density = simulation.GetGridDensity();
		const std::vector<Vector3>& velocity = simulation.GetGridVelocity();
		double x = 0.0, y = 0.0, z = 0.0;
		for (size_t index = 0; index < density.size(); ++index)
		{
			const double mass = (double)density[index] * cellVolume;
			x += mass * (double)velocity[index].x;
			y += mass * (double)velocity[index].y;
			z += mass * (double)velocity[index].z;
		}
		return Vector3((float)x, (float)y, (float)z);
	};
	auto ParticleMomentum = [](const Riemann::FluidSimulation& simulation) -> Vector3
	{
		double x = 0.0, y = 0.0, z = 0.0;
		for (const Riemann::FluidParticle& particle : simulation.GetParticles())
		{
			x += (double)particle.Mass * (double)particle.Velocity.x;
			y += (double)particle.Mass * (double)particle.Velocity.y;
			z += (double)particle.Mass * (double)particle.Velocity.z;
		}
		return Vector3((float)x, (float)y, (float)z);
	};

	Riemann::FluidSimulationSettings impulseSettings;
	impulseSettings.BoundsMin = Vector3::Zero();
	impulseSettings.BoundsMax = Vector3::One();
	impulseSettings.Resolution = Vector3i(10, 10, 10);
	impulseSettings.Gravity = Vector3::Zero();
	impulseSettings.ParticleSpacing = 0.1f;
	impulseSettings.SmoothingRadius = 0.2f;
	const Vector3 requestedImpulse(2.0f, -1.25f, 0.75f);

	impulseSettings.Method = Riemann::FluidSimulationMethod::Eulerian;
	Riemann::FluidSimulation impulseGrid(impulseSettings);
	impulseGrid.AddFluidBox(Vector3(0.2f), Vector3(0.8f));
	const Vector3 gridMomentumBefore = GridMomentum(impulseGrid);
	EXPECT(impulseGrid.ApplyImpulse(Vector3(0.5f), 0.42f, requestedImpulse));
	const Vector3 gridMomentumDelta = GridMomentum(impulseGrid) - gridMomentumBefore;
	EXPECT_NEAR(gridMomentumDelta.x, requestedImpulse.x, 2.0e-4f);
	EXPECT_NEAR(gridMomentumDelta.y, requestedImpulse.y, 2.0e-4f);
	EXPECT_NEAR(gridMomentumDelta.z, requestedImpulse.z, 2.0e-4f);
	EXPECT_FALSE(impulseGrid.ApplyImpulse(Vector3(2.0f), 0.05f, requestedImpulse));
	EXPECT_FALSE(impulseGrid.ApplyImpulse(Vector3(0.5f), 0.0f, requestedImpulse));
	EXPECT_FALSE(impulseGrid.ApplyImpulse(Vector3(nan, 0.5f, 0.5f), 0.2f, requestedImpulse));
	EXPECT_FALSE(impulseGrid.ApplyImpulse(Vector3(0.5f), 0.2f, Vector3(nan, 0.0f, 0.0f)));
	Riemann::FluidSimulation uninitializedImpulse;
	EXPECT_FALSE(uninitializedImpulse.ApplyImpulse(Vector3(0.5f), 0.2f, requestedImpulse));

	impulseSettings.Method = Riemann::FluidSimulationMethod::Lagrangian;
	Riemann::FluidSimulation impulseParticles(impulseSettings);
	impulseParticles.AddFluidBox(Vector3(0.2f), Vector3(0.8f));
	const Vector3 particleMomentumBefore = ParticleMomentum(impulseParticles);
	EXPECT(impulseParticles.ApplyImpulse(Vector3(0.5f), 0.42f, requestedImpulse));
	const Vector3 particleMomentumDelta = ParticleMomentum(impulseParticles) - particleMomentumBefore;
	EXPECT_NEAR(particleMomentumDelta.x, requestedImpulse.x, 2.0e-4f);
	EXPECT_NEAR(particleMomentumDelta.y, requestedImpulse.y, 2.0e-4f);
	EXPECT_NEAR(particleMomentumDelta.z, requestedImpulse.z, 2.0e-4f);
	EXPECT_FALSE(impulseParticles.ApplyImpulse(Vector3(-1.0f), 0.05f, requestedImpulse));
}

void TestFluidRigidCoupling()
{
	printf("Running TestFluidRigidCoupling\n");

	Riemann::FluidSimulationSettings fluidSettings;
	fluidSettings.Method = Riemann::FluidSimulationMethod::Eulerian;
	fluidSettings.BoundsMin = Vector3::Zero();
	fluidSettings.BoundsMax = Vector3::One();
	fluidSettings.Resolution = Vector3i(20, 20, 20);
	fluidSettings.Gravity = Vector3(0.0f, -9.81f, 0.0f);
	fluidSettings.ParticleSpacing = 0.05f;
	fluidSettings.SmoothingRadius = 0.1f;
	Riemann::FluidSimulation fluid(fluidSettings);
	fluid.AddFluidBox(fluidSettings.BoundsMin, fluidSettings.BoundsMax);

	Riemann::PhysicsWorldParam worldSettings;
	worldSettings.gravityAcc = fluidSettings.Gravity;
	Riemann::PhysicsWorld world(worldSettings);
	Riemann::RigidBodyParam bodySettings;
	bodySettings.rigidType = Riemann::RigidType::Dynamic;
	bodySettings.linearDamping = 1.0f;
	bodySettings.angularDamping = 1.0f;
	Riemann::RigidBody* rigid = world.CreateRigidBody(Riemann::GeometryFactory::CreateOBB(
		Vector3(0.5f), Vector3(0.1f)), bodySettings);
	EXPECT(rigid != nullptr);
	if (rigid == nullptr) return;
	Riemann::RigidBodyDynamic* box = rigid->CastDynamic();
	EXPECT(box != nullptr);
	if (box == nullptr) return;
	box->InvMass = 0.2f;

	Riemann::FluidRigidCouplingSettings couplingSettings;
	couplingSettings.SamplesPerAxis = 4;
	couplingSettings.QuadraticDragCoefficient = 0.0f;
	couplingSettings.ViscousDragCoefficient = 0.0f;
	couplingSettings.MaxLinearAcceleration = 1000.0f;
	couplingSettings.MaxAngularAcceleration = 1000.0f;
	Riemann::FluidRigidCoupling* coupling = new Riemann::FluidRigidCoupling(&fluid, couplingSettings);
	EXPECT(coupling->IsValid());
	Riemann::FluidRigidCouplingResult result;
	EXPECT(coupling->Evaluate(box, &result));
	const float boxVolume = 0.2f * 0.2f * 0.2f;
	const float expectedDisplacedMass = fluidSettings.RestDensity * boxVolume;
	const Vector3 expectedBuoyancy = -fluidSettings.Gravity * expectedDisplacedMass;
	EXPECT(result.SampleCount == couplingSettings.SamplesPerAxis * couplingSettings.SamplesPerAxis
		* couplingSettings.SamplesPerAxis);
	EXPECT(result.ActiveSampleCount == result.SampleCount);
	EXPECT_FALSE(result.Clamped);
	EXPECT_NEAR(result.SubmergedVolume, boxVolume, 1.0e-5f);
	EXPECT_NEAR(result.DisplacedMass, expectedDisplacedMass, 1.0e-3f);
	EXPECT_NEAR(result.BuoyancyForce.x, expectedBuoyancy.x, 1.0e-4f);
	EXPECT_NEAR(result.BuoyancyForce.y, expectedBuoyancy.y, 1.0e-3f);
	EXPECT_NEAR(result.BuoyancyForce.z, expectedBuoyancy.z, 1.0e-4f);
	EXPECT(result.BuoyancyForce.Dot(-fluidSettings.Gravity) > 0.0f);
	EXPECT(result.Force.y > 0.0f);

	EXPECT(world.AddForceField(coupling) == coupling);
	world.Simulate();
	EXPECT(box->V.y > 0.0f);
	EXPECT(coupling->GetPendingFeedbackCount() == (size_t)result.ActiveSampleCount);
	world.Simulate();
	EXPECT(box->V.y > 0.0f);
	EXPECT(coupling->GetPendingFeedbackCount() == (size_t)result.ActiveSampleCount);
	EXPECT(coupling->GetLastFeedbackFailureCount() == 0);
	world.Reset();
	EXPECT(world.GetForceFieldCount() == 1);
}

void TestAerodynamicsRigidCoupling()
{
	printf("Running TestAerodynamicsRigidCoupling\n");

	Riemann::AerodynamicsSimulationSettings gasSettings;
	gasSettings.Method = Riemann::FluidSimulationMethod::Eulerian;
	gasSettings.BoundsMin = Vector3::Zero();
	gasSettings.BoundsMax = Vector3::One();
	gasSettings.Resolution = Vector3i(12, 12, 12);
	gasSettings.Gravity = Vector3::Zero();
	gasSettings.SpeedOfSound = 20.0f;
	gasSettings.ParticleSpacing = 1.0f / 12.0f;
	gasSettings.SmoothingRadius = 0.15f;
	Riemann::AerodynamicsSimulation ambientGas(gasSettings);
	EXPECT(ambientGas.IsInitialized());

	Riemann::PhysicsWorldParam worldSettings;
	worldSettings.gravityAcc = Vector3::Zero();
	Riemann::PhysicsWorld evaluationWorld(worldSettings);
	Riemann::RigidBodyParam bodySettings;
	bodySettings.rigidType = Riemann::RigidType::Dynamic;
	bodySettings.linearDamping = 1.0f;
	bodySettings.angularDamping = 1.0f;
	bodySettings.sleepThreshold = 0.0f;
	Riemann::RigidBodyDynamic* box = evaluationWorld.CreateRigidBody(
		Riemann::GeometryFactory::CreateOBB(Vector3(0.5f), Vector3(0.1f)), bodySettings)->CastDynamic();
	Riemann::RigidBodyDynamic* sphere = evaluationWorld.CreateRigidBody(
		Riemann::GeometryFactory::CreateSphere(Vector3(0.5f), 0.1f), bodySettings)->CastDynamic();
	EXPECT(box != nullptr && sphere != nullptr);
	if (box == nullptr || sphere == nullptr)
	{
		evaluationWorld.Reset();
		return;
	}
	box->InvMass = 1.0f;
	sphere->InvMass = 1.0f;

	Riemann::AerodynamicsRigidCouplingSettings couplingSettings;
	couplingSettings.SurfaceSamplesPerAxis = 4;
	couplingSettings.MaxLinearAcceleration = 100000.0f;
	couplingSettings.MaxAngularAcceleration = 100000.0f;
	Riemann::AerodynamicsRigidCoupling ambientCoupling(&ambientGas, couplingSettings);
	EXPECT(ambientCoupling.IsValid());
	Riemann::AerodynamicsRigidCouplingResult boxResult;
	Riemann::AerodynamicsRigidCouplingResult sphereResult;
	EXPECT(ambientCoupling.Evaluate(box, &boxResult));
	EXPECT(ambientCoupling.Evaluate(sphere, &sphereResult));
	EXPECT(boxResult.SampleCount == 6 * couplingSettings.SurfaceSamplesPerAxis
		* couplingSettings.SurfaceSamplesPerAxis);
	EXPECT(sphereResult.SampleCount == 2 * couplingSettings.SurfaceSamplesPerAxis
		* couplingSettings.SurfaceSamplesPerAxis);
	EXPECT(boxResult.Force.SquareLength() < 1.0e-8f);
	EXPECT(boxResult.Torque.SquareLength() < 1.0e-8f);
	EXPECT(sphereResult.Force.SquareLength() < 1.0e-8f);
	EXPECT(sphereResult.Torque.SquareLength() < 1.0e-8f);

	Riemann::AerodynamicsSimulation pressureGradient(gasSettings);
	pressureGradient.SetDensityBox(Vector3::Zero(), Vector3(0.5f, 1.0f, 1.0f),
		1.35f * gasSettings.AmbientDensity);
	pressureGradient.SetDensityBox(Vector3(0.5f, 0.0f, 0.0f), Vector3::One(),
		0.75f * gasSettings.AmbientDensity);
	Riemann::AerodynamicsRigidCoupling gradientCoupling(&pressureGradient, couplingSettings);
	box->SetLinearVelocity(Vector3::Zero());
	box->SetAngularVelocity(Vector3::Zero());
	EXPECT(gradientCoupling.Evaluate(box, &boxResult));
	EXPECT(boxResult.Force.x > 0.0f);
	EXPECT(std::fabs(boxResult.Force.y) < 1.0e-3f * std::fabs(boxResult.Force.x));
	EXPECT(std::fabs(boxResult.Force.z) < 1.0e-3f * std::fabs(boxResult.Force.x));

	box->SetLinearVelocity(Vector3(5.0f, 0.0f, 0.0f));
	box->SetAngularVelocity(Vector3::Zero());
	EXPECT(ambientCoupling.Evaluate(box, &boxResult));
	EXPECT(boxResult.Force.x < 0.0f);
	EXPECT(boxResult.Force.Dot(box->GetLinearVelocity()) < 0.0f);
	box->SetLinearVelocity(Vector3::Zero());
	box->SetAngularVelocity(Vector3(0.0f, 0.0f, 5.0f));
	EXPECT(ambientCoupling.Evaluate(box, &boxResult));
	EXPECT(boxResult.Torque.Dot(box->GetAngularVelocity()) < 0.0f);

	Riemann::AerodynamicsRigidCouplingSettings clampSettings = couplingSettings;
	clampSettings.MaxLinearAcceleration = 0.05f;
	clampSettings.MaxAngularAcceleration = 0.05f;
	Riemann::AerodynamicsRigidCoupling clampedCoupling(&ambientGas, clampSettings);
	box->SetLinearVelocity(Vector3(50.0f, 0.0f, 0.0f));
	box->SetAngularVelocity(Vector3::Zero());
	EXPECT(clampedCoupling.Evaluate(box, &boxResult));
	EXPECT(boxResult.Clamped);
	EXPECT(boxResult.Force.Length() * box->GetInverseMass()
		<= clampSettings.MaxLinearAcceleration * 1.001f);
	evaluationWorld.Reset();

	Riemann::PhysicsWorld integrationWorld(worldSettings);
	Riemann::RigidBody* integrationRigid = integrationWorld.CreateRigidBody(
		Riemann::GeometryFactory::CreateOBB(Vector3(0.5f), Vector3(0.08f)), bodySettings);
	EXPECT(integrationRigid != nullptr);
	if (integrationRigid != nullptr)
	{
		Riemann::RigidBodyDynamic* integrationBox = integrationRigid->CastDynamic();
		EXPECT(integrationBox != nullptr);
		if (integrationBox != nullptr)
		{
			integrationBox->InvMass = 1.0f;
			Riemann::AerodynamicsRigidCoupling* worldCoupling =
				new Riemann::AerodynamicsRigidCoupling(&pressureGradient, couplingSettings);
			EXPECT(integrationWorld.AddForceField(worldCoupling) == worldCoupling);
			integrationWorld.Simulate();
			EXPECT(integrationBox->V.x > 0.0f);
			EXPECT(std::fabs(integrationBox->V.y) < 1.0e-4f);
			EXPECT(std::fabs(integrationBox->V.z) < 1.0e-4f);
		}
	}
	integrationWorld.Reset();

	Riemann::AerodynamicsSimulation collisionGas(gasSettings);
	collisionGas.SetDensityBox(Vector3::Zero(), Vector3(0.40f, 1.0f, 1.0f),
		1.30f * gasSettings.AmbientDensity);
	collisionGas.SetDensityBox(Vector3(0.60f, 0.0f, 0.0f), Vector3::One(),
		1.30f * gasSettings.AmbientDensity);
	collisionGas.SetDensityBox(Vector3(0.42f, 0.0f, 0.0f), Vector3(0.58f, 1.0f, 1.0f),
		0.60f * gasSettings.AmbientDensity);
	Riemann::RigidBodyDynamic* collisionBox = nullptr;
	Riemann::RigidBodyDynamic* collisionSphere = nullptr;
	int drivenContactCount = 0;
	Riemann::PhysicsWorldParam collisionWorldSettings = worldSettings;
	collisionWorldSettings.onContact = [&](const Riemann::PhysicsContactInfo& contact)
	{
		if ((contact.BodyA == collisionBox && contact.BodyB == collisionSphere)
			|| (contact.BodyA == collisionSphere && contact.BodyB == collisionBox))
		{
			++drivenContactCount;
		}
	};
	Riemann::PhysicsWorld collisionWorld(collisionWorldSettings);
	collisionBox = collisionWorld.CreateRigidBody(Riemann::GeometryFactory::CreateOBB(
		Vector3(0.46f, 0.5f, 0.47f), Vector3(0.08f, 0.09f, 0.09f)), bodySettings)->CastDynamic();
	collisionSphere = collisionWorld.CreateRigidBody(Riemann::GeometryFactory::CreateSphere(
		Vector3(0.62f, 0.5f, 0.53f), 0.075f), bodySettings)->CastDynamic();
	EXPECT(collisionBox != nullptr && collisionSphere != nullptr);
	if (collisionBox != nullptr && collisionSphere != nullptr)
	{
		collisionBox->InvMass = 1.0f;
		collisionSphere->InvMass = 1.0f;
		Riemann::AerodynamicsRigidCouplingSettings drivenSettings = couplingSettings;
		drivenSettings.MaxLinearAcceleration = 10.0f;
		drivenSettings.MaxAngularAcceleration = 20.0f;
		Riemann::AerodynamicsRigidCoupling* drivenCoupling =
			new Riemann::AerodynamicsRigidCoupling(&collisionGas, drivenSettings);
		EXPECT(collisionWorld.AddForceField(drivenCoupling) == drivenCoupling);
		const float initialBoxX = collisionBox->X.x;
		const float initialSphereX = collisionSphere->X.x;
		for (int frame = 0; frame < 90 && drivenContactCount == 0; ++frame)
		{
			collisionWorld.Simulate();
			collisionGas.Step(1.0f / 60.0f);
		}
		EXPECT(collisionBox->X.x > initialBoxX);
		EXPECT(collisionSphere->X.x < initialSphereX);
		EXPECT(drivenContactCount > 0);
	}
	collisionWorld.Reset();
}

void TestAerodynamicsSimulation()
{
	printf("Running TestAerodynamicsSimulation\n");

	Riemann::AerodynamicsSimulationSettings settings;
	settings.BoundsMin = Vector3(0.0f, 0.0f, 0.0f);
	settings.BoundsMax = Vector3(1.0f, 1.0f, 1.0f);
	settings.Resolution = Vector3i(8, 8, 8);
	settings.SpeedOfSound = 5.0f;
	settings.ParticleSpacing = 0.125f;
	settings.SmoothingRadius = 0.25f;
	settings.MaxTimeStep = 1.0f / 240.0f;
	settings.MaxSubsteps = 128;

	{
		Riemann::AerodynamicsSimulationSettings invalid = settings;
		invalid.AmbientDensity = std::numeric_limits<float>::quiet_NaN();
		Riemann::AerodynamicsSimulation simulation;
		EXPECT_FALSE(simulation.Initialize(invalid));
		invalid = settings;
		invalid.Method = (Riemann::FluidSimulationMethod)255;
		EXPECT_FALSE(simulation.Initialize(invalid));
		invalid = settings;
		invalid.AcousticCFL = 1.5f;
		EXPECT_FALSE(simulation.Initialize(invalid));
		invalid = settings;
		invalid.ParticleSpacing = 0.2f;
		EXPECT_FALSE(simulation.Initialize(invalid));
		invalid = settings;
		invalid.SmoothingRadius = 0.125f;
		EXPECT_FALSE(simulation.Initialize(invalid));
		std::vector<float> densityField;
		EXPECT_FALSE(simulation.BuildDensityField(&densityField));
		EXPECT_FALSE(simulation.BuildDensityField(nullptr));
	}
	{
		Riemann::AerodynamicsSimulationSettings lattice = settings;
		lattice.Method = Riemann::FluidSimulationMethod::Lagrangian;
		lattice.Resolution = Vector3i(3, 3, 3);
		lattice.ParticleSpacing = 0.26f;
		lattice.SmoothingRadius = 0.55f;
		Riemann::AerodynamicsSimulation simulation(lattice);
		EXPECT(simulation.IsInitialized());
		EXPECT(simulation.GetParticles().size() == 27);
		EXPECT_NEAR(simulation.GetTotalMass(), lattice.AmbientDensity, 1.0e-4f);
		for (const Riemann::AirParticle& particle : simulation.GetParticles())
		{
			EXPECT(particle.Position.x <= lattice.BoundsMax.x - 0.5f * lattice.ParticleSpacing);
			EXPECT(particle.Position.y <= lattice.BoundsMax.y - 0.5f * lattice.ParticleSpacing);
			EXPECT(particle.Position.z <= lattice.BoundsMax.z - 0.5f * lattice.ParticleSpacing);
		}
	}
	{
		Riemann::AerodynamicsSimulationSettings stiff = settings;
		stiff.SpeedOfSound = 1000000.0f;
		stiff.MaxSubsteps = 4;
		Riemann::AerodynamicsSimulation simulation(stiff);
		EXPECT(simulation.IsInitialized());
		simulation.Step(1.0e-4f);
		EXPECT(simulation.GetPendingTime() > 0.0);
		for (float density : simulation.GetGridDensity()) EXPECT(std::isfinite(density));
	}
	{
		Riemann::AerodynamicsSimulationSettings thin = settings;
		thin.Method = Riemann::FluidSimulationMethod::Lagrangian;
		thin.AmbientDensity = 1.0e-6f;
		Riemann::AerodynamicsSimulation simulation(thin);
		EXPECT(simulation.IsInitialized());
		simulation.SetVelocityBox(thin.BoundsMin, thin.BoundsMax, Vector3(0.5f, 0.0f, 0.0f));
		EXPECT(simulation.SetMethod(Riemann::FluidSimulationMethod::Eulerian, true));
		EXPECT(simulation.Sample(Vector3(0.5f, 0.5f, 0.5f)).Velocity.x > 0.25f);
	}

	settings.Method = Riemann::FluidSimulationMethod::Eulerian;
	Riemann::AerodynamicsSimulation eulerian(settings);
	EXPECT(eulerian.IsInitialized());
	EXPECT(eulerian.GetMethod() == Riemann::FluidSimulationMethod::Eulerian);
	const Riemann::AerodynamicSample ambient = eulerian.Sample(Vector3(0.5f, 0.5f, 0.5f));
	EXPECT_NEAR(ambient.Density, settings.AmbientDensity, 1.0e-4f);
	EXPECT_NEAR(ambient.Pressure, settings.AmbientPressure, 1.0e-2f);
	EXPECT(ambient.PressureGradient.SquareLength() < 1.0e-4f);
	std::vector<Riemann::RenderMeshDesc> gasMeshes;
	EXPECT(Riemann::BuildGasDensityMeshes(eulerian, "gas_test", &gasMeshes));
	EXPECT(gasMeshes.size() == 2);
	EXPECT(gasMeshes[0].Id == "gas_test.compressed");
	EXPECT(gasMeshes[1].Id == "gas_test.rarefied");
	for (const Riemann::RenderMeshDesc& mesh : gasMeshes)
	{
		EXPECT(mesh.Vertices.empty() && mesh.Indices.empty());
		EXPECT(mesh.BlendMode == Riemann::RenderBlendMode::Additive);
		EXPECT(mesh.Unlit && !mesh.CastShadow);
	}
	Riemann::Renderer* nullRenderer = Riemann::Renderer::CreateNullRenderer();
	EXPECT(nullRenderer != nullptr);
	if (nullRenderer != nullptr)
	{
		EXPECT(nullRenderer->AddMesh(gasMeshes[0]));
		const Vertex1 dynamicVertices[3] =
		{
			Vertex1(Vector3(0.0f, 0.0f, 0.0f), Vector3::UnitY()),
			Vertex1(Vector3(1.0f, 0.0f, 0.0f), Vector3::UnitY()),
			Vertex1(Vector3(0.0f, 1.0f, 0.0f), Vector3::UnitY()),
		};
		const uint32_t dynamicIndices[3] = { 0, 1, 2 };
		EXPECT(nullRenderer->UpdateMeshGeometry(gasMeshes[0].Id.c_str(),
			dynamicVertices, 3, dynamicIndices, 3));
		EXPECT(nullRenderer->UpdateVertices(gasMeshes[0].Id.c_str(), dynamicVertices, 3));
		EXPECT_FALSE(nullRenderer->UpdateMeshGeometry(gasMeshes[0].Id.c_str(),
			dynamicVertices, 3, dynamicIndices, 2));
		EXPECT(nullRenderer->UpdateMeshGeometry(gasMeshes[0].Id.c_str(), nullptr, 0, nullptr, 0));
		delete nullRenderer;
	}

	eulerian.SetDensityBox(Vector3(0.2f, 0.2f, 0.2f), Vector3(0.5f, 0.8f, 0.8f),
		2.0f * settings.AmbientDensity);
	std::vector<float> eulerianDensityField;
	EXPECT(eulerian.BuildDensityField(&eulerianDensityField));
	EXPECT(eulerianDensityField == eulerian.GetGridDensity());
	EXPECT(Riemann::BuildGasDensityMeshes(eulerian, "gas_test", &gasMeshes));
	EXPECT(!gasMeshes[0].Indices.empty());
	EXPECT(gasMeshes[1].Vertices.empty() && gasMeshes[1].Indices.empty());
	for (uint32_t index : gasMeshes[0].Indices) EXPECT(index < gasMeshes[0].Vertices.size());
	for (const Vertex1& vertex : gasMeshes[0].Vertices)
	{
		const Vector3 worldPosition = gasMeshes[0].WorldTransform * vertex.Pos;
		EXPECT(worldPosition.x >= settings.BoundsMin.x - 1.0e-4f
			&& worldPosition.x <= settings.BoundsMax.x + 1.0e-4f);
		EXPECT(worldPosition.y >= settings.BoundsMin.y - 1.0e-4f
			&& worldPosition.y <= settings.BoundsMax.y + 1.0e-4f);
		EXPECT(worldPosition.z >= settings.BoundsMin.z - 1.0e-4f
			&& worldPosition.z <= settings.BoundsMax.z + 1.0e-4f);
	}
	Riemann::AerodynamicsSimulation boundaryCompressedGas(settings);
	boundaryCompressedGas.SetDensityBox(settings.BoundsMin, Vector3(0.3f, 0.3f, 0.3f),
		2.0f * settings.AmbientDensity);
	EXPECT(Riemann::BuildGasDensityMeshes(boundaryCompressedGas, "gas_boundary", &gasMeshes));
	EXPECT(!gasMeshes[0].Indices.empty());
	for (const Vertex1& vertex : gasMeshes[0].Vertices)
	{
		const Vector3 worldPosition = gasMeshes[0].WorldTransform * vertex.Pos;
		EXPECT(worldPosition.x >= settings.BoundsMin.x - 1.0e-4f
			&& worldPosition.x <= settings.BoundsMax.x + 1.0e-4f);
		EXPECT(worldPosition.y >= settings.BoundsMin.y - 1.0e-4f
			&& worldPosition.y <= settings.BoundsMax.y + 1.0e-4f);
		EXPECT(worldPosition.z >= settings.BoundsMin.z - 1.0e-4f
			&& worldPosition.z <= settings.BoundsMax.z + 1.0e-4f);
	}
	Riemann::AerodynamicsSimulation rarefiedGas(settings);
	rarefiedGas.SetDensitySphere(settings.BoundsMin, 0.3f,
		0.5f * settings.AmbientDensity);
	EXPECT(Riemann::BuildGasDensityMeshes(rarefiedGas, "gas_test", &gasMeshes));
	EXPECT(gasMeshes[0].Vertices.empty() && gasMeshes[0].Indices.empty());
	EXPECT(!gasMeshes[1].Indices.empty());
	for (const Vertex1& vertex : gasMeshes[1].Vertices)
	{
		const Vector3 worldPosition = gasMeshes[1].WorldTransform * vertex.Pos;
		EXPECT(worldPosition.x >= settings.BoundsMin.x - 1.0e-4f
			&& worldPosition.x <= settings.BoundsMax.x + 1.0e-4f);
		EXPECT(worldPosition.y >= settings.BoundsMin.y - 1.0e-4f
			&& worldPosition.y <= settings.BoundsMax.y + 1.0e-4f);
		EXPECT(worldPosition.z >= settings.BoundsMin.z - 1.0e-4f
			&& worldPosition.z <= settings.BoundsMax.z + 1.0e-4f);
	}
	Riemann::GasDensityVisualizationSettings invalidVisualization;
	invalidVisualization.RelativeThreshold = 0.0f;
	EXPECT_FALSE(Riemann::BuildGasDensityMeshes(rarefiedGas, "gas_test", &gasMeshes,
		invalidVisualization));
	EXPECT(eulerian.Sample(Vector3(0.35f, 0.5f, 0.5f)).Pressure > settings.AmbientPressure);
	const Riemann::AerodynamicSample edge = eulerian.Sample(Vector3(0.52f, 0.5f, 0.5f));
	EXPECT(edge.PressureGradient.x < 0.0f);
	const Riemann::AerodynamicForceResult pressureForce = eulerian.ComputeSphereForce(
		Vector3(0.52f, 0.5f, 0.5f), 0.08f, Vector3::Zero());
	EXPECT(pressureForce.Force.x > 0.0f);

	const float initialMass = eulerian.GetTotalMass();
	eulerian.Step(settings.MaxTimeStep);
	EXPECT(eulerian.Sample(Vector3(0.52f, 0.5f, 0.5f)).Velocity.x > 0.0f);
	EXPECT(eulerian.GetTotalMomentum().SquareLength() < 1.0e-4f);
	for (int frame = 1; frame < 20; ++frame) eulerian.Step(1.0f / 120.0f);
	EXPECT_NEAR(eulerian.GetTotalMass(), initialMass, 0.002f * initialMass);
	for (size_t index = 0; index < eulerian.GetGridDensity().size(); ++index)
	{
		EXPECT(std::isfinite(eulerian.GetGridDensity()[index]) && eulerian.GetGridDensity()[index] >= 0.0f);
		EXPECT(std::isfinite(eulerian.GetGridPressure()[index]) && eulerian.GetGridPressure()[index] >= 0.0f);
	}

	Riemann::AerodynamicsSimulation wind(settings);
	wind.SetVelocityBox(settings.BoundsMin, settings.BoundsMax, Vector3(2.0f, 0.0f, 0.0f));
	const Riemann::AerodynamicForceResult drag = wind.ComputeSphereForce(
		Vector3(0.5f, 0.5f, 0.5f), 0.1f, Vector3::Zero());
	EXPECT(drag.Force.x > 0.0f);
	EXPECT(std::fabs(drag.Force.y) < 1.0e-5f && std::fabs(drag.Force.z) < 1.0e-5f);

	std::vector<Riemann::AerodynamicSurfaceSample> closedSurface(6);
	const Vector3 normals[6] = { Vector3::UnitX(), -Vector3::UnitX(), Vector3::UnitY(),
		-Vector3::UnitY(), Vector3::UnitZ(), -Vector3::UnitZ() };
	for (int index = 0; index < 6; ++index)
	{
		closedSurface[(size_t)index].Normal = normals[index];
		closedSurface[(size_t)index].Position = Vector3(0.5f, 0.5f, 0.5f) + normals[index] * 0.1f;
		closedSurface[(size_t)index].Area = 0.04f;
	}
	const Riemann::AerodynamicForceResult symmetric = Riemann::AerodynamicsSimulation(settings)
		.ComputeSurfaceForce(Vector3(0.5f, 0.5f, 0.5f), closedSurface);
	EXPECT(symmetric.Force.SquareLength() < 1.0e-6f);
	EXPECT(symmetric.Torque.SquareLength() < 1.0e-6f);

	const float massBeforeConversion = eulerian.GetTotalMass();
	const float densityBeforeConversion = eulerian.Sample(Vector3(0.35f, 0.5f, 0.5f)).Density;
	EXPECT(eulerian.SetMethod(Riemann::FluidSimulationMethod::Lagrangian, true));
	EXPECT(!eulerian.GetParticles().empty());
	EXPECT_NEAR(eulerian.GetTotalMass(), massBeforeConversion, 0.002f * massBeforeConversion);
	EXPECT_NEAR(eulerian.Sample(Vector3(0.35f, 0.5f, 0.5f)).Density,
		densityBeforeConversion, 0.25f * settings.AmbientDensity);
	eulerian.Step(1.0f / 240.0f);
	for (const Riemann::AirParticle& particle : eulerian.GetParticles())
	{
		EXPECT(std::isfinite(particle.Density) && particle.Density > 0.0f);
		EXPECT(std::isfinite(particle.Velocity.x) && std::isfinite(particle.Velocity.y)
			&& std::isfinite(particle.Velocity.z));
	}
	const float particleMass = eulerian.GetTotalMass();
	const Riemann::AerodynamicSample beforeGridConversion = eulerian.Sample(Vector3(0.35f, 0.5f, 0.5f));
	EXPECT(eulerian.SetMethod(Riemann::FluidSimulationMethod::Eulerian, true));
	EXPECT_NEAR(eulerian.GetTotalMass(), particleMass, 0.002f * particleMass);
	const Riemann::AerodynamicSample afterGridConversion = eulerian.Sample(Vector3(0.35f, 0.5f, 0.5f));
	EXPECT_NEAR(afterGridConversion.Density, beforeGridConversion.Density,
		0.25f * settings.AmbientDensity);
	EXPECT((afterGridConversion.Velocity - beforeGridConversion.Velocity).Length() < 0.5f);
	EXPECT_NEAR(afterGridConversion.Pressure, beforeGridConversion.Pressure,
		settings.AmbientDensity * settings.SpeedOfSound * settings.SpeedOfSound);

	settings.Method = Riemann::FluidSimulationMethod::Lagrangian;
	Riemann::AerodynamicsSimulation lagrangian(settings);
	EXPECT(lagrangian.GetParticles().size() == 512);
	std::vector<float> lagrangianDensityField;
	EXPECT(lagrangian.BuildDensityField(&lagrangianDensityField));
	EXPECT(lagrangianDensityField.size() == 512);
	for (float density : lagrangianDensityField)
		EXPECT(std::isfinite(density) && density > 0.0f);
	for (int frame = 0; frame < 5; ++frame) lagrangian.Step(1.0f / 240.0f);
	float maxAmbientSpeed = 0.0f;
	for (const Riemann::AirParticle& particle : lagrangian.GetParticles())
		maxAmbientSpeed = std::max(maxAmbientSpeed, particle.Velocity.Length());
	EXPECT(maxAmbientSpeed < 1.0e-3f);
	EXPECT(lagrangian.GetTotalMomentum().SquareLength() < 1.0e-6f);

	Riemann::AerodynamicsSimulation particlePulse(settings);
	particlePulse.SetDensitySphere(Vector3(0.5f, 0.5f, 0.5f), 0.22f,
		2.0f * settings.AmbientDensity);
	EXPECT(particlePulse.BuildDensityField(&lagrangianDensityField));
	EXPECT(*std::max_element(lagrangianDensityField.begin(), lagrangianDensityField.end())
		> 1.08f * settings.AmbientDensity);
	EXPECT(Riemann::BuildGasDensityMeshes(particlePulse, "gas_sph", &gasMeshes));
	EXPECT(!gasMeshes[0].Indices.empty());
	particlePulse.Step(settings.MaxTimeStep);
	float radialMomentum = 0.0f;
	for (const Riemann::AirParticle& particle : particlePulse.GetParticles())
		radialMomentum += (particle.Position - Vector3(0.5f, 0.5f, 0.5f)).Dot(particle.Velocity)
			* particle.Mass;
	EXPECT(radialMomentum > 0.0f);
	EXPECT(particlePulse.GetTotalMomentum().SquareLength() < 1.0e-5f);

	Riemann::AerodynamicsSimulationSettings viscousSettings = settings;
	viscousSettings.DynamicViscosity = 100.0f;
	viscousSettings.ArtificialViscosity = 0.0f;
	Riemann::AerodynamicsSimulation viscous(viscousSettings);
	viscous.SetVelocityBox(Vector3(0.0f, 0.0f, 0.0f), Vector3(0.5f, 1.0f, 1.0f),
		Vector3(1.0f, 0.0f, 0.0f));
	viscous.SetVelocityBox(Vector3(0.5f, 0.0f, 0.0f), Vector3(1.0f, 1.0f, 1.0f),
		Vector3(-1.0f, 0.0f, 0.0f));
	float initialKineticEnergy = 0.0f;
	for (const Riemann::AirParticle& particle : viscous.GetParticles())
		initialKineticEnergy += 0.5f * particle.Mass * particle.Velocity.SquareLength();
	viscous.Step(0.001f);
	float finalKineticEnergy = 0.0f;
	for (const Riemann::AirParticle& particle : viscous.GetParticles())
	{
		EXPECT(std::isfinite(particle.Velocity.x) && std::isfinite(particle.Velocity.y)
			&& std::isfinite(particle.Velocity.z));
		finalKineticEnergy += 0.5f * particle.Mass * particle.Velocity.SquareLength();
	}
	EXPECT(finalKineticEnergy <= 1.01f * initialKineticEnergy);

	Riemann::AerodynamicsSimulationSettings disturbanceSettings = settings;
	disturbanceSettings.Method = Riemann::FluidSimulationMethod::Eulerian;
	Riemann::AerodynamicsSimulation disturbanceGas(disturbanceSettings);
	EXPECT(disturbanceGas.ApplyImpulse(Vector3(0.25f, 0.5f, 0.5f), 0.18f,
		Vector3(0.1f, 0.0f, 0.0f)));
}

void TestMarchingCubes()
{
	printf("Running TestMarchingCubes\n");

	Riemann::MarchingCubesInput input;
	input.Origin = Vector3::Zero();
	input.Spacing = Vector3::One();
	input.Resolution = Vector3i(4, 4, 4);
	input.IsoValue = 1.5f;
	input.HighValuesInside = true;
	std::vector<float> planeValues(64, 0.0f);
	for (int z = 0; z < 4; ++z)
	for (int y = 0; y < 4; ++y)
	for (int x = 0; x < 4; ++x)
		planeValues[(size_t)x + 4 * ((size_t)y + 4 * (size_t)z)] = (float)x;
	input.Values = planeValues.data();
	input.ValueCount = planeValues.size();
	Riemann::FluidSurfaceMesh plane;
	EXPECT(Riemann::MarchingCubes::Extract(input, &plane));
	EXPECT(plane.Positions.size() == 16);
	EXPECT(plane.Normals.size() == plane.Positions.size());
	EXPECT(plane.Indices.size() == 54);
	for (size_t index = 0; index < plane.Positions.size(); ++index)
	{
		EXPECT_NEAR(plane.Positions[index].x, 1.5f, 1.0e-5f);
		EXPECT(plane.Normals[index].Dot(-Vector3::UnitX()) > 0.99f);
	}

	const int corners[8][3] =
	{
		{ 0, 0, 0 }, { 1, 0, 0 }, { 1, 0, 1 }, { 0, 0, 1 },
		{ 0, 1, 0 }, { 1, 1, 0 }, { 1, 1, 1 }, { 0, 1, 1 },
	};
	for (int cubeCase = 0; cubeCase < 256; ++cubeCase)
	{
		float values[8] = {};
		for (int corner = 0; corner < 8; ++corner)
		{
			const int x = corners[corner][0], y = corners[corner][1], z = corners[corner][2];
			values[x + 2 * (y + 2 * z)] = (cubeCase & (1 << corner)) ? 1.0f : -1.0f;
		}
		input.Resolution = Vector3i(2, 2, 2);
		input.Values = values;
		input.ValueCount = 8;
		input.IsoValue = 0.0f;
		Riemann::FluidSurfaceMesh mesh;
		EXPECT(Riemann::MarchingCubes::Extract(input, &mesh));
		EXPECT(mesh.Indices.size() % 3 == 0);
		for (uint32_t index : mesh.Indices) EXPECT(index < mesh.Positions.size());
		for (size_t triangle = 0; triangle < mesh.Indices.size(); triangle += 3)
		{
			const Vector3& a = mesh.Positions[mesh.Indices[triangle]];
			const Vector3& b = mesh.Positions[mesh.Indices[triangle + 1]];
			const Vector3& c = mesh.Positions[mesh.Indices[triangle + 2]];
			EXPECT((b - a).Cross(c - a).SquareLength() > 1.0e-12f);
		}
	}

	input.Origin = Vector3(-1.4f, -1.4f, -1.4f);
	input.Spacing = Vector3(0.2f, 0.2f, 0.2f);
	input.Resolution = Vector3i(15, 15, 15);
	input.IsoValue = 0.0f;
	std::vector<float> sphereValues((size_t)15 * 15 * 15);
	for (int z = 0; z < 15; ++z)
	for (int y = 0; y < 15; ++y)
	for (int x = 0; x < 15; ++x)
	{
		const Vector3 position = input.Origin + Vector3((float)x, (float)y, (float)z) * input.Spacing;
		sphereValues[(size_t)x + 15 * ((size_t)y + 15 * (size_t)z)] = 1.0f - position.Length();
	}
	input.Values = sphereValues.data();
	input.ValueCount = sphereValues.size();
	Riemann::FluidSurfaceMesh sphere;
	EXPECT(Riemann::MarchingCubes::Extract(input, &sphere));
	EXPECT(!sphere.Indices.empty());
	for (size_t index = 0; index < sphere.Positions.size(); ++index)
	{
		EXPECT(std::fabs(sphere.Positions[index].Length() - 1.0f) < 0.18f);
		EXPECT(sphere.Normals[index].Dot(sphere.Positions[index].SafeUnit()) > 0.9f);
	}
	std::map<std::pair<uint32_t, uint32_t>, int> edgeUse;
	for (size_t triangle = 0; triangle < sphere.Indices.size(); triangle += 3)
	{
		for (int edge = 0; edge < 3; ++edge)
		{
			uint32_t a = sphere.Indices[triangle + (size_t)edge];
			uint32_t b = sphere.Indices[triangle + (size_t)((edge + 1) % 3)];
			if (b < a) std::swap(a, b);
			++edgeUse[std::make_pair(a, b)];
		}
	}
	for (const auto& edge : edgeUse) EXPECT(edge.second == 2);

	Riemann::FluidSurfaceMesh sphereAgain;
	EXPECT(Riemann::MarchingCubes::Extract(input, &sphereAgain));
	EXPECT(sphereAgain.Indices == sphere.Indices);
	EXPECT(sphereAgain.Positions == sphere.Positions);

	for (int z = 0; z < 15; ++z)
	for (int y = 0; y < 15; ++y)
	for (int x = 0; x < 15; ++x)
	{
		const Vector3 position = input.Origin + Vector3((float)x, (float)y, (float)z) * input.Spacing;
		sphereValues[(size_t)x + 15 * ((size_t)y + 15 * (size_t)z)] = 1.0e-5f * position.Length();
	}
	input.IsoValue = 1.0e-5f;
	input.HighValuesInside = false;
	Riemann::FluidSurfaceMesh rarefiedSphere;
	EXPECT(Riemann::MarchingCubes::Extract(input, &rarefiedSphere));
	EXPECT(!rarefiedSphere.Indices.empty());
	for (size_t index = 0; index < rarefiedSphere.Positions.size(); ++index)
	{
		EXPECT(rarefiedSphere.Normals[index].Dot(
			rarefiedSphere.Positions[index].SafeUnit()) > 0.9f);
	}

	sphereValues[0] = std::numeric_limits<float>::quiet_NaN();
	EXPECT_FALSE(Riemann::MarchingCubes::Extract(input, &sphereAgain));
}

void TestCores()
{
	TestFluidSimulation();
	TestFluidRigidCoupling();
	TestAerodynamicsRigidCoupling();
	TestAerodynamicsSimulation();
	TestMarchingCubes();
	TestFileAndBaseUtilities();
	TestDynamicArray();
	TestStaticContainers();
	TestListAndBatchContainers();
	TestPriorityQueue();
	TestBitSet();
	TestBitSetOperations();
	// TestThreadPool();
	// TestJob();
	TestGraph();
	TestListSet();
	TestMapAndSetCapacity();
	TestSmallSet();
	TestSmallMap();
	TestLinearSet();
	TestLinearMap();
	TestRingBuffer();
}
