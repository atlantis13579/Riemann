
#include <assert.h>
#include <string>
#include <chrono>
#include <algorithm>
#include <map>
#include <vector>
#include "Test.h"
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

void TestCores()
{
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
