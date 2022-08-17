#pragma once

#include <assert.h>
#include <queue>
#include <set>
#include <vector>

template <typename N>
struct Graph
{
	struct Edge
	{
		Edge(int _n0, int _n1) : n0(_n0), n1(_n1) {}
		int n0;
		int n1;
	};
	std::vector<N>		nodes;
	std::vector<Edge>	edges;

	void AddNode(const N& v)
	{
		nodes.push_back(v);
	}

	void AddEdge(int s, int e)
	{
		edges.emplace_back(s, e);
	}

	// Kahn’s Algorithm for Topological Sort
	bool HasCycle() const
	{
		std::vector<bool> adjs;
		std::vector<int> degree;
		BuildAdjacencyMatrix(adjs);
		BuildDegreeMatrix(degree);
		
		int n = (int)nodes.size();
		std::queue<int> qu;
		for (int i = 0; i < n; ++i)
		{
			if (degree[i] == 0)
			{
				qu.push(i);
			}
		}
		
		while (!qu.empty())
		{
			int c = qu.front();
			qu.pop();
			
			for (int i = 0; i < n; ++i)
			{
				if (adjs[c*n+i] == false)
					continue;

				assert(degree[i] > 0);
				degree[i] -= 1;
				if (degree[i] == 0)
				{
					qu.push(i);
				}
			}
		}
		
		bool has_cycle = false;
		for (int i = 0; i < n; ++i)
		{
			if (degree[i] != 0)
			{
				has_cycle = true;
				break;
			}
		}
		return has_cycle;
	}

	void BuildAdjacencyMatrix(std::vector<bool>& adjs) const
	{
		size_t n = nodes.size();
		adjs.resize(n*n, false);
		for (size_t i = 0; i < edges.size(); ++i)
		{
			auto e = edges[i];
			adjs[e.n0 * n + e.n1] = true;
		}
	}
	
	void BuildDegreeMatrix(std::vector<int>& degrees) const
	{
		size_t n = nodes.size();
		degrees.resize(n, 0);
		for (size_t i = 0; i < edges.size(); ++i)
		{
			auto e = edges[i];
			degrees[e.n1] += 1;
		}
	}
	
	void BuildSparseAdjacencyMatrix(std::vector<int>& entry, std::vector<int>& adjs)
	{
		std::vector<Edge>	&edges1 = edges;
		std::vector<Edge>	edges2 = edges;
		
		// Make Edge Ascending
		for (size_t i = 0; i < edges1.size(); ++i)
		{
			Edge &a = edges1[i];
			if (a.n0 > a.n1)
			{
				std::swap(a.n0, a.n1);
			}
		}
		
		std::sort(edges1.begin(), edges1.end());
		
		// Make Edge Descending
		for (size_t i = 0; i < edges2.size(); ++i)
		{
			Edge& a = edges1[i];
			if (a.n0 < a.n1)
			{
				std::swap(a.n0, a.n1);
			}
		}
		std::sort(edges2.begin(), edges2.end());
		
		entry.resize(nodes.size());
		adjs.clear();
		
		for (size_t i = 0; i < entry.size(); ++i)
		{
			entry[i] = -1;
		}
		
		// Merge edges1 and edges2
		size_t pivot1 = 0, pivot2 = 0;
		int n = (int)nodes.size();
		for (int i = 0; i < n; ++i)
		{
			size_t size_old = adjs.size();
			
			while (pivot2 < edges2.size() && edges2[pivot2].n0 == i)
			{
				adjs.push_back(edges2[pivot2].n1);
				pivot2++;
			}
			
			while (pivot1 < edges1.size() && edges1[pivot1].n0 == i)
			{
				adjs.push_back(edges1[pivot1].n1);
				pivot1++;
			}
			
			if (adjs.size() > size_old)
			{
				entry[i] = (int)size_old;
			}
		}
	}
	
	static bool GetAdjacencyList(const std::vector<int>& entry, int adjs, int n0, int *entry0, int *entry1)
	{
		if (entry[n0] == -1)
		{
			return false;
		}
		
		int j0 = entry[n0], j1 = adjs - 1;
		for (size_t j = n0 + 1; j < entry.size(); ++j)
		{
			if (entry[j] != -1)
			{
				j1 = entry[j] - 1;
				break;
			}
		}
		
		*entry0 = j0;
		*entry1 = j1;
		return true;
	}
};

template <typename N, typename E>
struct GraphNE
{
	struct Edge
	{
		Edge(int _n0, int _n1, const E& _obj) : n0(_n0), n1(_n1), obj(_obj) {}
		int n0;
		int n1;
		E   obj;
	};
	struct EdgeAdj
	{
		EdgeAdj(int _n1, const E& _obj) : n1(_n1), obj(_obj) {}
		int n1;
		E   obj;
	};
	std::vector<N>		nodes;
	std::vector<Edge>	edges;

	void AddNode(const N& n)
	{
		nodes.push_back(n);
	}

	void AddEdge(int n1, int n2, const E& e)
	{
		edges.emplace_back(n1, n2, e);
	}

	void BuildSparseAdjacencyMatrix(std::vector<int>& entry, std::vector<EdgeAdj>& adjs)
	{
		std::vector<Edge>& edges1 = edges;
		std::vector<Edge>  edges2 = edges;

		// Make Edge Ascending
		for (size_t i = 0; i < edges1.size(); ++i)
		{
			Edge& a = edges1[i];
			if (a.n0 > a.n1)
			{
				std::swap(a.n0, a.n1);
			}
		}

		auto sort = [](const Edge& a, const Edge& b) {
			if (a.n0 == b.n0)
				return a.n1 < b.n1;
			return a.n0 < b.n0;
		};

		std::sort(edges1.begin(), edges1.end(), sort);

		// Make Edge Descending
		for (size_t i = 0; i < edges2.size(); ++i)
		{
			Edge& a = edges2[i];
			if (a.n0 < a.n1)
			{
				std::swap(a.n0, a.n1);
			}
		}
		std::sort(edges2.begin(), edges2.end(), sort);

		entry.resize(nodes.size());
		adjs.clear();

		for (size_t i = 0; i < entry.size(); ++i)
		{
			entry[i] = -1;
		}

		// Merge edges1 and edges2
		size_t pivot1 = 0, pivot2 = 0;
		int n = (int)nodes.size();
		for (int i = 0; i < n; ++i)
		{
			size_t size_old = adjs.size();

			while (pivot2 < edges2.size() && edges2[pivot2].n0 == i)
			{
				adjs.emplace_back(edges2[pivot2].n1, edges2[pivot2].obj);
				pivot2++;
			}

			while (pivot1 < edges1.size() && edges1[pivot1].n0 == i)
			{
				adjs.emplace_back(edges1[pivot1].n1, edges1[pivot1].obj);
				pivot1++;
			}

			if (adjs.size() > size_old)
			{
				entry[i] = (int)size_old;
			}
		}
	}

	void BuildNodeIslands(const std::vector<bool>& separate, std::vector<std::vector<N>>* islands)
	{
		islands->clear();

		std::vector<bool> in_island;
		std::vector<int> entry;
		std::vector<EdgeAdj> adjs;
		BuildSparseAdjacencyMatrix(entry, adjs);

		int n = (int)nodes.size();

		in_island.resize(n);
		for (int i = 0; i < n; ++i)
		{
			in_island[i] = false;
		}

		for (int i = 0; i < n; ++i)
		{
			if (entry[i] == -1 || separate[i] || in_island[i])
			{
				continue;
			}

			std::vector<N> island;

			// BFS
			std::queue<int> qu;
			std::set<int> in_queue;
			qu.push(i);
			in_queue.insert(i);
			while (!qu.empty())
			{
				int curr = qu.front();
				qu.pop();

				island.push_back(nodes[curr]);
				in_island[curr] = true;

				if (entry[curr] == -1 || separate[curr])
				{
					continue;
				}

				int j0, j1;
				Graph<N>::GetAdjacencyList(entry, (int)adjs.size(), curr, &j0, &j1);
				assert(j0 <= j1);

				for (int j = j0; j <= j1; ++j)
				{
					if (in_queue.count(adjs[j].n1) != 0)
						continue;
					qu.push(adjs[j].n1);
					in_queue.insert(adjs[j].n1);
				}
			}

			islands->push_back(std::move(island));
		}
	}

	void BuildEdgeIslands(const std::vector<bool>& separate, std::vector<std::vector<E>>* islands)
	{
		islands->clear();

		std::vector<bool> in_island;
		std::vector<int> entry;
		std::vector<EdgeAdj> adjs;
		BuildSparseAdjacencyMatrix(entry, adjs);
		
		int n = (int)nodes.size();

		in_island.resize(n);
		for (int i = 0; i < n; ++i)
		{
			in_island[i] = false;
		}

		for (int i = 0; i < n; ++i)
		{
			if (entry[i] == -1 || separate[i] || in_island[i])
			{
				continue;
			}

			std::vector<E> island;

			// BFS
			std::queue<int> qu;
			std::set<E> in_queue;
			qu.push(i);
			while (!qu.empty())
			{
				int curr = qu.front();
				qu.pop();

				in_island[curr] = true;

				if (entry[curr] == -1 || separate[curr])
				{
					continue;
				}

				int j0, j1;
				Graph<N>::GetAdjacencyList(entry, (int)adjs.size(), curr, &j0, &j1);
				assert(j0 <= j1);

				for (int j = j0; j <= j1; ++j)
				{
					if (in_queue.count(adjs[j].obj) != 0)
						continue;
					qu.push(adjs[j].n1);
					in_queue.insert(adjs[j].obj);
					island.push_back(adjs[j].obj);
				}
			}

			islands->push_back(std::move(island));
		}
	}

};

