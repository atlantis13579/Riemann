#pragma once

#include <assert.h>
#include <queue>
#include <set>
#include <vector>

template <typename T>
struct Graph
{
	void AddNode(const T& v)
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
	
	void KahnTopologicalSort(std::vector<int> &sorted) const
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
			sorted.push_back(c);
			
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
		
		for (int i = 0; i < n; ++i)
		{
			if (degree[i] == 0)
				continue;
			
			sorted.push_back(i);
		}
	}

	void BuildAdjacencyMatrix(std::vector<bool>& adjs) const
	{
		size_t n = nodes.size();
		adjs.resize(n*n, false);
		for (size_t i = 0; i < edges.size(); ++i)
		{
			auto e = edges[i];
			adjs[e.first * n + e.second] = true;
		}
	}
	
	void BuildDegreeMatrix(std::vector<int>& degrees) const
	{
		size_t n = nodes.size();
		degrees.resize(n, 0);
		for (size_t i = 0; i < edges.size(); ++i)
		{
			auto e = edges[i];
			degrees[e.second] += 1;
		}
	}
	
	void BuildSparseAdjacencyMatrix(std::vector<int>& offset, std::vector<int>& adjs)
	{
		std::vector<std::pair<int, int>>	&edges1 = edges;
		std::vector<std::pair<int, int>>	edges2 = edges;
		
		// Make Edge Ascending
		for (size_t i = 0; i < edges1.size(); ++i)
		{
			std::pair<int, int> e = edges1[i];
			if (e.first > e.second)
			{
				edges1[i] = std::pair<int, int>(e.second, e.first);
			}
		}
		
		std::sort(edges1.begin(), edges1.end());
		
		// Make Edge Descending
		for (size_t i = 0; i < edges2.size(); ++i)
		{
			std::pair<int, int> e = edges2[i];
			if (e.first < e.second)
			{
				edges2[i] = std::pair<int, int>(e.second, e.first);
			}
		}
		std::sort(edges2.begin(), edges2.end());
		
		offset.resize(nodes.size());
		adjs.clear();
		
		for (size_t i = 0; i < offset.size(); ++i)
		{
			offset[i] = -1;
		}
		
		// Merge edges1 and edges2
		size_t pivot1 = 0, pivot2 = 0;
		int n = (int)nodes.size();
		for (int i = 0; i < n; ++i)
		{
			size_t size_old = adjs.size();
			
			while (pivot2 < edges2.size() && edges2[pivot2].first == i)
			{
				adjs.push_back(edges2[pivot2].second);
				pivot2++;
			}
			
			while (pivot1 < edges1.size() && edges1[pivot1].first == i)
			{
				adjs.push_back(edges1[pivot1].second);
				pivot1++;
			}
			
			if (adjs.size() > size_old)
			{
				offset[i] = (int)size_old;
			}
		}
	}
	
	static bool GetAdjacencyList(const std::vector<int>& offset, const std::vector<int>& adjs, int i, int *offset0, int *offset1)
	{
		if (offset[i] == -1)
		{
			return false;
		}
		
		int j0 = offset[i], j1 = (int)adjs.size() - 1;
		for (size_t j = i + 1; j < offset.size(); ++j)
		{
			if (offset[j] != -1)
			{
				j1 = offset[j] - 1;
				break;
			}
		}
		
		*offset0 = j0;
		*offset1 = j1;
		return true;
	}
	
	void BuildIslands(const std::vector<bool>& separate, std::vector<std::vector<T>> *islands)
	{
		islands->clear();
		
		std::vector<bool> in_island;
		std::vector<int> offset;
		std::vector<int> adjs;
		BuildSparseAdjacencyMatrix(offset, adjs);
		
		int n = (int)nodes.size();
		
		in_island.resize(n);
		for (int i = 0; i < n; ++i)
		{
			in_island[i] = false;
		}

		for (int i = 0; i < n; ++i)
		{
			if (offset[i] == -1 || separate[i] || in_island[i])
			{
				continue;
			}
			
			std::vector<T> island;
			
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
				
				if (offset[curr] == -1 || separate[curr])
				{
					continue;
				}
				
				int j0, j1;
				GetAdjacencyList(offset, adjs, curr, &j0, &j1);
				assert(j0 <= j1);
				
				for (int j = j0; j <= j1; ++j)
				{
					if (in_queue.count(adjs[j]) != 0)
						continue;
					qu.push(adjs[j]);
					in_queue.insert(adjs[j]);
				}
			}
			
			islands->push_back(std::move(island));
		}
		
	}

	std::vector<T>						nodes;
	std::vector<std::pair<int, int>>	edges;
};

