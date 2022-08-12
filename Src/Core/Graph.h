#pragma once

#include <assert.h>
#include <queue>
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

	std::vector<T>						nodes;
	std::vector<std::pair<int, int>>	edges;
};

