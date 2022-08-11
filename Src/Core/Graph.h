#pragma once

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

	std::vector<T>						nodes;
	std::vector<std::pair<int, int>>	edges;
};
