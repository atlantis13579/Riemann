#pragma once

#include <map>
#include <vector>

namespace Riemann
{
	bool PartitionCsrGraph(const std::vector<int>& nodes, std::vector<int>& xadj, std::vector<int>& adjncy, std::vector<int>* adjwgt, int num_part, std::vector<std::vector<int>>& parts);
	
	class ConnectionGraph;
	bool PartitionConnectionGraph(const std::vector<int>& nodes, const ConnectionGraph* p, int num_part, std::map<int, int>& node_to_index, std::vector<std::vector<int>>& parts);
}	// namespace Riemann