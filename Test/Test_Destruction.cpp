
#include "Test.h"
#include <vector>
#include "../Src/Destruction/GraphPartition.h"

using namespace Riemann;

void TestMetis()
{
	int nVertices = 8;
	std::vector<int> nodes;
	for (int i = 0; i < nVertices; ++i)
	{
		nodes.push_back(i);
	}
	std::vector<std::vector<int>> part;
	std::vector<int> xadj = { 0,2,4,7,10,13,16,18,20 };
	std::vector<int> adjncy = { 1,2,0,3,0,3,4,1,2,5,2,5,6,3,4,7,4,7,5,6 };
	PartitionCsrGraph(nodes, xadj, adjncy, nullptr, 2, part);

	EXPECT(part.size() == 2);
	EXPECT(part[0].size() == 4);
	EXPECT(part[1].size() == 4);
	return ;
}

void TestDestruction()
{
	TestMetis();
	return;
}
