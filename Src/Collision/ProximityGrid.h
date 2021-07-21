#pragma once

#include <vector>

struct Cell
{

};

class ProximityGrid
{
public:
	ProximityGrid() {}
	~ProximityGrid() {}

public:
	int m_nX, m_nY;
	std::vector<Cell> m_Grids;
};
