
#include "AABBPruner.h"
#include "AABBTree.h"

AABBPruner::AABBPruner()
{
	m_staticAABB = nullptr;
}

AABBPruner::~AABBPruner()
{
	if (m_staticAABB)
	{
		delete m_staticAABB;
		m_staticAABB = nullptr;
	}
}