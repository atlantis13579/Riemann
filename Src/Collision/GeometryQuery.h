#pragma once

class AABBPruner;
class SparseSpatialHash;

class GeometryQuery
{
public:
	GeometryQuery();
	~GeometryQuery();

	void Init();

private:
	AABBPruner*			m_staticPruner;
	AABBPruner*			m_dynamicPruner;
	SparseSpatialHash*	m_SpatialHashPruner;
};