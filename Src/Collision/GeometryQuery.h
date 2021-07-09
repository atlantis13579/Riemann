#pragma once

#include <vector>
#include "../Maths/Vector3d.h"
#include "GeometryObject.h"

class AABBPruner;
class SparseSpatialHash;
class BroadPhase;

class GeometryQuery
{
public:
	GeometryQuery();
	~GeometryQuery();

	void Simulate(float dt);

public:
	void BuildStaticGeometry(const std::vector<Geometry*>& Objects, int nPrimitivePerNode);

	bool RayCast(const Vector3d &Origin, const Vector3d& Dir, RayCastResult *Result);

	void CreateBroadPhaseFilter(const char* name);

private:
	AABBPruner*			m_staticGeometry;
	AABBPruner*			m_dynamicPruner;
	SparseSpatialHash*	m_SpatialHashPruner;

	BroadPhase*			m_BPhase;
};