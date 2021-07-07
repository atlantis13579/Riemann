#pragma once

#include <vector>
#include "../Maths/Vector3d.h"
#include "GeometryObject.h"

class AABBPruner;
class SparseSpatialHash;

class GeometryQuery
{
public:
	GeometryQuery();
	~GeometryQuery();

public:
	void BuildStaticGeometry(const std::vector<GeometryObject>& Objects, int nPrimitivePerNode);

	bool RayCast(const Vector3d &Origin, const Vector3d& Dir, RayCastResult *Result);

private:
	AABBPruner*			m_staticGeometry;
	AABBPruner*			m_dynamicPruner;
	SparseSpatialHash*	m_SpatialHashPruner;
};