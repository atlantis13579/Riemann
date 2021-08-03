#pragma once

#include <vector>
#include "../Maths/Vector3d.h"
#include "GeometryObject.h"

class AABBTree;
class DynamicAABBTree;
class SparseSpatialHash;

class GeometryQuery
{
public:
	GeometryQuery();
	~GeometryQuery();

public:
	void			BuildStaticGeometry(const std::vector<Geometry*>& Objects, int nPrimitivePerNode);
	
	bool			RayCast(const Vector3d &Origin, const Vector3d& Dir, RayCastResult *Result);

	AABBTree*		GetStaticTree()
	{
		return m_staticGeometry;
	}

private:
	std::vector<Geometry*>	m_Objects;

	AABBTree*				m_staticGeometry;
	DynamicAABBTree*		m_dynamicPruner;
	SparseSpatialHash*		m_SpatialHash;
};