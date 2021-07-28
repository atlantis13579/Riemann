#pragma once

#include "../CollisionPrimitive/Mesh.h"
#include "../Maths/Matrix3d.h"
#include "RTree.h"

class TriangleMesh : public Mesh
{
public:
	TriangleMesh()
	{
		m_Tree = nullptr;
		m_Memory = nullptr;
	}

	~TriangleMesh()
	{
		Release();
	}

	void			Release()
	{
		Mesh::Release();
		if (m_Tree)
		{
			delete m_Tree;
			m_Tree = nullptr;
		}
	}

	RTree*			CreateEmptyRTree();
	void*			AllocMemory(int Size, int Width);
	void			BuildRTree();

	bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const;

	const Box3d&	GetBoundingVolume() const
	{
		return BoundingBox;
	}

	Matrix3d GetInertiaTensor(float Mass) const;

	Vector3d GetSupport(const Vector3d& dir) const;

private:
	RTree*	m_Tree;
	void*	m_Memory;
};