#pragma once

#include "Mesh.h"
#include "MeshBVH4.h"
#include "../Maths/Matrix3d.h"

struct TriMeshHitOption 
{
	bool	hitClosest;
	float	maxDist;
};

struct TriMeshHitResult
{
	float		hitTime;
	Vector3d	hitNormal;
	uint32_t	hitIndex;
};

class TriangleMesh : public Mesh
{
public:
	TriangleMesh()
	{
		m_BVH = nullptr;
		m_Memory = nullptr;
	}

	~TriangleMesh()
	{
		Release();
	}

	void			Release()
	{
		Mesh::Release();
		if (m_BVH)
		{
			delete m_BVH;
			m_BVH = nullptr;
		}
	}

	MeshBVH4*		CreateEmptyBVH();
	void*			AllocMemory(int Size, int Width);
	void			BuildBVH();

	void			GetVertIndices(uint32_t triIndex, uint32_t& i0, uint32_t& i1, uint32_t& i2) const;

	bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const;
	bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, const TriMeshHitOption& Option, TriMeshHitResult* Result) const;
	bool			IntersectTri(uint32_t HitNode, const Vector3d& Origin, const Vector3d& Dir, const TriMeshHitOption& Option, TriMeshHitResult* Result) const;

	const Box3d&	GetBoundingVolume() const
	{
		return BoundingVolume;
	}

	Matrix3d		GetInertiaTensor(float Mass) const;

	Vector3d		GetSupport(const Vector3d& dir) const;

private:
	MeshBVH4*	m_BVH;
	void*		m_Memory;
};