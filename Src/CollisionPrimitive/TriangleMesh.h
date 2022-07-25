#pragma once

#include <assert.h>
#include "Mesh.h"
#include "MeshBVH4.h"
#include "../Maths/Matrix3.h"

struct TriMeshHitOption 
{
	bool	hitNearest;
	float	maxDist;
};

struct TriMeshHitResult
{
	float		hitTime;
	Vector3	hitNormal;
	uint32_t	hitIndex;
	int		    hitTestCount;
    
	void AddTestCount(int Count)
	{
		#ifdef _DEBUG
		hitTestCount += Count;
		#endif // _DEBUG
	}
};

class TriangleMesh : public Mesh
{
public:
	TriangleMesh()
	{
		m_BVH = nullptr;
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
	void			BuildBVH();

	void			GetVertIndices(uint32_t triIndex, uint32_t& i0, uint32_t& i1, uint32_t& i2) const;

	bool			IntersectRay(const Vector3& Origin, const Vector3& Dir, float* t) const;
	bool			IntersectRay(const Vector3& Origin, const Vector3& Dir, const TriMeshHitOption& Option, TriMeshHitResult* Result) const;
	bool			RayIntersectTri(uint32_t HitNode, const Vector3& Origin, const Vector3& Dir, const TriMeshHitOption& Option, TriMeshHitResult* Result) const;
	bool			OverlapTri(uint32_t HitNode, const Vector3& Bmin, const Vector3& Bmax) const;
	bool			IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;

	const Box3d&	GetBoundingVolume() const
	{
		return BoundingVolume;
	}

	Matrix3		GetInertiaTensor(float Mass) const;
	Vector3		GetSupport(const Vector3& dir) const;
	int				GetSupportFace(const Vector3& dir, Vector3* FacePoints) const;

private:
	MeshBVH4*	m_BVH;
};
