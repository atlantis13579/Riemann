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
	Vector3		hitNormal;
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

	bool			IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
	bool			IntersectRay(const Vector3& Origin, const Vector3& Direction, const TriMeshHitOption& Option, TriMeshHitResult* Result) const;
	bool			RayIntersectTri(uint32_t HitNode, const Vector3& Origin, const Vector3& Direction, const TriMeshHitOption& Option, TriMeshHitResult* Result) const;
	bool			IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;
	bool			IntersectOBB(const Vector3& Center, const Vector3& Extent, const Matrix3& rot) const;
	bool			IntersectSphere(const Vector3& Center, float Radius) const;
	bool			IntersectCapsule(const Vector3& X0, const Vector3& X1, float Radius) const;

	bool CalculateVolumeProperties(MassParameters* p, float Density) const
	{
		p->Volume = 0.0f;
		p->Mass = 0.0f;
		p->CenterOfMass = BoundingVolume.GetCenter();
		p->BoundingVolume = BoundingVolume;
		p->InertiaMat = Matrix3::Identity();
		return true;
	}

	Vector3			GetSupport(const Vector3& Direction) const;
	int				GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const;

private:
	MeshBVH4*	m_BVH;
};
