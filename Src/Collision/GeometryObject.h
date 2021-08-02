#pragma once

#include <string>
#include "../Maths/Box3d.h"
#include "../Maths/Transform.h"
#include "../CollisionPrimitive/GeometryType.h"

typedef bool			(*RayCastFunc)		(void*, const Vector3d&, const Vector3d&, float*);
typedef bool			(*OverlapFunc)		(const void*, const void*);
typedef bool			(*SweepFunc)		(const void*, const void*, const Vector3d&, float*);

class GeometryFactory;
class Geometry_Registration;

class Geometry
{
	friend class GeometryFactory;
	friend class Geometry_Registration;
public:
	virtual ~Geometry() {}

	void					SetPosition(const Vector3d& Position);
	Vector3d				GetPosition() const;
	Matrix3d				GetRotationMatrix() const;
	Quaternion				GetRotationQuat() const;
	void					SetRotationQuat(const Quaternion& Rotation);
	const Matrix4d&			GetWorldMatrix();
	const Matrix4d&			GetInverseWorldMatrix();

	void*					GetEntity();
	void					SetEntity(void* Entity);

	Transform*				GetTransform()
	{
		return &m_Transform;
	}

	GeometryType			GetGeometryType() const
	{
		return m_Type;
	}

	template<class GEOM_TYPE>
	GEOM_TYPE*				GetGeometry()
	{
		if (m_Type == GEOM_TYPE::StaticType())
		{
			return static_cast<GEOM_TYPE*>(GetGeometry());
		}
		return nullptr;
	}

	template<class GEOM_TYPE>
	const GEOM_TYPE*		GetGeometry() const
	{
		if (m_Type == GEOM_TYPE::StaticType())
		{
			return static_cast<GEOM_TYPE*>(GetGeometry());
		}
		return nullptr;
	}

	const char*				GetName() const
	{
		return m_Name.c_str();
	}

	void					SetName(const char* name)
	{
		if (name)
		{
			m_Name = std::string(name);
		}
	}

	bool					RayCast(const Vector3d& Origin, const Vector3d &Dir, float* t);
	bool					Overlap(const Geometry* Geom) const;
	bool					Sweep(const Geometry* Geom, const Vector3d& Dir, float* t) const;

	const Box3d&			GetBoundingVolume_WorldSpace() const;
	Vector3d				GetSupport_WorldSpace(const Vector3d& Dir);
	Matrix3d				GetInverseInertia_WorldSpace(float Mass) const;

private:
	virtual Matrix3d		GetInertia_LocalSpace(float Mass) const = 0;
	virtual Vector3d		GetSupport_LocalSpace(const Vector3d& Dir) const = 0;
	virtual Box3d			GetBoundingVolume_LocalSpace() const = 0;

	virtual const void*		GetGeometry() const = 0;
	virtual void*			GetGeometry() = 0;

protected:
	GeometryType			m_Type;
	Box3d					m_BoxWorld;
	Transform				m_Transform;
	std::string				m_Name;
	void*					m_Entity;

	static RayCastFunc		raycastTable[GeometryType::GEOMETRY_COUNT];
	static SweepFunc		sweepTable[GeometryType::GEOMETRY_COUNT][GeometryType::GEOMETRY_COUNT];
	static OverlapFunc		overlapTable[GeometryType::GEOMETRY_COUNT][GeometryType::GEOMETRY_COUNT];
};

class GeometryFactory
{
public:
	static Geometry* CreateOBB(const Vector3d& Center, const Vector3d & Extent, const Quaternion& Rot = Quaternion::One());
	static Geometry* CreatePlane(const Vector3d& Center, const Vector3d& Normal);
	static Geometry* CreateSphere(const Vector3d& Center, float Radius);
	static Geometry* CreateCapsule(const Vector3d& X1, const Vector3d& X2, float Radius);
	static Geometry* CreateHeightField(const Box3d& Bv, int nRows, int nCols);
	static Geometry* CreateConvexMesh();
	static Geometry* CreateTriangle(const Vector3d& A, const Vector3d& B, const Vector3d& C);
	static Geometry* CreateTriangleMesh();
};

struct RayCastResult
{
	RayCastResult()
	{
		hit = false;
		hitTime = FLT_MAX;
		hitGeom = nullptr;
	}

	bool		hit;
	float		hitTime;
	Vector3d	hitPoint;
	Vector3d	hitNormal;
	Geometry*	hitGeom;
};


struct SweepResult
{
	SweepResult()
	{
	}
};


struct OverlapResult
{
	OverlapResult()
	{
	}
};