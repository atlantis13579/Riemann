#pragma once

#include <stdint.h>

#include "../Maths/Box3d.h"
#include "../Maths/Transform.h"
#include "../CollisionPrimitive/ShapeType.h"

class GeometryFactory;
class GeometryRegistration;
struct RayCastOption;
struct RayCastResult;
struct OverlapOption;
struct OverlapResult;
struct SweepOption;
struct SweepResult;

class Geometry
{
	friend class GeometryFactory;
	friend class GeometryRegistration;
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

	const Transform&		GetTransform() const
	{
		return m_Transform;
	}
	
	Transform*				GetTransform()
	{
		return &m_Transform;
	}

	ShapeType				GetShapeType() const
	{
		return m_Type;
	}

	template<class GEOM_TYPE>
	GEOM_TYPE*				GetShapeObj()
	{
		if (m_Type == GEOM_TYPE::StaticType())
		{
			return static_cast<GEOM_TYPE*>(GetShapeObjPtr());
		}
		return nullptr;
	}

	template<class GEOM_TYPE>
	const GEOM_TYPE*		GetShapeObj() const
	{
		if (m_Type == GEOM_TYPE::StaticType())
		{
			return static_cast<const GEOM_TYPE*>(GetShapeObjPtr());
		}
		return nullptr;
	}

	uint64_t				GetGuid() const
	{
		return m_Guid;
	}

	void					SetGuid(uint64_t guid)
	{
		m_Guid = guid;
	}

	virtual bool			RayCast(const Vector3d& Origin, const Vector3d &Dir, const RayCastOption* Option, RayCastResult *Result) const = 0;
	bool					Overlap(const Geometry* Geom) const;
	bool					Sweep(const Geometry* Geom, const Vector3d& Dir, float* t) const;

	void					UpdateBoundingVolume();
	const Box3d&			GetBoundingVolume_WorldSpace() const;
	Vector3d				GetSupport_WorldSpace(const Vector3d& Dir);
	Matrix3d				GetInverseInertia_WorldSpace(float Mass) const;

private:
	virtual Matrix3d		GetInertia_LocalSpace(float Mass) const = 0;
	virtual Vector3d		GetSupport_LocalSpace(const Vector3d& Dir) const = 0;
	virtual Box3d			GetBoundingVolume_LocalSpace() const = 0;

	const void*				GetShapeObjPtr() const
	{
		const unsigned char* p = reinterpret_cast<const unsigned char*>(this);
		return static_cast<const void*>(p + sizeof(Geometry));
	}

	void*					GetShapeObjPtr()
	{
		unsigned char* p = reinterpret_cast<unsigned char*>(this);
		return static_cast<void*>(p + sizeof(Geometry));
	}

protected:
	ShapeType				m_Type;
	Box3d					m_BoxWorld;
	Transform				m_Transform;
	uint64_t				m_Guid;
	void*					m_Entity;
};

class GeometryFactory
{
public:
	static void		 DeleteGeometry(Geometry *Geom);

	static Geometry* CreateOBB(const Vector3d& Center, const Vector3d & Extent, const Quaternion& Rot = Quaternion::One());
	static Geometry* CreatePlane(const Vector3d& Center, const Vector3d& Normal);
	static Geometry* CreateSphere(const Vector3d& Center, float Radius);
	static Geometry* CreateCapsule(const Vector3d& X1, const Vector3d& X2, float Radius);
	static Geometry* CreateHeightField(const Box3d& Bv, int nRows, int nCols);
	static Geometry* CreateConvexMesh();
	static Geometry* CreateTriangle(const Vector3d& A, const Vector3d& B, const Vector3d& C);
	static Geometry* CreateTriangleMesh();

	static int		ObjectCount[GEOMETRY_COUNT];
};
