#pragma once

#include <stdint.h>

#include "../Core/StaticArray.h"
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

struct CollisionData
{
	CollisionData()
	{
		v0 = 0;
	}
	unsigned int v0;
};

typedef	StaticArray<Vector3, MAX_FACE_POINTS> SupportFace;

class Geometry
{
	friend class GeometryFactory;
	friend class GeometryRegistration;
public:
	virtual ~Geometry() {}

	void					SetPosition(const Vector3& Position);
	Vector3				GetPosition() const;
	Matrix3				GetRotationMatrix() const;
	Quaternion				GetRotationQuat() const;
	void					SetRotationQuat(const Quaternion& Rotation);
	const Matrix4&			GetWorldMatrix();
	const Matrix4&			GetInverseWorldMatrix();

	void*					GetEntity();
	void					SetEntity(void* Entity);

	const Transform*		GetTransform() const
	{
		return &m_Transform;
	}
	
	Transform*				GetTransform()
	{
		return &m_Transform;
	}

	ShapeType3d				GetShapeType() const
	{
		return m_Type;
	}
	
	const CollisionData&	GetFilterData() const
	{
		return m_FilterData;
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

	virtual bool			RayCast(const Vector3& Origin, const Vector3 &Dir, const RayCastOption* Option, RayCastResult *Result) const = 0;
	bool					Overlap(const Geometry* Geom) const;
	bool					Sweep(const Geometry* Geom, const Vector3& Dir, float* t) const;

	void					UpdateBoundingVolume();
	const Box3d&			GetBoundingVolume_WorldSpace() const;
	Vector3				GetSupport_WorldSpace(const Vector3& Dir) const;
	void					GetSupportFace_WorldSpace(const Vector3& Dir, SupportFace& Face) const;
	Matrix3				GetInverseInertia_LocalSpace(float InvMass) const;

private:
	virtual Matrix3		GetInertia_LocalSpace(float InvMass) const = 0;
	virtual Vector3		GetSupport_LocalSpace(const Vector3& Dir) const = 0;
	virtual void			GetSupportFace_LocalSpace(const Vector3& Dir, SupportFace& Face) const = 0;
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
	ShapeType3d				m_Type;
	Box3d					m_BoxWorld;
	Transform				m_Transform;
	uint64_t				m_Guid;
	CollisionData			m_FilterData;
	void*					m_Entity;
};

class GeometryFactory
{
public:
	static void		 DeleteGeometry(Geometry *Geom);

	static Geometry* CreateOBB(const Vector3& Center, const Vector3 & HalfExtent, const Quaternion& Rot = Quaternion::One());
	static Geometry* CreatePlane(const Vector3& Center, const Vector3& Normal, float HalfThickness = 0.01f);
	static Geometry* CreateSphere(const Vector3& Center, float Radius);
	static Geometry* CreateCapsule(const Vector3& X1, const Vector3& X2, float Radius);
	static Geometry* CreateHeightField(const Box3d& Bv, int nRows, int nCols);
	static Geometry* CreateConvexMesh();
	static Geometry* CreateTriangle(const Vector3& A, const Vector3& B, const Vector3& C);
	static Geometry* CreateTriangleMesh();

	static int		ObjectCount[(int)ShapeType3d::TYPE_COUNT];
};
