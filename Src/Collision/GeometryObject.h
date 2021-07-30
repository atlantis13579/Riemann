#pragma once

#include <string>
#include "../Maths/Box3d.h"
#include "../Maths/Transform.h"

enum GeometryShapeType
{
	UNKNOWN = 0,
	OBB,
	PLANE,
	SPHERE,
	CAPSULE,
	TRIANGLE,
	CYLINDER,
	CONVEX_MESH,
	TRIANGLE_MESH,
	COUNT,
};

typedef Box3d			(*GetAABBFunc)		(void*);
typedef bool			(*RayCastFunc)		(void*, const Vector3d&, const Vector3d&, float*);
typedef Vector3d		(*SupportFunc)		(void*, const Vector3d&);
typedef Matrix3d		(*InertiaFunc)		(void*, float);
typedef void			(*DestoryFunc)		(void*);
typedef bool			(*OverlapFunc)		(void*, void*);
typedef bool			(*SweepFunc)		(void*, void*, const Vector3d&, float*);

struct GeometryShape
{
	GeometryShape()
	{
		Type = GeometryShapeType::UNKNOWN;
		Object = nullptr;
	}

	GeometryShapeType	Type;
	void				*Object;
};

class GeometryFactory;

class Geometry
{
	friend class GeometryFactory;

public:
	Geometry(const Vector3d& Position, GeometryShapeType _Type, void* _ShapeObj, void* _Entity = nullptr);
	~Geometry();

	Box3d					GetBoundingVolumeLocalSpace() const;
	const Box3d&			GetBoundingVolumeWorldSpace() const;
	Vector3d				GetSupportWorldSpace(const Vector3d& Dir);

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

	GeometryShapeType		GetShapeType()
	{
		return m_Shape.Type;
	}

	template<class T>
	T*						CastGeometry()
	{
		return (T*)m_Shape.Object;
	}

	void*					GetShapeGeometry()
	{
		return m_Shape.Object;
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
	bool					Overlap(const Geometry *Geom);
	bool					Sweep(const Geometry* Geom, const Vector3d& Dir, float* t);
	Matrix3d				GetInertia(float Mass) const;
	Matrix3d				GetInverseInertia(float Mass) const;

private:
	Vector3d				GetSupportLocalSpace(const Vector3d& Dir) const;

private:
	GeometryShape			m_Shape;
	Box3d					m_BoxWorld;
	Transform				m_Transform;
	std::string				m_Name;
	void*					m_Entity;

public:
	template <class T>
	static Box3d			GetBoundingVolume(void* Obj)
	{
		T* p = reinterpret_cast<T*>(Obj);
		return p->GetBoundingVolume();
	}

	template <class T>
	static bool				RayCast(void* Obj, const Vector3d& Origin, const Vector3d& Dir, float* t)
	{
		T* p = reinterpret_cast<T*>(Obj);
		return p->IntersectRay(Origin, Dir, t);
	}

	template <class T>
	static Vector3d			GetSupport(void* Obj, const Vector3d& Dir)
	{
		T* p = reinterpret_cast<T*>(Obj);
		return p->GetSupport(Dir);
	}
	
	template <class T>
	static Matrix3d			GetInertia(void* Obj, float Mass)
	{
		T* p = reinterpret_cast<T*>(Obj);
		return p->GetInertiaTensor(Mass);
	}			

	template <class T>
	static void				Destory(void* Obj)
	{
		T* p = reinterpret_cast<T*>(Obj);
		delete p;
	}

	static GetAABBFunc		getaabbTable[GeometryShapeType::COUNT];
	static RayCastFunc		raycastTable[GeometryShapeType::COUNT];
	static SupportFunc		supportTable[GeometryShapeType::COUNT];
	static InertiaFunc		inertiaTable[GeometryShapeType::COUNT];
	static DestoryFunc		destoryTable[GeometryShapeType::COUNT];
	static SweepFunc		sweepTable[GeometryShapeType::COUNT][GeometryShapeType::COUNT];
	static OverlapFunc		overlapTable[GeometryShapeType::COUNT][GeometryShapeType::COUNT];
};


class GeometryFactory
{
public:
	static Geometry* CreateOBB(const Vector3d& Center, const Vector3d & Extent, const Quaternion& Rot = Quaternion::One());
	static Geometry* CreatePlane(const Vector3d& Center, const Vector3d& Normal);
	static Geometry* CreateSphere(const Vector3d& Center, float Radius);
	static Geometry* CreateCapsule(const Vector3d& X1, const Vector3d& X2, float Radius);
	static Geometry* CreateTriangle(const Vector3d& A, const Vector3d& B, const Vector3d& C);
	static Geometry* CreateTriangleMesh();
	static Geometry* CreateConvexMesh();
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