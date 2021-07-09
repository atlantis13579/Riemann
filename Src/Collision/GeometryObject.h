#pragma once

#include "../Maths/Box3d.h"
#include "../Maths/Transform.h"

enum GeometryShapeType
{
	UNKNOWN = 0,
	OBB,
	PLANE,
	SPHERE,
	CAPSULE,
	CONVEX,
	CYLINDER,
	TRIANGLE,
	TRIANGLEMESH,
	COUNT,
};

#define	DECL_GEOMETRY_OBJ(_name)													\
	static Box3d	GetBoundingBox_##_name(void *);									\
	static bool		RayCast_##_name(void*, const Vector3d&, const Vector3d&, float*);	\
	static Vector3d	GetSupport_##_name(void*, const Vector3d&);						\
	static Matrix3d	GetInertia_##_name(void*, float);								\
	static void		Destory_##_name(void*);											\

typedef Box3d			(*GetAABBFunc)		(void*);
typedef bool			(*RayCastFunc)		(void*, const Vector3d&, const Vector3d&, float*);
typedef Vector3d		(*SupportFunc)		(void*, const Vector3d&);
typedef Matrix3d		(*InertiaFunc)		(void*, float);
typedef void			(*DestoryFunc)		(void*);

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

	const Box3d&			GetBoundingBoxWorld() const;
	Vector3d				GetSupportWorld(const Vector3d& Dir);

	void					SetPositionWorld(const Vector3d& Position);
	void					SetPositionOffset(const Vector3d& Offset);
	Vector3d				GetPositionWorld() const;
	Quaternion				GetRotation() const;
	void					SetRotation(const Quaternion& Rotation);
	const Matrix4d&			GetWorldMatrix();
	const Matrix4d&			GetInverseWorldMatrix();

	void*					GetEntity();
	void					SetEntity(void *Entity);

	Transform*				GetTransform();

	bool					RayCast(const Vector3d& Origin, const Vector3d &Dir, float* t);
	Matrix3d				GetInertia(float Mass) const;
	Matrix3d				GetInverseInertia(float Mass) const;

protected:
	Box3d					GetBoundingBoxLocal() const;
	Vector3d				GetSupportLocal(const Vector3d& Dir) const;

private:
	GeometryShape	m_Shape;
	Box3d	m_BoxWorld;
	Transform		m_Transform;
	void*			m_Entity;

public:
	DECL_GEOMETRY_OBJ(OrientedBox);
	DECL_GEOMETRY_OBJ(Plane);
	DECL_GEOMETRY_OBJ(Sphere);
	DECL_GEOMETRY_OBJ(Triangle);
	DECL_GEOMETRY_OBJ(Capsule);

	static GetAABBFunc		getaabbTable[GeometryShapeType::COUNT];
	static RayCastFunc		raycastTable[GeometryShapeType::COUNT];
	static SupportFunc		supportTable[GeometryShapeType::COUNT];
	static InertiaFunc		inertiaTable[GeometryShapeType::COUNT];
	static DestoryFunc		destoryTable[GeometryShapeType::COUNT];
};


class GeometryFactory
{
public:
	static Geometry* CreateOBB(const Vector3d& Position, const Vector3d & Bmin, const Vector3d & Bmax);
	static Geometry* CreatePlane(const Vector3d& Position, const Vector3d& Normal, float D);
	static Geometry* CreateSphere(const Vector3d& Position, const Vector3d& Center, float Radius);
	static Geometry* CreateCapsule(const Vector3d& Position, const Vector3d& X1, const Vector3d& X2, float Radius);
	static Geometry* CreateTriangle(const Vector3d& Position, const Vector3d& A, const Vector3d& B, const Vector3d& C);
};


struct RayCastResult
{
	RayCastResult()
	{
		hit = false;
		hitTime = 0.0f;
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