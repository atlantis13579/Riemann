#pragma once

#include "../Maths/Vector3d.h"
#include "../Maths/Matrix3d.h"
#include "../Maths/BoundingBox3d.h"

enum GeometryShapeType
{
	UNKNOWN = 0,
	AABB,
	PLANE,
	SPHERE,
	OBB,
	CAPSULE,
	CONVEX,
	CYLINDER,
	TRIANGLEMESH,
	COUNT,
};

#define	DECL_GEOMETRY_OBJ(_name)													\
	static BoundingBox3d GetBoundingBox_##_name(void *);							\
	static bool RayCast_##_name(void*, const Vector3d&, const Vector3d&, float*);	\
	static Vector3d	GetSupport_##_name(void*, const Vector3d&);						\
	static Matrix3d	GetInertia_##_name(void*, float);								\

typedef BoundingBox3d	(*GetAABBFunc)		(void*);
typedef bool			(*RayCastFunc)		(void*, const Vector3d&, const Vector3d&, float*);
typedef Vector3d		(*SupportFunc)		(void*, const Vector3d&);
typedef Matrix3d		(*InertiaFunc)		(void*, float);

struct GeometryShape
{
	GeometryShape()
	{
		Type = GeometryShapeType::UNKNOWN;
		Object = nullptr;
	}
	~GeometryShape()
	{
		Object = nullptr;	// Not Hold the memory
	}

	GeometryShapeType	Type;
	void				*Object;
};

class GeometryObject
{
public:
	GeometryObject(GeometryShapeType _Type, void* _ShapeObj, void *_Entity = nullptr)
	{
		Shape.Type = _Type;
		Shape.Object = _ShapeObj;
		Entity = _Entity;
	}

	GeometryShape	Shape;
	void*			Entity;

public:
	static GeometryObject*	CreateObject(GeometryShapeType Type, void* ShapeObj, void *Entity);
	static BoundingBox3d	GetBoundingBox(const GeometryObject* Geom);
	static bool				RayCast(const GeometryObject *Geom, const Vector3d& Origin, const Vector3d &Dir, float* t);
	static Vector3d			GetSupport(const GeometryObject* Geom, const Vector3d& Dir);
	static Vector3d			GetSupport(const GeometryObject* Geom1, const GeometryObject* Geom2, const Vector3d& Dir);
	static Matrix3d			GetInertia(const GeometryObject* Geom, float Mass);

	DECL_GEOMETRY_OBJ(AxisAlignedBox3);
	// DECL_GEOMETRY_OBJ(Plane);
	// DECL_GEOMETRY_OBJ(Sphere);

	static GetAABBFunc		getaabbTable[GeometryShapeType::COUNT];
	static RayCastFunc		raycastTable[GeometryShapeType::COUNT];
	static SupportFunc		supportTable[GeometryShapeType::COUNT];
	static InertiaFunc		inertiaTable[GeometryShapeType::COUNT];
};

struct RayCastResult
{
	RayCastResult()
	{
		hit = false;
		t = 0.0f;
		Object = nullptr;
	}

	bool  hit;
	float t;
	Vector3d hitPos;
	Vector3d hitNormal;
	GeometryObject* Object;
};

