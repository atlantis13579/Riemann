#pragma once

#include "../Maths/Vector3d.h"
#include "../Maths/Matrix3d.h"
#include "../Maths/BoundingBox3d.h"
#include "../Maths/Transform.h"

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
	static void		Destory_##_name(void*);											\

typedef BoundingBox3d	(*GetAABBFunc)		(void*);
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

class GeometryObjectFactory;

class GeometryObject
{
	friend class GeometryObjectFactory;

public:
	~GeometryObject();

	BoundingBox3d			GetBoundingBoxLocal() const;
	const BoundingBox3d&	GetBoundingBoxWorld() const;

	void					SetPosition(const Vector3d& Position);

	bool					RayCast(const Vector3d& Origin, const Vector3d &Dir, float* t);
	Matrix3d				GetInertia(float Mass);
	Vector3d				GetSupport(const Vector3d& Dir);
	static Vector3d			GetSupport(const GeometryObject* Geom1, const GeometryObject* Geom2, const Vector3d& Dir);

protected:
	GeometryObject(const Vector3d& Position, GeometryShapeType _Type, void* _ShapeObj, void* _Entity);\

private:
	GeometryShape	m_Shape;
	BoundingBox3d	m_BoxWorld;
	Transform		m_Transform;
	void*			m_Entity;

public:
	DECL_GEOMETRY_OBJ(AxisAlignedBox3);
	DECL_GEOMETRY_OBJ(Plane);
	// DECL_GEOMETRY_OBJ(Sphere);

	static GetAABBFunc		getaabbTable[GeometryShapeType::COUNT];
	static RayCastFunc		raycastTable[GeometryShapeType::COUNT];
	static SupportFunc		supportTable[GeometryShapeType::COUNT];
	static InertiaFunc		inertiaTable[GeometryShapeType::COUNT];
	static DestoryFunc		destoryTable[GeometryShapeType::COUNT];
};


class GeometryObjectFactory
{
public:
	static GeometryObject* CreateAABB(const Vector3d& Position, const Vector3d & Bmin, const Vector3d & Bmax);
	static GeometryObject* CreatePlane(const Vector3d& Position, const Vector3d& Normal, float D);
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

