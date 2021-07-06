#pragma once

#include "../Maths/Vector3d.h"
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
	static bool RayCast_##_name(void*, const Vector3d&, const Vector3d&, float*);	

typedef BoundingBox3d	(*GetBoundingBoxFunc)	(void*);
typedef bool			(*RayCastFunc)	(void*, const Vector3d&, const Vector3d&, float*);

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
	GeometryObject(GeometryShapeType _Type, void* _Obj)
	{
		Shape.Type = _Type;
		Shape.Object = _Obj;
	}

	GeometryShape Shape;

public:
	static GeometryObject* CreateObject(GeometryShapeType Type, void* Object);

	DECL_GEOMETRY_OBJ(AxisAlignedBox3);
	DECL_GEOMETRY_OBJ(Plane);
	DECL_GEOMETRY_OBJ(Sphere);

	static GetBoundingBoxFunc	getboundingboxTable[GeometryShapeType::COUNT];
	static RayCastFunc			rasycastTable[GeometryShapeType::COUNT];
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
	GeometryObject* Object;
};

