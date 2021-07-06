#pragma once

enum GeometryShapeType
{
	UNKNOWN = 0,
	AABB,
	OBB,
	SPHERE,
	PLANE,
	CAPSULE,
	CONVEX,
	TRIANGLEMESH,
};

struct GeometryShape
{
	GeometryShape()
	{
		ShapeType = GeometryShapeType::UNKNOWN;
	}

	GeometryShapeType	ShapeType;
	void				*Object;
};

struct GeometryObject
{
	GeometryShape Shape;
};
