#pragma once

#include "../Maths/Vector3d.h"

class Geometry;

class ContactResult
{
public:
	ContactResult()
	{
		SumImpulseNormal = 0.0f;
		SumImpulseTangent = 0.0f;
		SumImpulseBinormal = 0.0f;
		PositionLocalA = PositionLocalB = PositionWorldA = PositionWorldB = Vector3d::Zero();
	}

	Vector3d	PositionLocalA;
	Vector3d	PositionLocalB;
	Vector3d	PositionWorldA;
	Vector3d	PositionWorldB;
	Vector3d	Normal;
	Vector3d	Tangent;
	Vector3d	Binormal;

	float		PenetrationDepth;
	float		SumImpulseNormal;
	float		SumImpulseTangent;
	float		SumImpulseBinormal;
};

#define MAX_CONTACT_POINTS 5

class ContactManifold
{
public:
	Geometry* GeomA = nullptr;
	Geometry* GeomB = nullptr;

	ContactResult ContactPoints[MAX_CONTACT_POINTS];
    int NumContactPointCount = 0;

	void	AddNewContact(Geometry* _Geom1, Geometry* _Geom2, const ContactResult& result);
};

class ContactReport
{
public:
	virtual ~ContactReport() {}
};

