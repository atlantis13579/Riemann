#pragma once

#include "../Maths/Vector3d.h"

class Geometry;

class ContactResult
{
public:
	ContactResult()
	{
		SumImpulseNormal = 0.0f;
		SumImpulseTangent1 = 0.0f;
		SumImpulseTangent2 = 0.0f;
		PositionLocal1 = PositionLocal2 = PositionWorld1 = PositionWorld2 = Vector3d::Zero();
	}

	Vector3d	PositionLocal1;
	Vector3d	PositionLocal2;
	Vector3d	PositionWorld1;
	Vector3d	PositionWorld2;
	Vector3d	Normal;
	Vector3d	Tangent1;
	Vector3d	Tangent2;
	Vector3d	RelativePosition1;
	Vector3d	RelativePosition2;

	float		PenetrationDepth;
	float		SumImpulseNormal;
	float		SumImpulseTangent1;
	float		SumImpulseTangent2;
};

#define MAX_CONTACT_POINTS 5

class ContactManifold
{
public:
	Geometry* Geom1 = nullptr;
	Geometry* Geom2 = nullptr;

	ContactResult ContactPoints[MAX_CONTACT_POINTS];
    int NumContactPointCount = 0;

	void	AddNewContact(Geometry* _Geom1, Geometry* _Geom2, const ContactResult& result);
};
