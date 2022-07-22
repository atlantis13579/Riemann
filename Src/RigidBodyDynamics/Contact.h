#pragma once

#include <vector>
#include "../Maths/Vector3.h"
#include "../Core/BatchList.h"

class Geometry;

class Contact
{
public:
	Contact()
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

#define MAX_CONTACT_POINTS 4

class ContactManifold
{
public:
	Geometry* GeomA = nullptr;
	Geometry* GeomB = nullptr;

	Contact ContactPoints[MAX_CONTACT_POINTS];
    int		NumContactPointCount = 0;

	void	AddNewContact(Geometry* _Geom1, Geometry* _Geom2, const Contact& result);
};

class ContactManifoldManager
{
public:
	ContactManifoldManager();
	~ContactManifoldManager();

private:
	BatchList<ContactManifold>	mManifolds;
};

class ContactReport
{
public:
	virtual ~ContactReport() {}
};

