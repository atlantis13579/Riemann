#pragma once

#include <vector>
#include "../Maths/Vector3.h"
#include "../Core/BatchList.h"

class Geometry;

struct Contact
{
public:
	Contact()
	{
		totalImpulseNormal = 0.0f;
		totalImpulseTangent = 0.0f;
		totalImpulseBinormal = 0.0f;
		PositionLocalA = PositionLocalB = PositionWorldA = PositionWorldB = Vector3d::Zero();
	}

	Vector3d	PositionWorldA;
	Vector3d	PositionWorldB;
	Vector3d	PositionLocalA;
	Vector3d	PositionLocalB;
	Vector3d	Normal;
	Vector3d	Tangent;
	Vector3d	Binormal;
	float		PenetrationDepth;

	//	For warm start
	float		totalImpulseNormal;
	float		totalImpulseTangent;
	float		totalImpulseBinormal;
};

#define MAX_CONTACT_POINTS 4

class ContactManifold
{
public:
	Geometry* GeomA = nullptr;
	Geometry* GeomB = nullptr;

	Contact ContactPoints[MAX_CONTACT_POINTS + 1];
    int		NumContactPointCount = 0;

	void	AddNewContact(Geometry* _Geom1, Geometry* _Geom2, const Contact& result);

private:
	bool	MergeManifold();
	bool	FindBest4();
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

