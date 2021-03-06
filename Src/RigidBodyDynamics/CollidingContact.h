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
		PositionLocalA = PositionLocalB = PositionWorldA = PositionWorldB = Vector3::Zero();
	}

	Vector3	PositionWorldA;
	Vector3	PositionWorldB;
	Vector3	PositionLocalA;
	Vector3	PositionLocalB;
	Vector3	Normal;
	Vector3	Tangent;
	Vector3	Binormal;
	float	PenetrationDepth;

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
	ContactManifold *next = nullptr;

	void	Reset()
	{
		NumContactPointCount = 0;
	}

	Contact ContactPoints[MAX_CONTACT_POINTS + 1];
    int		NumContactPointCount = 0;

	void	AddNewContact(Geometry* _Geom1, Geometry* _Geom2, const Contact& result);

private:
	bool	MergeManifold();
	bool	MergeManifold2();
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

