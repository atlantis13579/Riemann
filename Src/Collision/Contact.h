#pragma once

#include "../Maths/Vector3d.h"

class Geometry;

class ContactResult
{
public:
	enum eStatus
	{
		Separated,   // Shapes doesnt penetrate	
		Penetrating, // Shapes are penetrating	
		GJK_Failed,  // GJK phase fail, no big issue, shapes are probably just touching
		EPA_Failed   // EPA phase fail, bigger problem, need to save parameters, and debug
	} status;

	Geometry* Geom1 = nullptr;
	Geometry* Geom2 = nullptr;
	Vector3d WitnessLocal1;
	Vector3d WitnessLocal2;
	Vector3d WitnessWorld1;
	Vector3d WitnessWorld2;
	Vector3d Normal;
	Vector3d Tangent;
	Vector3d Tangent2;

	float   PenetrationDistance;
	float   ImpulseNormal;
	float   ImpulseTangent1;
	float   ImpulseTangent2;
};


#define MAX_CONTACT_POINTS 5

class ContactManifold
{
public:
	Geometry* Geom1 = nullptr;
	Geometry* Geom2 = nullptr;

	ContactResult ContactPoints[MAX_CONTACT_POINTS];
    int NumContactPointCount = 0;
};