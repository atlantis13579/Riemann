#pragma once

#include "../../Maths/Quaternion.h"
#include "../../Maths/Vector3.h"

class Vehicle
{
public:
	Vector3		GetPosition() const;
	Quaternion	GetRotation() const;
	Vector3		GetFrontVector() const;
	Vector3		GetUpVector() const;
	Vector3		GetLeftVector() const;
	Vector3		GetRightVector() const;

	float		GetSteerAngle() const;
	float		GetMaxSteerAngle() const;

	Vector3		GetCenterOfMass() const;
	Vector3		GetFrontAxleCenter() const;
	Vector3		GetRearAxleCenter() const;
	float		GetFrontAxleWidth() const;
	float		GetReartAxleWidth() const;
	float		GetAxleLength() const;
};
