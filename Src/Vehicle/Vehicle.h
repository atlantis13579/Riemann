#pragma once

#include "../Maths/Quaternion.h"
#include "../Maths/Vector3d.h"

class VehicleModel
{
public:
	Vector3d	GetPosition() const;
	Quaternion	GetRotation() const;
	Vector3d	GetFrontVector() const;
	Vector3d	GetUpVector() const;
	Vector3d	GetLeftVector() const;
	Vector3d	GetRightVector() const;

	float		GetSteerAngle() const;
	float		GetMaxSteerAngle() const;

	Vector3d	GetCenterOfMass() const;
	Vector3d	GetFrontAxleCenter() const;
	Vector3d	GetRearAxleCenter() const;
	float		GetFrontAxleWidth() const;
	float		GetReartAxleWidth() const;
	float		GetAxleSeparation() const;
};