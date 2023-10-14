#pragma once

#include "Vector3.h"
#include "Quaternion.h"
#include "Matrix3.h"

struct Pose
{
	Vector3		pos;
	Quaternion	quat;
};