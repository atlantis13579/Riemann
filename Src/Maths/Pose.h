#pragma once

#include "Vector3.h"
#include "Quaternion.h"
#include "Matrix3.h"

struct Pose
{
	Pose()
	{
		pos = Vector3::Zero();
		quat = Quaternion::One();
	}

	Pose(const Vector3 &_pos, const Quaternion &_quat)
	{
		pos = _pos;
		quat = _quat;
	}

	Pose(const Pose& _pose)
	{
		pos = _pose.pos;
		quat = _pose.quat;
	}

	Vector3		pos;
	Quaternion	quat;
};