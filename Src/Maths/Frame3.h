#pragma once

#include "Vector2.h"
#include "Vector3.h"

class Frame3
{
public:
	Frame3() {}
	Frame3(const Vector3 &_origin, const Vector3 &_normal)
	{
		Origin = _origin;
		Normal = _normal.Unit();
		Tangent = Normal.Cross(Vector3::UnitX());
		if (Tangent.SquareLength() < 1e-6f)
		{
			Tangent = Normal.Cross(Vector3::UnitY());
		}
		Binormal = Normal.Cross(Tangent);
	}

	Vector3 OrthogonalCompose(const Vector3& v) const
	{
		return Tangent * v.x + Normal * v.y + Binormal * v.z;
	}

	Vector3 OrthogonalDecompose(const Vector3& p) const
	{
		Vector3 v = p - Origin;
		const float m = v.Length();
		const float x = m * v.Dot(Tangent);
		const float y = m * v.Dot(Normal);
		const float z = m * v.Dot(Binormal);
		return Vector3(x, y, z);
	}

	Vector2 ProjectXZ(const Vector3& p) const
	{
		Vector3 v = OrthogonalDecompose(p);
		return Vector2(v.x, v.z);
	}

	Vector2 ProjectXY(const Vector3& p) const
	{
		Vector3 v = OrthogonalDecompose(p);
		return Vector2(v.x, v.y);
	}

	Vector2 ProjectYZ(const Vector3& p) const
	{
		Vector3 v = OrthogonalDecompose(p);
		return Vector2(v.y, v.z);
	}

	inline const Vector3& GetOrigin() const { return Origin; };

	inline const Vector3& GetAxisX() const { return Tangent; };
	inline const Vector3& GetAxisY() const { return Normal; };
	inline const Vector3& GetAxisZ() const { return Binormal; };

	inline const Vector3& GetTangent() const { return Tangent; };
	inline const Vector3& GetNormal() const { return Normal; };
	inline const Vector3& GetBinormal() const { return Binormal; };

private:
	Vector3	Origin { Vector3::Zero() };
	Vector3	Tangent { Vector3::UnitX() };		// axis x
	Vector3	Normal { Vector3::UnitY() };		// axis y
	Vector3	Binormal { Vector3::UnitZ() };		// axis z
};