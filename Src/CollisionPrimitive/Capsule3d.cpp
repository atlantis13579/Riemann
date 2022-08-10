#include "Capsule3d.h"
#include "AxisAlignedBox3d.h"
#include "Segment3d.h"
#include "../Maths/Maths.h"

bool Capsule3d::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
{
	const Vector3 vx0 = Origin - X0;
	Vector3 Axis = (X1 - X0) / Length;
	const float dp = DotProduct(vx0, Axis);
	if (dp >= -Radius && dp <= Length + Radius)
	{
		const float proj = Clamp<float>(dp, 0, Length);
		const Vector3 proj_pos = Axis * proj;
		const float dist = (vx0 - proj_pos).SquareLength();
		if (dist <= Radius * Radius)
		{
			*t = 0;
			return true;
		}
	}

	const float ad = DotProduct(Axis, Direction);
	const float sqrDist = vx0.SquareLength();
	const float a = 1 - ad * ad;
	const float c = sqrDist - dp * dp - Radius * Radius;

	bool bCheckCaps = false;

	if (c <= 0.f)
	{
		bCheckCaps = true;
	}
	else
	{
		const float b = DotProduct(vx0, Direction) - dp * ad;
		const float delta = b * b - a * c;

		if (delta < 0)
		{
			bCheckCaps = true;
		}
		else
		{
			float time;
			const bool hit = delta < 1e-4;
			if (hit)
			{
				time = (a == 0) ? 0 : (-b / a);

			}
			else
			{
				time = (a == 0) ? 0 : ((-b - sqrtf(delta)) / a);
				if (time < 0)
				{
					return false;
				}
			}

			const Vector3 pos = Origin + Direction * time;
			const float dist = DotProduct(pos - X0, Axis);
			if (dist >= 0 && dist < Length)
			{
				*t = time;
				return true;
			}
			else
			{
				bCheckCaps = !hit;
			}
		}
	}

	if (bCheckCaps)
	{
		Sphere3d sphere1(X0, Radius);
		Sphere3d sphere2(X1, Radius);

		float t1, t2;
		bool hit1 = sphere1.IntersectRay(Origin, Direction, &t1);
		bool hit2 = sphere2.IntersectRay(Origin, Direction, &t2);

		if (hit1 && hit2)
		{
			*t = std::min(t1, t2);
			return true;
		}
		else if (hit1)
		{
			*t = t1;
			return true;
		}
		else if (hit2)
		{
			*t = t2;
			return true;
		}
	}

	return false;
}

bool Capsule3d::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
{
	AxisAlignedBox3d aabb(Bmin, Bmax);
	return aabb.SqrDistanceToSegment(X0, X1) <= Radius * Radius;
}

bool Capsule3d::IntersectSphere(const Vector3& InCenter, float InRadius) const
{
	float SqrDist = Segment3d::SqrDistancePointToSegment(InCenter, X0, X1);
	return SqrDist <= (Radius + InRadius) * (Radius + InRadius);
}

bool Capsule3d::IntersectCapsule(const Vector3& P0, const Vector3& P1, float rRadius) const
{
	if ((P1 - P0).SquareLength() < TINY_NUMBER)
	{
		return IntersectSphere(P0, rRadius);
	}

	const float SqrDist = Segment3d::SqrDistanceSegmentToSegment(P0, P1, X0, X1);
	return SqrDist <= (Radius + rRadius) * (Radius + rRadius);
}

// computes shortest vector going from capsule axis to triangle edge
static Vector3 computeEdgeAxis(const Vector3& p, const Vector3& a, const Vector3& q, const Vector3& b)
{
	const Vector3 T = q - p;
	const float ADotA = a.Dot(a);
	const float ADotB = a.Dot(b);
	const float ADotT = a.Dot(T);
	const float BDotT = b.Dot(T);
	const float BDotB = b.Dot(b);
	
	const float denom = ADotA * BDotB - ADotB * ADotB;

	float t = fabsf(denom) < 1e-6f ? 0.0f : (ADotT  * BDotB - BDotT * ADotB) / denom;
	t = Clamp(t, 0.0f, 1.0f);

	float u = fabsf(BDotB) < 1e-6f ? 0.0f : (t * ADotB - BDotT) / BDotB;

	if (u < 0.0f)
	{
		u = 0.0f;
		t = ADotT / ADotA;
		t = Clamp(t, 0.0f, 1.0f);
	}
	else if (u > 1.0f)
	{
		u = 1.0f;
		t = (ADotB + ADotT) / ADotA;
		t = Clamp(t, 0.0f, 1.0f);
	}
	return T + b*u - a*t;
}

// projections of capsule & triangle overlap on given axis
static bool intersectAxis(const Capsule3d& capsule, const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& axis)
{
	float min0 = capsule.X0.Dot(axis);
	float max0 = capsule.X1.Dot(axis);
	if (min0 > max0)
		std::swap(min0, max0);
	
	const float MR = axis.SquareLength() * capsule.Radius;
	min0 -= MR;
	max0 += MR;

	float min1 = std::min(A.Dot(axis), std::min(B.Dot(axis), C.Dot(axis)));
	float max1 = std::max(A.Dot(axis), std::max(B.Dot(axis), C.Dot(axis)));

	if (max0 < min1 || max1 < min0)
		return false;

	return true;
}

bool Capsule3d::IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
{
	const float sqrDist = Segment3d::SqrDistancePointToSegment(A, X0, X1);
	if (sqrDist <= Radius * Radius)
		return true;
	
	Vector3 N = (A - B).Cross(A - C);
	Vector3 Axis = X1 - X0;

	if(!intersectAxis(*this, A, B, C, N))
		return false;

	if(!intersectAxis(*this, A, B, C, computeEdgeAxis(A, B - A, X0, Axis)))
		return false;

	if(!intersectAxis(*this, A, B, C, computeEdgeAxis(B, C - B, X0, Axis)))
		return false;

	if(!intersectAxis(*this, A, B, C, computeEdgeAxis(C, A - C, X0, Axis)))
		return false;

	return true;
}

Vector3 Capsule3d::GetSupport(const Vector3& X0, const Vector3& X1, float Radius, const Vector3& Direction)
{
	Vector3 Axis = X1 - X0;
	float dp = DotProduct(Direction, Axis);
	Vector3 cap = dp >= 0 ? X1 : X0;
	return cap + Direction.Unit() * Radius;
}

int Capsule3d::GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
{
	Vector3 DirectionXZ = Vector3(Direction.x, 0.0f, Direction.z);

	// hit  top/bottom
	float len = DirectionXZ.Length();
	if (len == 0.0f)
	{
		FacePoints[0] = GetSupport(Direction);
		return 1;
	}

	Vector3 support = (Radius / len) * DirectionXZ;
	Vector3 support_top = Vector3(0, Length * 0.5f, 0) - support;
	Vector3 support_bottom = Vector3(0, -Length * 0.5f, 0) - support;

	float proj_top = support_top.Dot(Direction);
	float proj_bottom = support_bottom.Dot(Direction);

	// near parallel, hit edge
	if (fabsf(proj_top - proj_bottom) < 0.02f * Direction.Length())
	{
		FacePoints[0] = support_top;
		FacePoints[1] = support_bottom;
		return 2;
	}

	FacePoints[0] = GetSupport(Direction);
	return 1;
}

void Capsule3d::GetVertices(int stackCount, int sliceCount, std::vector<Vector3>& Vertices, std::vector<Vector3>* Normals)
{
	const float mPI = 2.0f * asinf(1.0f);

	float phiStep = mPI / stackCount;
	float thetaStep = 2.0f * mPI / sliceCount;
	float Length = (X1 - X0).Length();

	Vertices.push_back(Vector3(0, Length * 0.5f + Radius, 0));
	if (Normals) Normals->push_back(Vector3::UnitY());

	for (int i = 1; i < stackCount; i++)
	{
		float phi = i * phiStep;
		float height = i <= stackCount / 2 ? Length * 0.5f : -Length * 0.5f;
		for (int j = 0; j <= sliceCount; j++)
		{
			float theta = j * thetaStep;
			Vector3 p = Vector3(Radius * sinf(phi) * cosf(theta), height + Radius * cosf(phi), Radius * sinf(phi) * sinf(theta));
			Vertices.push_back(p);
			if (Normals) Normals->push_back(p);
		}
	}
	Vertices.push_back(Vector3(0, -Length * 0.5f - Radius, 0));
	if (Normals) Normals->push_back(-Vector3::UnitY());

	if (!IsYAxisAligned())
	{
		Matrix3 Rot;
		Rot.FromTwoAxis(Vector3::UnitY(), X1 - X0);

		Vector3* pV = Vertices.data();
		for (size_t i = 0; i < Vertices.size(); ++i)
		{
			pV[i] = Rot * pV[i];
		}

		if (Normals)
		{
			Vector3* pN = Normals->data();
			for (size_t i = 0; i < Normals->size(); ++i)
			{
				pN[i] = Rot * pN[i];
			}
		}
	}

	Vector3 Center = GetCenter();
	if (Center.SquareLength() > 0.001f)
	{
		Vector3* pV = Vertices.data();
		for (size_t i = 0; i < Vertices.size(); ++i)
		{
			pV[i] = pV[i] + Center;
		}
	}
}

void Capsule3d::GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
{
	const int stackCount = 5;
	const int sliceCount = 8;

	GetVertices(stackCount, sliceCount, Vertices, &Normals);

	for (int i = 1; i <= sliceCount; i++)
	{
		Indices.push_back(0);
		Indices.push_back(i + 1);
		Indices.push_back(i);
	}

	int baseIndex = 1;
	int Count = sliceCount + 1;
	for (int i = 0; i < stackCount - 2; i++)
	{
		for (int j = 0; j < sliceCount; j++)
		{
			Indices.push_back(baseIndex + i * Count + j);
			Indices.push_back(baseIndex + i * Count + j + 1);
			Indices.push_back(baseIndex + (i + 1) * Count + j);

			Indices.push_back(baseIndex + (i + 1) * Count + j);
			Indices.push_back(baseIndex + i * Count + j + 1);
			Indices.push_back(baseIndex + (i + 1) * Count + j + 1);
		}
	}
	int PoleIndex = (int)Vertices.size() - 1;
	baseIndex = PoleIndex - Count;
	for (int i = 0; i < sliceCount; i++)
	{
		Indices.push_back(PoleIndex);
		Indices.push_back(baseIndex + i);
		Indices.push_back(baseIndex + i + 1);
	}
}

void Capsule3d::GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
{
	const int stackCount = 5;
	const int sliceCount = 8;

	GetVertices(stackCount, sliceCount, Vertices, nullptr);

	for (int i = 1; i <= sliceCount; i++)
	{
		Indices.push_back(0);
		Indices.push_back(i);

		Indices.push_back(i);
		Indices.push_back(i + 1);
	}

	int baseIndex = 1;
	int Count = sliceCount + 1;
	for (int i = 0; i < stackCount - 2; i++)
	{
		for (int j = 0; j < sliceCount; j++)
		{
			Indices.push_back(baseIndex + i * Count + j);
			Indices.push_back(baseIndex + i * Count + j + 1);

			Indices.push_back(baseIndex + i * Count + j + 1);
			Indices.push_back(baseIndex + (i + 1) * Count + j + 1);

			Indices.push_back(baseIndex + (i + 1) * Count + j + 1);
			Indices.push_back(baseIndex + i * Count + j + 1);

			Indices.push_back(baseIndex + i * Count + j + 1);
			Indices.push_back(baseIndex + (i + 1) * Count + j + 1);
		}
	}
	int PoleIndex = (int)Vertices.size() - 1;
	baseIndex = PoleIndex - Count;
	for (int i = 0; i < sliceCount; i++)
	{
		Indices.push_back(PoleIndex);
		Indices.push_back(baseIndex + i);
	}
}
