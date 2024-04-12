#pragma once

#include <math.h>
#include <stdint.h>
#include <vector>

#undef max
#undef min

#include "ShapeType.h"
#include "AxisAlignedBox3.h"
#include "Sphere3.h"
#include "../Maths/Matrix3.h"

namespace Riemann
{
	// {x | (x - [X0 + (X1 - X0)*t])^2 <= Radius }, 0 <= t <= 1 }
	class Capsule3
	{
	public:
		Vector3		X0;
		Vector3		X1;
		float		Radius;
		float		Length;

	public:
		Capsule3() {}

		Capsule3(const Vector3& _X0, const Vector3& _X1, float _Radius)
		{
			Init(_X0, _X1, _Radius);
		}

		static constexpr ShapeType	StaticType()
		{
			return ShapeType::CAPSULE;
		}

		inline void		Init(const Vector3& _X0, const Vector3& _X1, float _Radius)
		{
			X0 = _X0;
			X1 = _X1;
			Radius = _Radius;
			Length = (_X1 - _X0).Length();
		}

	public:
		inline Vector3	GetAxis() const
		{
			return X1 - X0;
		}

		inline Vector3	GetUnitAxis() const
		{
			return (X1 - X0).Unit();
		}

		inline Vector3	GetCenter() const
		{
			return (X0 + X1) * 0.5f;
		}

		inline float	GetHeight() const
		{
			return Length;
		}

		inline float	GetHalfHeight() const
		{
			return Length * 0.5f;
		}

		Box3 CalculateBoundingVolume() const
		{
			Box3 box(X0, X0);
			box.Encapsulate(X1);
			box.Thicken(Radius);
			return box;
		}

		bool IsYAxisAligned() const
		{
			return (X1 - X0).ParallelTo(Vector3::UnitY());
		}

		bool IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
		{
			const Vector3 vx0 = Origin - X0;
			Vector3 Axis = (X1 - X0) / Length;
			const float dp = DotProduct(vx0, Axis);
			if (dp >= -Radius && dp <= Length + Radius)
			{
				const float proj = Maths::Clamp<float>(dp, 0, Length);
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
				Sphere3 sphere1(X0, Radius);
				Sphere3 sphere2(X1, Radius);

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

		bool IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
		{
			AxisAlignedBox3 aabb(Bmin, Bmax);
			return aabb.SqrDistanceToSegment(X0, X1) <= Radius * Radius;
		}

		bool IntersectSphere(const Vector3& InCenter, float InRadius) const
		{
			float SqrDist = Segment3::SqrDistancePointToSegment(InCenter, X0, X1);
			return SqrDist <= (Radius + InRadius) * (Radius + InRadius);
		}

		bool IntersectSegment(const Vector3& P0, const Vector3& P1) const
		{
			if ((P1 - P0).SquareLength() < TINY_NUMBER)
			{
				float SqrDist = Segment3::SqrDistancePointToSegment(P0, X0, X1);
				return SqrDist <= Radius * Radius;
			}

			const float SqrDist = Segment3::SqrDistanceSegmentToSegment(P0, P1, X0, X1);
			return SqrDist <= Radius * Radius;
		}

		bool IntersectCapsule(const Vector3& P0, const Vector3& P1, float rRadius) const
		{
			if ((P1 - P0).SquareLength() < TINY_NUMBER)
			{
				return IntersectSphere(P0, rRadius);
			}

			const float SqrDist = Segment3::SqrDistanceSegmentToSegment(P0, P1, X0, X1);
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

			float t = fabsf(denom) < 1e-6f ? 0.0f : (ADotT * BDotB - BDotT * ADotB) / denom;
			t = Maths::Clamp(t, 0.0f, 1.0f);

			float u = fabsf(BDotB) < 1e-6f ? 0.0f : (t * ADotB - BDotT) / BDotB;

			if (u < 0.0f)
			{
				u = 0.0f;
				t = ADotT / ADotA;
				t = Maths::Clamp(t, 0.0f, 1.0f);
			}
			else if (u > 1.0f)
			{
				u = 1.0f;
				t = (ADotB + ADotT) / ADotA;
				t = Maths::Clamp(t, 0.0f, 1.0f);
			}
			return T + b * u - a * t;
		}

		bool IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
		{
			const float sqrDist = Segment3::SqrDistancePointToSegment(A, X0, X1);
			if (sqrDist <= Radius * Radius)
				return true;

			Vector3 N = (A - B).Cross(A - C);
			Vector3 Axis = X1 - X0;

			if (!intersectAxis(*this, A, B, C, N))
				return false;

			if (!intersectAxis(*this, A, B, C, computeEdgeAxis(A, B - A, X0, Axis)))
				return false;

			if (!intersectAxis(*this, A, B, C, computeEdgeAxis(B, C - B, X0, Axis)))
				return false;

			if (!intersectAxis(*this, A, B, C, computeEdgeAxis(C, A - C, X0, Axis)))
				return false;

			return true;
		}

		bool PenetrateSphere(const Vector3& rCenter, float rRadius, Vector3* Normal, float* Depth) const
		{
			const float s = rRadius + Radius;
			Vector3 Closest = Segment3::ClosestPointOnSegmentToPoint(rCenter, X0, X1);
			if ((rCenter - Closest).SquareLength() > s * s)
			{
				return false;
			}

			*Normal = rCenter - Closest;

			const float len = Normal->Length();
			if (len < 1e-6f)
			{
				*Normal = Vector3::UnitY();
			}
			else
			{
				*Normal /= len;
			}
			*Normal = -*Normal;
			*Depth = s - len;
			return true;
		}

		bool PenetrateCapsule(const Vector3& P0, const Vector3& P1, float rRadius, Vector3* Normal, float* Depth) const
		{
			Vector3 s, t;
			Segment3::ClosestPointsBetweenSegmentsEx(X0, X1, P0, P1, s, t);

			const float r2 = Radius + rRadius;

			if ((s - t).SquareLength() > r2 * r2)
			{
				return false;
			}

			*Normal = s - t;

			const float len = Normal->Length();
			if (len < 1e-6f)
			{
				*Normal = Vector3::UnitY();
			}
			else
			{
				*Normal /= len;
			}
			*Depth = r2 - len;
			return true;
		}

		bool			CalculateVolumeProperties(MassParameters* p, float Density) const
		{
			p->Volume = CalculateVolume();
			p->Mass = p->Volume * Density;
			p->BoundingVolume = CalculateBoundingVolume();
			p->CenterOfMass = p->BoundingVolume.GetCenter();
			p->InertiaMat = CalculateInertiaTensor(p->Mass);
			return true;
		}

		float			CalculateVolume() const
		{
			return CalculateVolume(Radius, GetHeight());
		}

		static float	CalculateVolume(float Radius, float Height)
		{
			return (float)M_PI * Radius * Radius * (Height + 4.0f * Radius / 3.0f);
		}

		Matrix3			CalculateInertiaTensor(float Mass) const
		{
			return CalculateInertiaTensor(Radius, GetHeight(), Mass);
		}

		static Matrix3	CalculateInertiaTensor(float Radius, float Height, float Mass)
		{
			// https://www.wolframalpha.com/input/?i=capsule&assumption=%7B%22C%22,+%22capsule%22%7D+-%3E+%7B%22Solid%22%7D
			float R = Radius;
			float H = Height;
			float RR = R * R;
			float HH = H * H;

			// (5H^3 + 20*H^2R + 45HR^2 + 32R^3) / (60H + 80R)
			float Diag12 = Mass * (5.0f * HH * H + 20.0f * HH * R + 45.0f * H * RR + 32.0f * RR * R) / (60.0f * H + 80.0f * R);
			// (R^2 * (15H + 16R) / (30H +40R))
			float Diag3 = Mass * (RR * (15.0f * H + 16.0f * R)) / (30.0f * H + 40.0f * R);

			return Matrix3(Diag12, Diag12, Diag3);
		}

		Vector3	GetSupport(const Vector3& Direction) const
		{
			return GetSupport(X0, X1, Radius, Direction);
		}

		static Vector3 GetSupport(const Vector3& X0, const Vector3& X1, float Radius, const Vector3& Direction)
		{
			Vector3 Axis = X1 - X0;
			float dp = DotProduct(Direction, Axis);
			Vector3 cap = dp >= 0 ? X1 : X0;
			return cap + Direction.Unit() * Radius;
		}

		int GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
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

		void GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
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

		void GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
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

	private:

		void GetVertices(int stackCount, int sliceCount, std::vector<Vector3>& Vertices, std::vector<Vector3>* Normals)
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

	private:
		// projections of capsule & triangle overlap on given axis
		static bool intersectAxis(const Capsule3& capsule, const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& axis)
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

	};
}