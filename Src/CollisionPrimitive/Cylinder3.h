#pragma once

#include <math.h>
#include <vector>

#include "ShapeType.h"
#include "../Maths/Box3.h"

namespace Geometry
{
	class Cylinder3
	{
	public:
		float		Radius;
		float		Height;
		Vector3		X0, X1;

	public:
		Cylinder3() {}

		Cylinder3(const Vector3& _X0, const Vector3& _X1, float _Radius)
		{
			Init(_X0, _X1, _Radius);
		}

		void Init(const Vector3& _X0, const Vector3& _X1, float _Radius)
		{
			Radius = _Radius;
			Height = (_X0 - _X1).Length();
			X0 = _X0;
			X1 = _X1;
		}

		static constexpr ShapeType	StaticType()
		{
			return ShapeType::CYLINDER;
		}

	public:

		inline Vector3 GetCenter() const
		{
			return (X0 + X1) * 0.5f;
		}

		inline Vector3 GetAxis() const
		{
			return X1 - X0;
		}

		inline Vector3	GetUnitAxis() const
		{
			return (X1 - X0).Unit();
		}

		inline float GetHeight() const
		{
			return Height;
		}

		inline float GetHalfHeight() const
		{
			return Height * 0.5f;
		}

		bool	CalculateVolumeProperties(MassParameters* p, float Density) const
		{
			p->Volume = CalculateVolume();
			p->Mass = p->Volume * Density;
			p->BoundingVolume = CalculateBoundingVolume();
			p->CenterOfMass = p->BoundingVolume.GetCenter();
			p->InertiaMat = CalculateInertiaTensor(p->Mass);
			return true;
		}

		float CalculateVolume() const
		{
			return CalculateVolume(Radius, Height);
		}

		static float CalculateVolume(float Radius, float Height)
		{
			return (float)M_PI * Radius * Radius * Height;
		}

		Box3 CalculateBoundingVolume() const
		{
			Box3 box(Vector3(-Radius, -Height * 0.5f, -Radius), Vector3(Radius, Height * 0.5f, Radius));
			Vector3 center = GetCenter();
			Vector3 axis = GetAxis();
			if (!axis.ParallelTo(Vector3::UnitY()) || center.SquareLength() > 1e-6)
			{
				Quaternion quat;
				quat.FromTwoAxis(Vector3::UnitY(), axis);
				box = Box3::Transform(box, center, quat);
			}
			return box;
		}

		Matrix3 CalculateInertiaTensor(float Mass) const
		{
			return CalculateInertiaTensor(Radius, Height, Mass);
		}

		static Matrix3 CalculateInertiaTensor(float Radius, float Height, float Mass)
		{
			// https://www.wolframalpha.com/input/?i=cylinder
			float RR = Radius * Radius;
			float Diag12 = Mass / 12.0f * (3.0f * RR + Height * Height);
			float Diag3 = Mass / 2.0f * RR;
			return Matrix3(Diag12, Diag12, Diag3);
		}

		bool IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
		{
			if (IntersectRayInfinityLength(Origin, Direction, Radius, t))
			{
				return false;
			}

			if (fabsf(Origin.y + (*t) * Direction.y) <= Height * 0.5f)
			{
				return true;
			}

			if (fabsf(Direction.y) > 1e-6f)
			{
				float h = Direction.y < 0.0f ? (Height * 0.5f - Origin.y) / Direction.y : -(Height * 0.5f + Origin.y) / Direction.y;

				if (h >= 0.0f)
				{
					Vector3 pos = Origin + h * Direction;
					float sq = pos.x * pos.x + pos.z * pos.z;
					if (sq <= Radius * Radius)
					{
						*t = h;
						return true;
					}
				}
			}

			return false;
		}

		static bool IntersectRayInfinityLength(const Vector3& Origin, const Vector3& Direction, float Radius, float* t)
		{
			Vector3 OriginXZ = Vector3(Origin.x, 0.0f, Origin.z);
			float sqr = OriginXZ.SquareLength();
			if (sqr > Radius * Radius)
			{
				Vector3 DirectionXZ = Vector3(Direction.x, 0.0f, Direction.z);
				float a = DirectionXZ.SquareLength();
				float b = 2.0f * OriginXZ.Dot(DirectionXZ);
				float c = sqr - Radius * Radius;

				float roots[2];
				if (Maths::SolveQuadratic(a, b, c) == 0)
				{
					return false;
				}
				float h = std::min(roots[0], roots[1]);
				if (h >= 0.0f)
				{
					*t = h;
					return true;
				}
			}
			*t = 0.0f;
			return true;
		}

		bool IntersectSegment(const Vector3& P0, const Vector3& P1) const
		{
			Vector3 sa, sb;
			Vector3 d = P0 - P1, m = sa - P1, n = sb - sa;
			float md = d.Dot(d);
			float nd = d.Dot(d);
			float dd = d.Dot(d);
			if (md < 0.0f && md + nd < 0.0f)
				return false;
			if (md > dd && md + nd > dd)
				return false;
			float nn = n.Dot(n);
			float mn = n.Dot(m);
			float a = dd * nn - nd * nd;
			float k = m.Dot(m) - Radius * Radius;
			float c = dd * k - md * md;
			if (fabsf(a) < 1e-6f)
			{
				return c <= 0;
			}
			float b = dd * mn - nd * md;
			float discr = b * b - a * c;
			if (discr < 0)
				return false;
			float t = (-b - sqrtf(discr)) / a;
			if (t < 0.0f || t > 1.0f)
				return false;
			if (md + t * nd < 0.0f)
			{
				if (nd <= 0.0f)
					return false;
				t = -md / nd;
				return k + 2 * t * (mn + t * nn) <= 0.0f;
			}
			else if (md + t * nd > dd)
			{
				if (nd >= 0.0f)
					return false;
				t = (dd - md) / nd;
				return k + dd - 2 * md + t * (2 * (mn - nd) + t * nn) <= 0.0f;
			}
			return true;
		}

		// A Fast and Robust GJK Implementation for Collision Detection of Convex Objects - Gino van den Bergen, page 8
		Vector3 GetSupport(const Vector3& Direction) const
		{
			const float sy = Direction.y > 0 ? 1.0f : -1.0f;
			const float o = sqrtf(Direction.x * Direction.x + Direction.z * Direction.z);
			if (o > 1e-6f)
			{
				Vector3 support(Radius * Direction.x / o, sy * Height * 0.5f, Radius * Direction.z / o);
				return support;
			}
			return Vector3(0.0f, sy * Height * 0.5f, 0.0f);
		}

		static const Vector3* GetCylinderFaces()
		{
			// Approximation of top faces
			static const Vector3 CylinderFaces[] =
			{
				Vector3(0.0f,		1.0f,	1.0f),
				Vector3(PI_OVER_4,	1.0f,	PI_OVER_4),
				Vector3(1.0f,		1.0f,	0.0f),
				Vector3(PI_OVER_4,	1.0f,	-PI_OVER_4),
				Vector3(-0.0f,		1.0f,	-1.0f),
				Vector3(-PI_OVER_4,	1.0f,	-PI_OVER_4),
				Vector3(-1.0f,		1.0f,	0.0f),
				Vector3(-PI_OVER_4,	1.0f,	PI_OVER_4)
			};
			return CylinderFaces;
		}

		int GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
		{
			const Vector3* CylinderFaces = GetCylinderFaces();

			const float HalfHeight = Height * 0.5f;
			const float o = sqrtf(Direction.x * Direction.x + Direction.z * Direction.z);

			// side
			if (o * HalfHeight > Radius * fabsf(Direction.y))
			{
				FacePoints[0] = Vector3(-Direction.x * Radius / o, HalfHeight, -Direction.z * Radius / o);
				FacePoints[1] = Vector3(-Direction.x * Radius / o, -HalfHeight, -Direction.z * Radius / o);
				return 2;
			}

			// top or bottom
			const float sy = Direction.y > 0 ? 1.0f : -1.0f;
			Vector3 s(Radius, HalfHeight * sy, Radius);
			int nPts = sizeof(CylinderFaces) / sizeof(CylinderFaces[0]);
			for (int i = 0; i < nPts; ++i)
			{
				FacePoints[i] = Vector3(CylinderFaces[i].x * s.x, CylinderFaces[i].y * s.y, CylinderFaces[i].z * s.z);
			}
			return nPts;
		}

		void GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
		{
			const Vector3* CylinderFaces = GetCylinderFaces();

			int nPts = sizeof(CylinderFaces) / sizeof(CylinderFaces[0]);
			Vector3 s = Vector3(Radius, Height * 0.5f, Radius);
			Vertices.resize(nPts * 2 + 2);
			Normals.resize(nPts * 2 + 2);

			Vertices[2 * nPts] = Vector3(0.0f, Height * 0.5f, 0.0f);
			Vertices[2 * nPts + 1] = Vector3(0.0f, -Height * 0.5f, 0.0f);
			Normals[2 * nPts] = Vector3::UnitY();
			Normals[2 * nPts + 1] = -Vector3::UnitY();
			for (int i = 0; i < nPts; ++i)
			{
				Vector3 p = Vector3(CylinderFaces[i].x * s.x, CylinderFaces[i].y * s.y, CylinderFaces[i].z * s.z);
				Vertices[i] = p;
				Vertices[i + nPts] = Vector3(p.x, -p.y, p.z);
				Normals[i] = (Vertices[2 * i] - Vertices[2 * nPts]).Unit();
				Normals[i + nPts] = (Vertices[2 * i + 1] - Vertices[2 * nPts + 1]).Unit();
			}

			Indices.resize(3 * 4 * nPts);
			for (int i = 0; i < nPts; ++i)
			{
				int j = (i + 1) % nPts;
				Indices[12 * i + 0] = j;
				Indices[12 * i + 1] = i;
				Indices[12 * i + 2] = i + nPts;

				Indices[12 * i + 3] = j + nPts;
				Indices[12 * i + 4] = j;
				Indices[12 * i + 5] = i + nPts;

				Indices[12 * i + 6] = i;
				Indices[12 * i + 7] = j;
				Indices[12 * i + 8] = 2 * nPts;

				Indices[12 * i + 9] = i + nPts;
				Indices[12 * i + 10] = j + nPts;
				Indices[12 * i + 11] = 2 * nPts + 1;
			}
		}

		void GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
		{
			const Vector3* CylinderFaces = GetCylinderFaces();

			int nPts = sizeof(CylinderFaces) / sizeof(CylinderFaces[0]);
			Vector3 s = Vector3(Radius, Height * 0.5f, Radius);
			Vertices.resize(nPts * 2 + 2);
			Vertices[2 * nPts] = Vector3(0.0f, Height * 0.5f, 0.0f);
			Vertices[2 * nPts + 1] = Vector3(0.0f, -Height * 0.5f, 0.0f);
			for (int i = 0; i < nPts; ++i)
			{
				Vector3 p = Vector3(CylinderFaces[i].x * s.x, CylinderFaces[i].y * s.y, CylinderFaces[i].z * s.z);
				Vertices[i] = p;
				Vertices[i + nPts] = Vector3(p.x, -p.y, p.z);
			}

			Indices.resize(2 * 3 * nPts);
			for (int i = 0; i < nPts; ++i)
			{
				int j = (i + 1) % nPts;
				Indices[6 * i + 0] = j;
				Indices[6 * i + 1] = i;

				Indices[6 * i + 2] = j + nPts;
				Indices[6 * i + 3] = i + nPts;

				Indices[6 * i + 4] = i;
				Indices[6 * i + 5] = i + nPts;
			}
		}
	};

	static_assert(sizeof(Cylinder3) == 32, "sizeof(Cylinder3) not right");
}