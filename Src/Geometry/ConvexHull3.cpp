
#include "ConvexHull3.h"
#include "../Maths/Maths.h"

#include <algorithm>
#include <limits>

namespace Riemann
{
	struct HullTriangle
	{
		int v[3];
		Vector3 normal;
		float d;
		bool deleted;
	};

	struct DirectedHullEdge
	{
		int a;
		int b;
	};

	static float ResolveHullTolerance(const std::vector<Vector3>& points, float requestedTolerance)
	{
		if (points.empty())
		{
			return std::max(requestedTolerance, 1e-6f);
		}

		Vector3 boundsMin = points[0];
		Vector3 boundsMax = points[0];
		for (const Vector3& p : points)
		{
			boundsMin = boundsMin.Min(p);
			boundsMax = boundsMax.Max(p);
		}

		const float diagonal = (boundsMax - boundsMin).Length();
		return std::max(std::max(requestedTolerance, diagonal * 1e-6f), 1e-7f);
	}

	static void BuildUniquePointSet(const Vector3* points, int numPoints, float tolerance, std::vector<Vector3>& uniquePoints)
	{
		uniquePoints.clear();
		if (points == nullptr || numPoints <= 0)
		{
			return;
		}

		const float toleranceSqr = tolerance * tolerance;
		uniquePoints.reserve(numPoints);
		for (int pointIndex = 0; pointIndex < numPoints; ++pointIndex)
		{
			const Vector3& point = points[pointIndex];
			bool found = false;
			for (const Vector3& existing : uniquePoints)
			{
				if ((existing - point).SquareLength() <= toleranceSqr)
				{
					found = true;
					break;
				}
			}
			if (!found)
			{
				uniquePoints.push_back(point);
			}
		}
	}

	static bool FindInitialTetrahedron(const std::vector<Vector3>& points, float tolerance, int outIndices[4])
	{
		const int pointCount = (int)points.size();
		if (pointCount < 4)
		{
			return false;
		}

		int minIndex[3] = { 0, 0, 0 };
		int maxIndex[3] = { 0, 0, 0 };
		for (int pointIndex = 1; pointIndex < pointCount; ++pointIndex)
		{
			for (int axis = 0; axis < 3; ++axis)
			{
				if (points[pointIndex][axis] < points[minIndex[axis]][axis])
				{
					minIndex[axis] = pointIndex;
				}
				if (points[pointIndex][axis] > points[maxIndex[axis]][axis])
				{
					maxIndex[axis] = pointIndex;
				}
			}
		}

		int candidateIndices[6] = { minIndex[0], maxIndex[0], minIndex[1], maxIndex[1], minIndex[2], maxIndex[2] };
		int i0 = -1;
		int i1 = -1;
		float maxDistanceSqr = -1.0f;
		for (int a = 0; a < 6; ++a)
		{
			for (int b = a + 1; b < 6; ++b)
			{
				const float distanceSqr = (points[candidateIndices[a]] - points[candidateIndices[b]]).SquareLength();
				if (distanceSqr > maxDistanceSqr)
				{
					maxDistanceSqr = distanceSqr;
					i0 = candidateIndices[a];
					i1 = candidateIndices[b];
				}
			}
		}

		if (i0 < 0 || i1 < 0 || maxDistanceSqr <= tolerance * tolerance)
		{
			return false;
		}

		const Vector3 edge = points[i1] - points[i0];
		int i2 = -1;
		float maxLineDistanceScaled = -1.0f;
		for (int pointIndex = 0; pointIndex < pointCount; ++pointIndex)
		{
			if (pointIndex == i0 || pointIndex == i1)
			{
				continue;
			}

			const float distanceScaled = (points[pointIndex] - points[i0]).Cross(edge).SquareLength();
			if (distanceScaled > maxLineDistanceScaled)
			{
				maxLineDistanceScaled = distanceScaled;
				i2 = pointIndex;
			}
		}

		if (i2 < 0 || maxLineDistanceScaled <= tolerance * tolerance * edge.SquareLength())
		{
			return false;
		}

		Vector3 planeNormal = (points[i1] - points[i0]).Cross(points[i2] - points[i0]);
		if (planeNormal.SafeNormalize() <= tolerance)
		{
			return false;
		}

		int i3 = -1;
		float maxPlaneDistance = -1.0f;
		for (int pointIndex = 0; pointIndex < pointCount; ++pointIndex)
		{
			if (pointIndex == i0 || pointIndex == i1 || pointIndex == i2)
			{
				continue;
			}

			const float distance = fabsf(planeNormal.Dot(points[pointIndex] - points[i0]));
			if (distance > maxPlaneDistance)
			{
				maxPlaneDistance = distance;
				i3 = pointIndex;
			}
		}

		if (i3 < 0 || maxPlaneDistance <= tolerance)
		{
			return false;
		}

		outIndices[0] = i0;
		outIndices[1] = i1;
		outIndices[2] = i2;
		outIndices[3] = i3;
		return true;
	}

	static bool MakeHullTriangle(
		const std::vector<Vector3>& points,
		int a,
		int b,
		int c,
		const Vector3& interiorPoint,
		float tolerance,
		HullTriangle& outTriangle)
	{
		if (a == b || a == c || b == c)
		{
			return false;
		}

		Vector3 normal = (points[b] - points[a]).Cross(points[c] - points[a]);
		if (normal.SafeNormalize() <= tolerance)
		{
			return false;
		}

		float d = -normal.Dot(points[a]);
		if (normal.Dot(interiorPoint) + d > 0.0f)
		{
			std::swap(b, c);
			normal = -normal;
			d = -normal.Dot(points[a]);
		}

		outTriangle.v[0] = a;
		outTriangle.v[1] = b;
		outTriangle.v[2] = c;
		outTriangle.normal = normal;
		outTriangle.d = d;
		outTriangle.deleted = false;
		return true;
	}

	static void AddHorizonEdge(std::vector<DirectedHullEdge>& horizon, int a, int b)
	{
		for (std::vector<DirectedHullEdge>::iterator it = horizon.begin(); it != horizon.end(); ++it)
		{
			if (it->a == b && it->b == a)
			{
				horizon.erase(it);
				return;
			}
		}
		horizon.push_back({ a, b });
	}

	static bool AddPointToHull(
		const std::vector<Vector3>& points,
		int pointIndex,
		const Vector3& interiorPoint,
		float tolerance,
		std::vector<HullTriangle>& triangles)
	{
		std::vector<int> visibleTriangles;
		visibleTriangles.reserve(triangles.size());
		for (int triangleIndex = 0; triangleIndex < (int)triangles.size(); ++triangleIndex)
		{
			const HullTriangle& triangle = triangles[triangleIndex];
			if (!triangle.deleted && triangle.normal.Dot(points[pointIndex]) + triangle.d > tolerance)
			{
				visibleTriangles.push_back(triangleIndex);
			}
		}

		if (visibleTriangles.empty())
		{
			return true;
		}

		std::vector<DirectedHullEdge> horizon;
		horizon.reserve(visibleTriangles.size() * 3);
		for (int triangleIndex : visibleTriangles)
		{
			const HullTriangle& triangle = triangles[triangleIndex];
			AddHorizonEdge(horizon, triangle.v[0], triangle.v[1]);
			AddHorizonEdge(horizon, triangle.v[1], triangle.v[2]);
			AddHorizonEdge(horizon, triangle.v[2], triangle.v[0]);
		}

		if (horizon.empty())
		{
			return false;
		}

		for (int triangleIndex : visibleTriangles)
		{
			triangles[triangleIndex].deleted = true;
		}

		for (const DirectedHullEdge& edge : horizon)
		{
			HullTriangle newTriangle;
			if (MakeHullTriangle(points, edge.a, edge.b, pointIndex, interiorPoint, tolerance, newTriangle))
			{
				triangles.push_back(newTriangle);
			}
		}

		return true;
	}

	static bool ExportHullTriangles(const std::vector<Vector3>& points, const std::vector<HullTriangle>& triangles, ConvexHull3d& outHull)
	{
		outHull.verts.clear();
		outHull.faces.clear();

		std::vector<int> remap(points.size(), -1);
		int activeFaceCount = 0;
		for (const HullTriangle& triangle : triangles)
		{
			if (triangle.deleted)
			{
				continue;
			}

			++activeFaceCount;
			for (int corner = 0; corner < 3; ++corner)
			{
				const int sourceIndex = triangle.v[corner];
				if (remap[sourceIndex] < 0)
				{
					if (outHull.verts.size() >= (size_t)std::numeric_limits<short>::max())
					{
						outHull.verts.clear();
						outHull.faces.clear();
						return false;
					}
					remap[sourceIndex] = (int)outHull.verts.size();
					outHull.verts.push_back(points[sourceIndex]);
				}
			}
		}

		if (activeFaceCount < 4 || outHull.verts.size() < 4)
		{
			outHull.verts.clear();
			return false;
		}

		outHull.faces.reserve(activeFaceCount);
		for (const HullTriangle& triangle : triangles)
		{
			if (triangle.deleted)
			{
				continue;
			}

			HullFace3d face;
			face.norm = triangle.normal;
			face.w = triangle.d;
			face.verts.resize(3);

			// ConvexMesh faces in this codebase use the opposite triangle winding
			// from the normal's cross-product winding.
			face.verts[0] = (short)remap[triangle.v[0]];
			face.verts[1] = (short)remap[triangle.v[2]];
			face.verts[2] = (short)remap[triangle.v[1]];
			outHull.faces.push_back(face);
		}

		return true;
	}

	bool BuildConvexHull3(const Vector3* Points, int NumPoints, ConvexHull3d& OutHull, const ConvexHull3Options& Options)
	{
		OutHull.verts.clear();
		OutHull.faces.clear();
		if (Points == nullptr || NumPoints < 4)
		{
			return false;
		}

		std::vector<Vector3> rawPoints(Points, Points + NumPoints);
		const float tolerance = ResolveHullTolerance(rawPoints, Options.DistanceTolerance);

		std::vector<Vector3> points;
		BuildUniquePointSet(Points, NumPoints, tolerance, points);
		if (points.size() < 4)
		{
			return false;
		}

		int tetra[4] = { -1, -1, -1, -1 };
		if (!FindInitialTetrahedron(points, tolerance, tetra))
		{
			return false;
		}

		const Vector3 interiorPoint = (points[tetra[0]] + points[tetra[1]] + points[tetra[2]] + points[tetra[3]]) * 0.25f;
		std::vector<HullTriangle> triangles;
		triangles.reserve(points.size() * 2);

		HullTriangle triangle;
		if (MakeHullTriangle(points, tetra[0], tetra[1], tetra[2], interiorPoint, tolerance, triangle))
		{
			triangles.push_back(triangle);
		}
		if (MakeHullTriangle(points, tetra[0], tetra[3], tetra[1], interiorPoint, tolerance, triangle))
		{
			triangles.push_back(triangle);
		}
		if (MakeHullTriangle(points, tetra[0], tetra[2], tetra[3], interiorPoint, tolerance, triangle))
		{
			triangles.push_back(triangle);
		}
		if (MakeHullTriangle(points, tetra[1], tetra[3], tetra[2], interiorPoint, tolerance, triangle))
		{
			triangles.push_back(triangle);
		}

		if (triangles.size() != 4)
		{
			return false;
		}

		for (int pointIndex = 0; pointIndex < (int)points.size(); ++pointIndex)
		{
			if (pointIndex == tetra[0] || pointIndex == tetra[1] || pointIndex == tetra[2] || pointIndex == tetra[3])
			{
				continue;
			}

			if (!AddPointToHull(points, pointIndex, interiorPoint, tolerance, triangles))
			{
				return false;
			}
		}

		return ExportHullTriangles(points, triangles, OutHull);
	}

	bool BuildConvexHull3(const std::vector<Vector3>& Points, ConvexHull3d& OutHull, const ConvexHull3Options& Options)
	{
		return BuildConvexHull3(Points.data(), (int)Points.size(), OutHull, Options);
	}

	// http://number-none.com/blow/inertia/deriving_i.html
	Matrix3 ComputePointCloudInertiaTensor_PCA(const Vector3* Vertices, int NumVertices)
	{
		Matrix3 covariance_matrix = Matrix3::ComputeCovarianceMatrix(Vertices, NumVertices);

		float eigens[3];
		Vector3 eigen_vectors[3];
		covariance_matrix.SolveEigenSymmetric(eigens, eigen_vectors);

		if (Determinant(eigen_vectors[0], eigen_vectors[1], eigen_vectors[2]) < 0)
		{
			eigen_vectors[2] = -eigen_vectors[2];
		}

		Matrix3 Inertia = Matrix3(eigen_vectors[0].x, eigen_vectors[1].x, eigen_vectors[2].x,
			eigen_vectors[0].y, eigen_vectors[1].y, eigen_vectors[2].y,
			eigen_vectors[0].z, eigen_vectors[1].z, eigen_vectors[2].z);

		return Inertia;
	}



	// https://people.eecs.berkeley.edu/~jfc/mirtich/massProps.html
	// https://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
	class PolyhedralMassProperties
	{
	public:
		void compProjectionIntegrals(const ConvexHull3d& hull, const HullFace3d& f)
		{
			float a0, a1, da;
			float b0, b1, db;
			float a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
			float a1_2, a1_3, b1_2, b1_3;
			float C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
			float Cab, Kab, Caab, Kaab, Cabb, Kabb;

			P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

			for (size_t i = 0; i < f.verts.size(); i++)
			{
				a0 = hull.verts[f.verts[i]][A];
				b0 = hull.verts[f.verts[i]][B];
				a1 = hull.verts[f.verts[(i + 1) % f.verts.size()]][A];
				b1 = hull.verts[f.verts[(i + 1) % f.verts.size()]][B];
				da = a1 - a0;
				db = b1 - b0;
				a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
				b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
				a1_2 = a1 * a1; a1_3 = a1_2 * a1;
				b1_2 = b1 * b1; b1_3 = b1_2 * b1;

				C1 = a1 + a0;
				Ca = a1 * C1 + a0_2; Caa = a1 * Ca + a0_3; Caaa = a1 * Caa + a0_4;
				Cb = b1 * (b1 + b0) + b0_2; Cbb = b1 * Cb + b0_3; Cbbb = b1 * Cbb + b0_4;
				Cab = 3 * a1_2 + 2 * a1 * a0 + a0_2; Kab = a1_2 + 2 * a1 * a0 + 3 * a0_2;
				Caab = a0 * Cab + 4 * a1_3; Kaab = a1 * Kab + 4 * a0_3;
				Cabb = 4 * b1_3 + 3 * b1_2 * b0 + 2 * b1 * b0_2 + b0_3;
				Kabb = b1_3 + 2 * b1_2 * b0 + 3 * b1 * b0_2 + 4 * b0_3;

				P1 += db * C1;
				Pa += db * Ca;
				Paa += db * Caa;
				Paaa += db * Caaa;
				Pb += da * Cb;
				Pbb += da * Cbb;
				Pbbb += da * Cbbb;
				Pab += db * (b1 * Cab + b0 * Kab);
				Paab += db * (b1 * Caab + b0 * Kaab);
				Pabb += da * (a1 * Cabb + a0 * Kabb);
			}

			P1 /= 2.0;
			Pa /= 6.0;
			Paa /= 12.0;
			Paaa /= 20.0;
			Pb /= -6.0;
			Pbb /= -12.0;
			Pbbb /= -20.0;
			Pab /= 24.0;
			Paab /= 60.0;
			Pabb /= -60.0;
		}

		void compFaceIntegrals(const ConvexHull3d& hull, const HullFace3d& f)
		{
			Vector3 n;
			float w;
			float k1, k2, k3, k4;

			compProjectionIntegrals(hull, f);

			w = f.w;
			n = f.norm;
			k1 = 1 / n[C]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

			Fa = k1 * Pa;
			Fb = k1 * Pb;
			Fc = -k2 * (n[A] * Pa + n[B] * Pb + w * P1);

			Faa = k1 * Paa;
			Fbb = k1 * Pbb;
			Fcc = k3 * (Maths::Sqr(n[A]) * Paa + 2 * n[A] * n[B] * Pab + Maths::Sqr(n[B]) * Pbb
				+ w * (2 * (n[A] * Pa + n[B] * Pb) + w * P1));

			Faaa = k1 * Paaa;
			Fbbb = k1 * Pbbb;
			Fccc = -k4 * (Maths::Cube(n[A]) * Paaa + 3 * Maths::Sqr(n[A]) * n[B] * Paab
				+ 3 * n[A] * Maths::Sqr(n[B]) * Pabb + Maths::Cube(n[B]) * Pbbb
				+ 3 * w * (Maths::Sqr(n[A]) * Paa + 2 * n[A] * n[B] * Pab + Maths::Sqr(n[B]) * Pbb)
				+ w * w * (3 * (n[A] * Pa + n[B] * Pb) + w * P1));

			Faab = k1 * Paab;
			Fbbc = -k2 * (n[A] * Pabb + n[B] * Pbbb + w * Pbb);
			Fcca = k3 * (Maths::Sqr(n[A]) * Paaa + 2 * n[A] * n[B] * Paab + Maths::Sqr(n[B]) * Pabb
				+ w * (2 * (n[A] * Paa + n[B] * Pab) + w * Pa));
		}

		void computeVolumeIntegrals(const ConvexHull3d& hull)
		{
			T0 = T1[0] = T1[1] = T1[2] = T2[0] = T2[1] = T2[2] = TP[0] = TP[1] = TP[2] = 0;

			for (int i = 0; i < hull.faces.size(); i++)
			{
				const HullFace3d& f = hull.faces[i];

				float nx = fabsf(f.norm.x);
				float ny = fabsf(f.norm.y);
				float nz = fabsf(f.norm.z);
				if (nx > ny && nx > nz) C = 0;
				else C = (ny > nz) ? 1 : 2;
				A = (C + 1) % 3;
				B = (A + 1) % 3;

				compFaceIntegrals(hull, f);

				T0 += f.norm.x * ((A == 0) ? Fa : ((B == 0) ? Fb : Fc));

				T1[A] += f.norm[A] * Faa;
				T1[B] += f.norm[B] * Fbb;
				T1[C] += f.norm[C] * Fcc;
				T2[A] += f.norm[A] * Faaa;
				T2[B] += f.norm[B] * Fbbb;
				T2[C] += f.norm[C] * Fccc;
				TP[A] += f.norm[A] * Faab;
				TP[B] += f.norm[B] * Fbbc;
				TP[C] += f.norm[C] * Fcca;
			}

			T1 *= 0.5f;
			T2 *= (1.0f / 3);
			TP *= 0.5f;
		}

	public:
		int A;   // alpha
		int B;   // beta
		int C;   // gamma
		float P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;
		float Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;
		float T0;
		Vector3 T1, T2, TP;
	};

	// See
	// Fast and Accurate Computation of Polyhedral Mass Properties, by Brian Mirtich
	// https://people.eecs.berkeley.edu/~jfc/mirtich/massProps.html
	// https://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
	Matrix3 ComputePolyhedralInertiaTensor_VolumeIntegration(const ConvexHull3d& hull, float density, float& Volume, float& Mass, Vector3& CenterOfMass)
	{
		PolyhedralMassProperties mp;
		mp.computeVolumeIntegrals(hull);

		if (fabsf(mp.T0) <= 1e-8f)
		{
			Volume = 0.0f;
			Mass = 0.0f;
			CenterOfMass = Vector3::Zero();
			return Matrix3::Identity();
		}

		const float OrientationSign = mp.T0 < 0.0f ? -1.0f : 1.0f;
		Volume = fabsf(mp.T0);
		Mass = density * Volume;
		CenterOfMass = mp.T1 / mp.T0;
		Matrix3 Inertia;

		const float SignedDensity = density * OrientationSign;
		Inertia[0][0] = SignedDensity * (mp.T2[1] + mp.T2[2]);
		Inertia[1][1] = SignedDensity * (mp.T2[2] + mp.T2[0]);
		Inertia[2][2] = SignedDensity * (mp.T2[0] + mp.T2[1]);
		Inertia[0][1] = Inertia[1][0] = -SignedDensity * mp.TP[0];
		Inertia[1][2] = Inertia[2][1] = -SignedDensity * mp.TP[1];
		Inertia[2][0] = Inertia[0][2] = -SignedDensity * mp.TP[2];

		/* translate inertia tensor to center of mass */
		Inertia[0][0] -= Mass * (CenterOfMass[1] * CenterOfMass[1] + CenterOfMass[2] * CenterOfMass[2]);
		Inertia[1][1] -= Mass * (CenterOfMass[2] * CenterOfMass[2] + CenterOfMass[0] * CenterOfMass[0]);
		Inertia[2][2] -= Mass * (CenterOfMass[0] * CenterOfMass[0] + CenterOfMass[1] * CenterOfMass[1]);
		Inertia[0][1] = Inertia[1][0] += Mass * CenterOfMass[0] * CenterOfMass[1];
		Inertia[1][2] = Inertia[2][1] += Mass * CenterOfMass[1] * CenterOfMass[2];
		Inertia[2][0] = Inertia[0][2] += Mass * CenterOfMass[2] * CenterOfMass[0];

		return Inertia;
	}
}
