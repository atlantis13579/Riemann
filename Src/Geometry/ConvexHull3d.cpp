
#include "ConvexHull3d.h"
#include "../Maths/Maths.h"

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
		Fcc = k3 * (Sqr(n[A]) * Paa + 2 * n[A] * n[B] * Pab + Sqr(n[B]) * Pbb
			+ w * (2 * (n[A] * Pa + n[B] * Pb) + w * P1));

		Faaa = k1 * Paaa;
		Fbbb = k1 * Pbbb;
		Fccc = -k4 * (Cube(n[A]) * Paaa + 3 * Sqr(n[A]) * n[B] * Paab
			+ 3 * n[A] * Sqr(n[B]) * Pabb + Cube(n[B]) * Pbbb
			+ 3 * w * (Sqr(n[A]) * Paa + 2 * n[A] * n[B] * Pab + Sqr(n[B]) * Pbb)
			+ w * w * (3 * (n[A] * Pa + n[B] * Pb) + w * P1));

		Faab = k1 * Paab;
		Fbbc = -k2 * (n[A] * Pabb + n[B] * Pbbb + w * Pbb);
		Fcca = k3 * (Sqr(n[A]) * Paaa + 2 * n[A] * n[B] * Paab + Sqr(n[B]) * Pabb
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
Matrix3 ComputePolyhedralInertiaTensor_VolumeIntegration(const ConvexHull3d& hull)
{
	PolyhedralMassProperties mp;
	mp.computeVolumeIntegrals(hull);

	float density = 1.0;  // assume unit density
	float mass = density * mp.T0;
	const Vector3 CenterOfMass = mp.T1 / mp.T0;
	Matrix3 Inertia;

	Inertia[0][0] = density * (mp.T2[1] + mp.T2[2]);
	Inertia[1][1] = density * (mp.T2[2] + mp.T2[0]);
	Inertia[2][2] = density * (mp.T2[0] + mp.T2[1]);
	Inertia[0][1] = Inertia[1][0] = -density * mp.TP[0];
	Inertia[1][2] = Inertia[2][1] = -density * mp.TP[1];
	Inertia[2][0] = Inertia[0][2] = -density * mp.TP[2];

	/* translate inertia tensor to center of mass */
	Inertia[0][0] -= mass * (CenterOfMass[1] * CenterOfMass[1] + CenterOfMass[2] * CenterOfMass[2]);
	Inertia[1][1] -= mass * (CenterOfMass[2] * CenterOfMass[2] + CenterOfMass[0] * CenterOfMass[0]);
	Inertia[2][2] -= mass * (CenterOfMass[0] * CenterOfMass[0] + CenterOfMass[1] * CenterOfMass[1]);
	Inertia[0][1] = Inertia[1][0] += mass * CenterOfMass[0] * CenterOfMass[1];
	Inertia[1][2] = Inertia[2][1] += mass * CenterOfMass[1] * CenterOfMass[2];
	Inertia[2][0] = Inertia[0][2] += mass * CenterOfMass[2] * CenterOfMass[0];

	return Inertia;
}

