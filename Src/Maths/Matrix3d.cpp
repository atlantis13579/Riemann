
#include "Matrix3d.h"
#include "Maths.h"

#include <stdint.h>

void Matrix3d::LoadRotateX(float rfloatAngle)
{
	float fCos, fSin;
	fCos = cosf(rfloatAngle);
	fSin = sinf(rfloatAngle);
	mat[0][0] = 1.0f;	mat[0][1] = 0.0f;	mat[0][2] = 0.0f;
	mat[1][0] = 1.0f;	mat[1][1] = fCos;	mat[1][2] = fSin;
	mat[2][0] = 1.0f;	mat[2][1] = -fSin; mat[2][2] = fCos;
}

void Matrix3d::LoadRotateY(float rfloatAngle)
{
	float fCos, fSin;
	fCos = cosf(rfloatAngle);
	fSin = sinf(rfloatAngle);
	mat[0][0] = fCos;	mat[0][1] = 0.0f;	mat[0][2] = -fSin;
	mat[1][0] = 0.0f;	mat[1][1] = 1.0f;	mat[1][2] = 0.0f;
	mat[2][0] = fSin;	mat[2][1] = 0.0f;	mat[2][2] = fCos;
}

void Matrix3d::LoadRotateZ(float rfloatAngle)
{
	float fCos, fSin;
	fCos = cosf(rfloatAngle);
	fSin = sinf(rfloatAngle);
	mat[0][0] = fCos;	mat[0][1] = fSin; mat[0][2] = 0.0f;
	mat[1][0] = -fSin; mat[1][1] = fCos;	mat[1][2] = 0.0f;
	mat[2][0] = 0.0f;	mat[2][1] = 0.0f;	mat[2][2] = 1.0f;
}

void Matrix3d::Load2DOrthogonalTransform(float dx, float dy, float dAngle) {
	float fCos, fSin;
	fCos = cosf(dAngle);
	fSin = sinf(dAngle);
	mat[0][0] = fCos;	mat[0][1] = -fSin; mat[0][2] = dx;
	mat[1][0] = fSin; mat[1][1] = fCos;	mat[1][2] = dy;
	mat[2][0] = 0.0f;	mat[2][1] = 0.0f;	mat[2][2] = 1.0f;
}

void Matrix3d::Bidiagonalize(Matrix3d& rA, Matrix3d& rL, Matrix3d& rR)
{
	float afV[3], afW[3];
	float fLength, fSign, fT1, fInvT1, fT2;
	bool bIdentity;

	// map first column to (*,0,0)
	fLength = sqrtf(rA[0][0] * rA[0][0] + rA[1][0] * rA[1][0] + rA[2][0] * rA[2][0]);
	if (fLength > 0.0)
	{
		fSign = (rA[0][0] > 0.0f ? 1.0f : -1.0f);
		fT1 = rA[0][0] + fSign * fLength;
		fInvT1 = 1.0f / fT1;
		afV[1] = rA[1][0] * fInvT1;
		afV[2] = rA[2][0] * fInvT1;

		fT2 = -2.0f / (1.0f + afV[1] * afV[1] + afV[2] * afV[2]);
		afW[0] = fT2 * (rA[0][0] + rA[1][0] * afV[1] + rA[2][0] * afV[2]);
		afW[1] = fT2 * (rA[0][1] + rA[1][1] * afV[1] + rA[2][1] * afV[2]);
		afW[2] = fT2 * (rA[0][2] + rA[1][2] * afV[1] + rA[2][2] * afV[2]);
		rA[0][0] += afW[0];
		rA[0][1] += afW[1];
		rA[0][2] += afW[2];
		rA[1][1] += afV[1] * afW[1];
		rA[1][2] += afV[1] * afW[2];
		rA[2][1] += afV[2] * afW[1];
		rA[2][2] += afV[2] * afW[2];

		rL[0][0] = 1.0f + fT2;
		rL[0][1] = rL[1][0] = fT2 * afV[1];
		rL[0][2] = rL[2][0] = fT2 * afV[2];
		rL[1][1] = 1.0f + fT2 * afV[1] * afV[1];
		rL[1][2] = rL[2][1] = fT2 * afV[1] * afV[2];
		rL[2][2] = 1.0f + fT2 * afV[2] * afV[2];
		bIdentity = false;
	}
	else
	{
		rL = Matrix3d::Identity();
		bIdentity = true;
	}

	// map first row to (*,*,0)
	fLength = sqrtf(rA[0][1] * rA[0][1] + rA[0][2] * rA[0][2]);
	if (fLength > 0.0f)
	{
		fSign = (rA[0][1] > 0.0f ? 1.0f : -1.0f);
		fT1 = rA[0][1] + fSign * fLength;
		afV[2] = rA[0][2] / fT1;

		fT2 = -2.0f / (1.0f + afV[2] * afV[2]);
		afW[0] = fT2 * (rA[0][1] + rA[0][2] * afV[2]);
		afW[1] = fT2 * (rA[1][1] + rA[1][2] * afV[2]);
		afW[2] = fT2 * (rA[2][1] + rA[2][2] * afV[2]);
		rA[0][1] += afW[0];
		rA[1][1] += afW[1];
		rA[1][2] += afW[1] * afV[2];
		rA[2][1] += afW[2];
		rA[2][2] += afW[2] * afV[2];

		rR[0][0] = 1.0f;
		rR[0][1] = rR[1][0] = 0.0f;
		rR[0][2] = rR[2][0] = 0.0f;
		rR[1][1] = 1.0f + fT2;
		rR[1][2] = rR[2][1] = fT2 * afV[2];
		rR[2][2] = 1.0f + fT2 * afV[2] * afV[2];
	}
	else
	{
		rR = Matrix3d::Identity();
	}

	// map second column to (*,*,0)
	fLength = sqrtf(rA[1][1] * rA[1][1] + rA[2][1] * rA[2][1]);
	if (fLength > 0.0f)
	{
		fSign = (rA[1][1] > 0.0f ? 1.0f : -1.0f);
		fT1 = rA[1][1] + fSign * fLength;
		afV[2] = rA[2][1] / fT1;

		fT2 = -2.0f / (1.0f + afV[2] * afV[2]);
		afW[1] = fT2 * (rA[1][1] + rA[2][1] * afV[2]);
		afW[2] = fT2 * (rA[1][2] + rA[2][2] * afV[2]);
		rA[1][1] += afW[1];
		rA[1][2] += afW[2];
		rA[2][2] += afV[2] * afW[2];

		float fA = 1.0f + fT2;
		float fB = fT2 * afV[2];
		float fC = 1.0f + fB * afV[2];

		if (bIdentity)
		{
			rL[0][0] = 1.0f;
			rL[0][1] = rL[1][0] = 0.0;
			rL[0][2] = rL[2][0] = 0.0;
			rL[1][1] = fA;
			rL[1][2] = rL[2][1] = fB;
			rL[2][2] = fC;
		}
		else
		{
			for (int iRow = 0; iRow < 3; ++iRow)
			{
				float fTmp0 = rL[iRow][1];
				float fTmp1 = rL[iRow][2];
				rL[iRow][1] = fA * fTmp0 + fB * fTmp1;
				rL[iRow][2] = fB * fTmp0 + fC * fTmp1;
			}
		}
	}
}

void Matrix3d::GolubKahanStep(Matrix3d& rA, Matrix3d& rL, Matrix3d& rR)
{
	float fT11 = rA[0][1] * rA[0][1] + rA[1][1] * rA[1][1];
	float fT22 = rA[1][2] * rA[1][2] + rA[2][2] * rA[2][2];
	float fT12 = rA[1][1] * rA[1][2];
	float fTrace = fT11 + fT22;
	float fDiff = fT11 - fT22;
	float fDiscr = sqrtf(fDiff * fDiff + 4.0f * fT12 * fT12);
	float fRoot1 = 0.5f * (fTrace + fDiscr);
	float fRoot2 = 0.5f * (fTrace - fDiscr);

	// adjust right
	float fY = rA[0][0] - (fabsf(fRoot1 - fT22) <=
		fabsf(fRoot2 - fT22) ? fRoot1 : fRoot2);
	float fZ = rA[0][1];
	float fInvLength = InvSqrt(fY * fY + fZ * fZ);
	float fSin = fZ * fInvLength;
	float fCos = -fY * fInvLength;

	float fTmp0 = rA[0][0];
	float fTmp1 = rA[0][1];
	rA[0][0] = fCos * fTmp0 - fSin * fTmp1;
	rA[0][1] = fSin * fTmp0 + fCos * fTmp1;
	rA[1][0] = -fSin * rA[1][1];
	rA[1][1] *= fCos;

	int iRow;
	for (iRow = 0; iRow < 3; ++iRow)
	{
		fTmp0 = rR[0][iRow];
		fTmp1 = rR[1][iRow];
		rR[0][iRow] = fCos * fTmp0 - fSin * fTmp1;
		rR[1][iRow] = fSin * fTmp0 + fCos * fTmp1;
	}

	// adjust left
	fY = rA[0][0];
	fZ = rA[1][0];
	fInvLength = InvSqrt(fY * fY + fZ * fZ);
	fSin = fZ * fInvLength;
	fCos = -fY * fInvLength;

	rA[0][0] = fCos * rA[0][0] - fSin * rA[1][0];
	fTmp0 = rA[0][1];
	fTmp1 = rA[1][1];
	rA[0][1] = fCos * fTmp0 - fSin * fTmp1;
	rA[1][1] = fSin * fTmp0 + fCos * fTmp1;
	rA[0][2] = -fSin * rA[1][2];
	rA[1][2] *= fCos;

	int iCol;
	for (iCol = 0; iCol < 3; ++iCol)
	{
		fTmp0 = rL[iCol][0];
		fTmp1 = rL[iCol][1];
		rL[iCol][0] = fCos * fTmp0 - fSin * fTmp1;
		rL[iCol][1] = fSin * fTmp0 + fCos * fTmp1;
	}

	// adjust right
	fY = rA[0][1];
	fZ = rA[0][2];
	fInvLength = InvSqrt(fY * fY + fZ * fZ);
	fSin = fZ * fInvLength;
	fCos = -fY * fInvLength;

	rA[0][1] = fCos * rA[0][1] - fSin * rA[0][2];
	fTmp0 = rA[1][1];
	fTmp1 = rA[1][2];
	rA[1][1] = fCos * fTmp0 - fSin * fTmp1;
	rA[1][2] = fSin * fTmp0 + fCos * fTmp1;
	rA[2][1] = -fSin * rA[2][2];
	rA[2][2] *= fCos;

	for (iRow = 0; iRow < 3; ++iRow)
	{
		fTmp0 = rR[1][iRow];
		fTmp1 = rR[2][iRow];
		rR[1][iRow] = fCos * fTmp0 - fSin * fTmp1;
		rR[2][iRow] = fSin * fTmp0 + fCos * fTmp1;
	}

	// adjust left
	fY = rA[1][1];
	fZ = rA[2][1];
	fInvLength = InvSqrt(fY * fY + fZ * fZ);
	fSin = fZ * fInvLength;
	fCos = -fY * fInvLength;

	rA[1][1] = fCos * rA[1][1] - fSin * rA[2][1];
	fTmp0 = rA[1][2];
	fTmp1 = rA[2][2];
	rA[1][2] = fCos * fTmp0 - fSin * fTmp1;
	rA[2][2] = fSin * fTmp0 + fCos * fTmp1;

	for (iCol = 0; iCol < 3; ++iCol)
	{
		fTmp0 = rL[iCol][1];
		fTmp1 = rL[iCol][2];
		rL[iCol][1] = fCos * fTmp0 - fSin * fTmp1;
		rL[iCol][2] = fSin * fTmp0 + fCos * fTmp1;
	}
}

void Matrix3d::SingularValueDecomposition(Matrix3d& rL, Vector3d& rS, Matrix3d& rR) const
{
	// temas: currently unused
	//const int iMax = 16;
	int iRow, iCol;
	const float fSvdEpsilon = 1e-04f;
	const int   iSvdMaxIterations = 32;

	Matrix3d kA = *this;
	Bidiagonalize(kA, rL, rR);

	for (uint32_t i = 0; i < iSvdMaxIterations; i++)
	{
		float fTmp, fTmp0, fTmp1;
		float fSin0, fCos0, fTan0;
		float fSin1, fCos1, fTan1;

		bool bTest1 = (fabsf(kA[0][1]) <=
			fSvdEpsilon * (fabsf(kA[0][0]) + fabsf(kA[1][1])));
		bool bTest2 = (fabsf(kA[1][2]) <=
			fSvdEpsilon * (fabsf(kA[1][1]) + fabsf(kA[2][2])));
		if (bTest1)
		{
			if (bTest2)
			{
				rS[0] = kA[0][0];
				rS[1] = kA[1][1];
				rS[2] = kA[2][2];
				break;
			}
			else
			{
				// 2x2 closed form factorization
				fTmp = (kA[1][1] * kA[1][1] - kA[2][2] * kA[2][2] +
					kA[1][2] * kA[1][2]) / (kA[1][2] * kA[2][2]);
				fTan0 = 0.5f * (fTmp + sqrtf(fTmp * fTmp + 4.0f));
				fCos0 = InvSqrt(1.0f + fTan0 * fTan0);
				fSin0 = fTan0 * fCos0;

				for (iCol = 0; iCol < 3; ++iCol)
				{
					fTmp0 = rL[iCol][1];
					fTmp1 = rL[iCol][2];
					rL[iCol][1] = fCos0 * fTmp0 - fSin0 * fTmp1;
					rL[iCol][2] = fSin0 * fTmp0 + fCos0 * fTmp1;
				}

				fTan1 = (kA[1][2] - kA[2][2] * fTan0) / kA[1][1];
				fCos1 = InvSqrt(1.0f + fTan1 * fTan1);
				fSin1 = -fTan1 * fCos1;

				for (iRow = 0; iRow < 3; ++iRow)
				{
					fTmp0 = rR[1][iRow];
					fTmp1 = rR[2][iRow];
					rR[1][iRow] = fCos1 * fTmp0 - fSin1 * fTmp1;
					rR[2][iRow] = fSin1 * fTmp0 + fCos1 * fTmp1;
				}

				rS[0] = kA[0][0];
				rS[1] = fCos0 * fCos1 * kA[1][1] -
					fSin1 * (fCos0 * kA[1][2] - fSin0 * kA[2][2]);
				rS[2] = fSin0 * fSin1 * kA[1][1] +
					fCos1 * (fSin0 * kA[1][2] + fCos0 * kA[2][2]);
				break;
			}
		}
		else
		{
			if (bTest2)
			{
				// 2x2 closed form factorization
				fTmp = (kA[0][0] * kA[0][0] + kA[1][1] * kA[1][1] -
					kA[0][1] * kA[0][1]) / (kA[0][1] * kA[1][1]);
				fTan0 = 0.5f * (-fTmp + sqrtf(fTmp * fTmp + 4.0f));
				fCos0 = InvSqrt(1.0f + fTan0 * fTan0);
				fSin0 = fTan0 * fCos0;

				for (iCol = 0; iCol < 3; ++iCol)
				{
					fTmp0 = rL[iCol][0];
					fTmp1 = rL[iCol][1];
					rL[iCol][0] = fCos0 * fTmp0 - fSin0 * fTmp1;
					rL[iCol][1] = fSin0 * fTmp0 + fCos0 * fTmp1;
				}

				fTan1 = (kA[0][1] - kA[1][1] * fTan0) / kA[0][0];
				fCos1 = InvSqrt(1.0f + fTan1 * fTan1);
				fSin1 = -fTan1 * fCos1;

				for (iRow = 0; iRow < 3; ++iRow)
				{
					fTmp0 = rR[0][iRow];
					fTmp1 = rR[1][iRow];
					rR[0][iRow] = fCos1 * fTmp0 - fSin1 * fTmp1;
					rR[1][iRow] = fSin1 * fTmp0 + fCos1 * fTmp1;
				}

				rS[0] = fCos0 * fCos1 * kA[0][0] -
					fSin1 * (fCos0 * kA[0][1] - fSin0 * kA[1][1]);
				rS[1] = fSin0 * fSin1 * kA[0][0] +
					fCos1 * (fSin0 * kA[0][1] + fCos0 * kA[1][1]);
				rS[2] = kA[2][2];
				break;
			}
			else
			{
				GolubKahanStep(kA, rL, rR);
			}
		}
	}

	// positize diagonal
	for (iRow = 0; iRow < 3; ++iRow)
	{
		if (rS[iRow] < 0.0)
		{
			rS[iRow] = -rS[iRow];
			for (iCol = 0; iCol < 3; ++iCol)
				rR[iRow][iCol] = -rR[iRow][iCol];
		}
	}
}

void Matrix3d::SingularValueComposition(const Matrix3d& rL,	const Vector3d& rS, const Matrix3d& rR)
{
	int iRow, iCol;
	Matrix3d kTmp;

	// product S*R
	for (iRow = 0; iRow < 3; ++iRow)
	{
		for (iCol = 0; iCol < 3; ++iCol)
			kTmp[iRow][iCol] = rS[iRow] * rR[iRow][iCol];
	}

	// product L*S*R
	for (iRow = 0; iRow < 3; ++iRow)
	{
		for (iCol = 0; iCol < 3; ++iCol)
		{
			mat[iRow][iCol] = 0.0;
			for (int iMid = 0; iMid < 3; iMid++)
				mat[iRow][iCol] += rL[iRow][iMid] * kTmp[iMid][iCol];
		}
	}
}

void Matrix3d::Orthonormalize()
{
	// Algorithm uses Gram-Schmidt orthogonalization.  If 'this' matrix is
	//	   [ m0 ]		[ q0 ]
	// M = | m1 | ==>	| q1 |
	//	   [ m2 ]		[ q2 ]
	//	 
	//   q0 = m0/|m0|
	//   q1 = (m1-(q0*m1)q0)/|m1-(q0*m1)q0|
	//   q2 = (m2-(q0*m2)q0-(q1*m2)q1)/|m2-(q0*m2)q0-(q1*m2)q1|
	//
	// where |V| indicates length of vector V and A*B indicates dot
	// product of vectors A and B.

	// compute q0
	float fInvLength = InvSqrt(mat[0][0] * mat[0][0]
		+ mat[0][1] * mat[0][1] + mat[0][2] * mat[0][2]);

	mat[0][0] *= fInvLength;
	mat[0][1] *= fInvLength;
	mat[0][2] *= fInvLength;

	// compute q1
	float fDot0 =
		mat[0][0] * mat[1][0] +
		mat[0][1] * mat[1][1] +
		mat[0][2] * mat[1][2];

	mat[1][0] -= fDot0 * mat[0][0];
	mat[1][1] -= fDot0 * mat[0][1];
	mat[1][2] -= fDot0 * mat[0][2];

	fInvLength = InvSqrt(mat[1][0] * mat[1][0] +
		mat[1][1] * mat[1][1] + mat[1][2] * mat[1][2]);

	mat[0][1] *= fInvLength;
	mat[1][1] *= fInvLength;
	mat[2][1] *= fInvLength;

	// q2 = (m2-(q0*m2)q0-(q1*m2)q1)/|m2-(q0*m2)q0-(q1*m2)q1|
	float fDot1 =
		mat[1][0] * mat[2][0] +
		mat[1][1] * mat[2][1] +
		mat[1][2] * mat[2][2];

	fDot0 =
		mat[0][0] * mat[2][0] +
		mat[0][1] * mat[2][1] +
		mat[0][2] * mat[2][2];

	mat[2][0] -= fDot0 * mat[0][0] + fDot1 * mat[1][0];
	mat[2][1] -= fDot0 * mat[0][1] + fDot1 * mat[1][1];
	mat[2][2] -= fDot0 * mat[0][2] + fDot1 * mat[1][2];

	fInvLength = InvSqrt(mat[2][0] * mat[2][0] +
		mat[2][1] * mat[2][1] + mat[2][2] * mat[2][2]);

	mat[2][0] *= fInvLength;
	mat[2][1] *= fInvLength;
	mat[2][2] *= fInvLength;
}

void Matrix3d::PolarDecomposition(Matrix3d& rU, Matrix3d& rP) const
{
	Matrix3d L, R;
	Vector3d S;
	SingularValueDecomposition(L, S, R);
	rP = R.Transpose() * Matrix3d(S.x, S.y, S.z) * R;
	rU = L * R;
}

void Matrix3d::QDUDecomposition(Matrix3d& rQ, Vector3d& rD, Vector3d& rU) const
{
	// Factor M = QR = QDU where Q is orthogonal, D is diagonal,
	// and U is upper triangular with ones on its diagonal.  Algorithm uses
	// Gram-Schmidt orthogonalization (the QR algorithm).
	//
	// If M = [ m0 | m1 | m2 ] and Q = [ q0 | q1 | q2 ], then
	//
	//   q0 = m0/|m0|
	//   q1 = (m1-(q0*m1)q0)/|m1-(q0*m1)q0|
	//   q2 = (m2-(q0*m2)q0-(q1*m2)q1)/|m2-(q0*m2)q0-(q1*m2)q1|
	//
	// where |V| indicates length of vector V and A*B indicates dot
	// product of vectors A and B.  The matrix R has entries
	//
	//   r00 = q0*m0  r01 = q0*m1  r02 = q0*m2
	//   r10 = 0      r11 = q1*m1  r12 = q1*m2
	//   r20 = 0      r21 = 0      r22 = q2*m2
	//
	// so D = diag(r00,r11,r22) and U has entries u01 = r01/r00,
	// u02 = r02/r00, and u12 = r12/r11.

	// Q = rotation
	// D = scaling
	// U = shear

	// D stores the three diagonal entries r00, r11, r22
	// U stores the entries U[0] = u01, U[1] = u02, U[2] = u12

	// build orthogonal matrix Q
	float fInvLength = InvSqrt(mat[0][0] * mat[0][0]
		+ mat[1][0] * mat[1][0] +
		mat[2][0] * mat[2][0]);
	rQ[0][0] = mat[0][0] * fInvLength;
	rQ[1][0] = mat[1][0] * fInvLength;
	rQ[2][0] = mat[2][0] * fInvLength;

	float fDot = rQ[0][0] * mat[0][1] + rQ[1][0] * mat[1][1] +
		rQ[2][0] * mat[2][1];
	rQ[0][1] = mat[0][1] - fDot * rQ[0][0];
	rQ[1][1] = mat[1][1] - fDot * rQ[1][0];
	rQ[2][1] = mat[2][1] - fDot * rQ[2][0];
	fInvLength = InvSqrt(rQ[0][1] * rQ[0][1] + rQ[1][1] * rQ[1][1] +
		rQ[2][1] * rQ[2][1]);
	rQ[0][1] *= fInvLength;
	rQ[1][1] *= fInvLength;
	rQ[2][1] *= fInvLength;

	fDot = rQ[0][0] * mat[0][2] + rQ[1][0] * mat[1][2] +
		rQ[2][0] * mat[2][2];
	rQ[0][2] = mat[0][2] - fDot * rQ[0][0];
	rQ[1][2] = mat[1][2] - fDot * rQ[1][0];
	rQ[2][2] = mat[2][2] - fDot * rQ[2][0];
	fDot = rQ[0][1] * mat[0][2] + rQ[1][1] * mat[1][2] +
		rQ[2][1] * mat[2][2];
	rQ[0][2] -= fDot * rQ[0][1];
	rQ[1][2] -= fDot * rQ[1][1];
	rQ[2][2] -= fDot * rQ[2][1];
	fInvLength = InvSqrt(rQ[0][2] * rQ[0][2] + rQ[1][2] * rQ[1][2] +
		rQ[2][2] * rQ[2][2]);
	rQ[0][2] *= fInvLength;
	rQ[1][2] *= fInvLength;
	rQ[2][2] *= fInvLength;

	// guarantee that orthogonal matrix has determinant 1 (no reflections)
	float fDet = rQ[0][0] * rQ[1][1] * rQ[2][2] + rQ[0][1] * rQ[1][2] * rQ[2][0] +
		rQ[0][2] * rQ[1][0] * rQ[2][1] - rQ[0][2] * rQ[1][1] * rQ[2][0] -
		rQ[0][1] * rQ[1][0] * rQ[2][2] - rQ[0][0] * rQ[1][2] * rQ[2][1];

	if (fDet < 0.0)
	{
		for (int iRow = 0; iRow < 3; ++iRow)
			for (int iCol = 0; iCol < 3; ++iCol)
				rQ[iRow][iCol] = -rQ[iRow][iCol];
	}

	// build "right" matrix R
	Matrix3d kR;
	kR[0][0] = rQ[0][0] * mat[0][0] + rQ[1][0] * mat[1][0] +
		rQ[2][0] * mat[2][0];
	kR[0][1] = rQ[0][0] * mat[0][1] + rQ[1][0] * mat[1][1] +
		rQ[2][0] * mat[2][1];
	kR[1][1] = rQ[0][1] * mat[0][1] + rQ[1][1] * mat[1][1] +
		rQ[2][1] * mat[2][1];
	kR[0][2] = rQ[0][0] * mat[0][2] + rQ[1][0] * mat[1][2] +
		rQ[2][0] * mat[2][2];
	kR[1][2] = rQ[0][1] * mat[0][2] + rQ[1][1] * mat[1][2] +
		rQ[2][1] * mat[2][2];
	kR[2][2] = rQ[0][2] * mat[0][2] + rQ[1][2] * mat[1][2] +
		rQ[2][2] * mat[2][2];

	// the scaling component
	rD[0] = kR[0][0];
	rD[1] = kR[1][1];
	rD[2] = kR[2][2];

	// the shear component
	float fInvD0 = 1.0f / rD[0];
	rU[0] = kR[0][1] * fInvD0;
	rU[1] = kR[0][2] * fInvD0;
	rU[2] = kR[1][2] / rD[1];
}
//-----------------------------------------------------------------------
float Matrix3d::MaxCubicRoot(float afCoeff[3])
{
	// Spectral norm is for A^T*A, so characteristic polynomial
	// P(x) = c[0]+c[1]*x+c[2]*x^2+x^3 has three positive float roots.
	// This yields the assertions c[0] < 0 and c[2]*c[2] >= 3*c[1].

	// quick out for uniform scale (triple root)
	const float fOneThird = 1.0f / 3.0f;
	const float fEpsilon = 1e-06f;
	float fDiscr = afCoeff[2] * afCoeff[2] - 3.0f * afCoeff[1];
	if (fDiscr <= fEpsilon)
		return -fOneThird * afCoeff[2];

	// Compute an upper bound on roots of P(x).  This assumes that A^T*A
	// has been scaled by its largest entry.
	float fX = 1.0;
	float fPoly = afCoeff[0] + fX * (afCoeff[1] + fX * (afCoeff[2] + fX));
	if (fPoly < 0.0)
	{
		// uses a matrix norm to find an upper bound on maximum root
		fX = fabsf(afCoeff[0]);
		float fTmp = 1.0f + fabsf(afCoeff[1]);
		if (fTmp > fX)
			fX = fTmp;
		fTmp = 1.0f + fabsf(afCoeff[2]);
		if (fTmp > fX)
			fX = fTmp;
	}

	// Newton's method to find root
	float fTwoC2 = 2.0f * afCoeff[2];
	for (int i = 0; i < 16; ++i)
	{
		fPoly = afCoeff[0] + fX * (afCoeff[1] + fX * (afCoeff[2] + fX));
		if (fabsf(fPoly) <= fEpsilon)
			return fX;

		float fDeriv = afCoeff[1] + fX * (fTwoC2 + 3.0f * fX);
		fX -= fPoly / fDeriv;
	}

	return fX;
}
//-----------------------------------------------------------------------
float Matrix3d::SpectralNorm() const
{
	Matrix3d kP;
	int iRow, iCol;
	float fPmax = 0.0;
	for (iRow = 0; iRow < 3; ++iRow)
	{
		for (iCol = 0; iCol < 3; ++iCol)
		{
			kP[iRow][iCol] = 0.0;
			for (int iMid = 0; iMid < 3; iMid++)
			{
				kP[iRow][iCol] +=
					mat[iMid][iRow] * mat[iMid][iCol];
			}
			if (kP[iRow][iCol] > fPmax)
				fPmax = kP[iRow][iCol];
		}
	}

	float fInvPmax = 1.0f / fPmax;
	for (iRow = 0; iRow < 3; ++iRow)
	{
		for (iCol = 0; iCol < 3; ++iCol)
			kP[iRow][iCol] *= fInvPmax;
	}

	float afCoeff[3];
	afCoeff[0] = -(kP[0][0] * (kP[1][1] * kP[2][2] - kP[1][2] * kP[2][1]) +
		kP[0][1] * (kP[2][0] * kP[1][2] - kP[1][0] * kP[2][2]) +
		kP[0][2] * (kP[1][0] * kP[2][1] - kP[2][0] * kP[1][1]));
	afCoeff[1] = kP[0][0] * kP[1][1] - kP[0][1] * kP[1][0] +
		kP[0][0] * kP[2][2] - kP[0][2] * kP[2][0] +
		kP[1][1] * kP[2][2] - kP[1][2] * kP[2][1];
	afCoeff[2] = -(kP[0][0] + kP[1][1] + kP[2][2]);

	float fRoot = MaxCubicRoot(afCoeff);
	float fNorm = sqrtf(fPmax * fRoot);
	return fNorm;
}

float Matrix3d::ToAxisAngle(Vector3d& Axis) const
{
	// Let (x,y,z) be the unit-length axis and let A be an angle of rotation.
	// The rotation matrix is R = I + sin(A)*P + (1-cos(A))*P^2 where
	// I is the identity and
	//
	//       +-        -+
	//   P = |  0 -z +y |
	//       | +z  0 -x |
	//       | -y +x  0 |
	//       +-        -+
	//
	// If A > 0, R represents a counterclockwise rotation about the axis in
	// the sense of looking from the tip of the axis vector towards the
	// origin.  Some algebra will show that
	//
	//   cos(A) = (trace(R)-1)/2  and  R - R^t = 2*sin(A)*P
	//
	// In the event that A = pi, R-R^t = 0 which prevents us from extracting
	// the axis through P.  Instead note that R = I+2*P^2 when A = pi, so
	// P^2 = (R-I)/2.  The diagonal entries of P^2 are x^2-1, y^2-1, and
	// z^2-1.  We can solve these for axis (x,y,z).  Because the angle is pi,
	// it does not matter which sign you choose on the square roots.

	float fTrace = mat[0][0] + mat[1][1] + mat[2][2];
	float fCos = 0.5f * (fTrace - 1.0f);
	float Angle = acosf(fCos);  // in [0,PI]

	if (Angle > 0.0f)
	{
		if (Angle < PI)
		{
			Axis.x = mat[2][1] - mat[1][2];
			Axis.y = mat[0][2] - mat[2][0];
			Axis.z = mat[1][0] - mat[0][1];
			Axis.Normalize();
		}
		else
		{
			// angle is PI
			float fHalfInverse;
			if (mat[0][0] >= mat[1][1])
			{
				// r00 >= r11
				if (mat[0][0] >= mat[2][2])
				{
					// r00 is maximum diagonal term
					Axis.x = 0.5f * sqrtf(mat[0][0] -
						mat[1][1] - mat[2][2] + 1.0f);
					fHalfInverse = 0.5f / Axis.x;
					Axis.y = fHalfInverse * mat[0][1];
					Axis.z = fHalfInverse * mat[0][2];
				}
				else
				{
					// r22 is maximum diagonal term
					Axis.z = 0.5f * sqrtf(mat[2][2] -
						mat[0][0] - mat[1][1] + 1.0f);
					fHalfInverse = 0.5f / Axis.z;
					Axis.x = fHalfInverse * mat[0][2];
					Axis.y = fHalfInverse * mat[1][2];
				}
			}
			else
			{
				// r11 > r00
				if (mat[1][1] >= mat[2][2])
				{
					// r11 is maximum diagonal term
					Axis.y = 0.5f * sqrtf(mat[1][1] -
						mat[0][0] - mat[2][2] + 1.0f);
					fHalfInverse = 0.5f / Axis.y;
					Axis.x = fHalfInverse * mat[0][1];
					Axis.z = fHalfInverse * mat[1][2];
				}
				else
				{
					// r22 is maximum diagonal term
					Axis.z = 0.5f * sqrtf(mat[2][2] -
						mat[0][0] - mat[1][1] + 1.0f);
					fHalfInverse = 0.5f / Axis.z;
					Axis.x = fHalfInverse * mat[0][2];
					Axis.y = fHalfInverse * mat[1][2];
				}
			}
		}
	}
	else
	{
		// The angle is 0 and the matrix is the identity.  Any axis will
		// work, so just use the x-axis.
		Axis.x = 1.0f;
		Axis.y = 0.0f;
		Axis.z = 0.0f;
	}

	return Angle;
}

bool Matrix3d::ToEulerAnglesXYZ(float& Yaw, float& Pitch, float& Roll) const
{
	// rot =  cy*cz          -cy*sz           sy
	//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
	//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy

	Pitch = asinf(mat[0][2]);
	if (Pitch < PI_OVER_2)
	{
		if (Pitch > -PI_OVER_2)
		{
			Yaw = atan2f(-mat[1][2], mat[2][2]);
			Roll = atan2f(-mat[0][1], mat[0][0]);
			return true;
		}
		else
		{
			// WARNING.  Not a unique solution.
			float fRmY = atan2f(mat[1][0], mat[1][1]);
			Roll = float(0.0);  // any angle works
			Yaw = Roll - fRmY;
			return false;
		}
	}
	else
	{
		// WARNING.  Not a unique solution.
		float fRpY = atan2f(mat[1][0], mat[1][1]);
		Roll = float(0.0);  // any angle works
		Yaw = fRpY - Roll;
		return false;
	}
}

void Matrix3d::FromEulerAnglesXYZ(float Yaw, float Pitch,	float Roll)
{
	float fCos, fSin;

	fCos = cosf(Yaw);
	fSin = sinf(Yaw);
	Matrix3d kXMat(1.0f, 0.0f, 0.0f,
		0.0f, fCos, fSin,
		0.0f, -fSin, fCos);

	fCos = cosf(Pitch);
	fSin = sinf(Pitch);
	Matrix3d kYMat(fCos, 0.0f, -fSin,
		0.0f, 1.0f, 0.0f,
		fSin, 0.0f, fCos);

	fCos = cosf(Roll);
	fSin = sinf(Roll);
	Matrix3d kZMat(fCos, fSin, 0.0f,
		-fSin, fCos, 0.0f,
		0.0f, 0.0f, 1.0f);

	*this = kXMat * kYMat * kZMat;
}

void Matrix3d::TriDiagonal(float Diag[3], float SubDiag[3])
{
	// Householder reduction T = Q^t M Q
	//   Input:
	//     mat, symmetric 3x3 matrix M
	//   Output:
	//     mat, orthogonal matrix Q
	//     diag, diagonal entries of T
	//     subd, subdiagonal entries of T (T is symmetric)

	float fA = mat[0][0];
	float fB = mat[0][1];
	float fC = mat[0][2];
	float fD = mat[1][1];
	float fE = mat[1][2];
	float fF = mat[2][2];

	Diag[0] = fA;
	SubDiag[2] = 0.0;
	if (fabsf(fC) >= Epsilon(1.0f))
	{
		float fLength = sqrtf(fB * fB + fC * fC);
		float fInvLength = 1.0f / fLength;
		fB *= fInvLength;
		fC *= fInvLength;
		float fQ = 2.0f * fB * fE + fC * (fF - fD);
		Diag[1] = fD + fC * fQ;
		Diag[2] = fF - fC * fQ;
		SubDiag[0] = fLength;
		SubDiag[1] = fE - fB * fQ;
		mat[0][0] = 1.0f;
		mat[0][1] = 0.0f;
		mat[0][2] = 0.0f;
		mat[1][0] = 0.0f;
		mat[1][1] = fB;
		mat[1][2] = fC;
		mat[2][0] = 0.0f;
		mat[2][1] = fC;
		mat[2][2] = -fB;
	}
	else
	{
		Diag[1] = fD;
		Diag[2] = fF;
		SubDiag[0] = fB;
		SubDiag[1] = fE;
		mat[0][0] = 1.0f;
		mat[0][1] = 0.0f;
		mat[0][2] = 0.0f;
		mat[1][0] = 0.0f;
		mat[1][1] = 1.0f;
		mat[1][2] = 0.0f;
		mat[2][0] = 0.0f;
		mat[2][1] = 0.0f;
		mat[2][2] = 1.0f;
	}
}

bool Matrix3d::QRIteration(float Diag[3], float SubDiag[3])
{
	for (int i0 = 0; i0 < 3; i0++)
	{
		const int iMaxIter = 32;
		int it = 0;
		while (it++ < iMaxIter)
		{
			int i1;
			for (i1 = i0; i1 <= 1; ++i1)
			{
				float sum = fabsf(Diag[i1]) +
					fabsf(Diag[i1 + 1]);
				if (fabsf(SubDiag[i1]) + sum == sum)
					break;
			}
			if (i1 == i0)
				break;

			float t0 = (Diag[i0 + 1] - Diag[i0]) / (2.0f * SubDiag[i0]);
			float t1 = sqrtf(t0 * t0 + 1.0f);
			if (t0 < 0.0)
				t0 = Diag[i1] - Diag[i0] + SubDiag[i0] / (t0 - t1);
			else
				t0 = Diag[i1] - Diag[i0] + SubDiag[i0] / (t0 + t1);
			float fSin = 1.0;
			float fCos = 1.0;
			float t2 = 0.0;
			for (int i2 = i1 - 1; i2 >= i0; --i2)
			{
				float fTmp3 = fSin * SubDiag[i2];
				float fTmp4 = fCos * SubDiag[i2];
				if (fabsf(fTmp3) >= fabsf(t0))
				{
					fCos = t0 / fTmp3;
					t1 = sqrtf(fCos * fCos + 1.0f);
					SubDiag[i2 + 1] = fTmp3 * t1;
					fSin = 1.0f / t1;
					fCos *= fSin;
				}
				else
				{
					fSin = fTmp3 / t0;
					t1 = sqrtf(fSin * fSin + 1.0f);
					SubDiag[i2 + 1] = t0 * t1;
					fCos = 1.0f / t1;
					fSin *= fCos;
				}
				t0 = Diag[i2 + 1] - t2;
				t1 = (Diag[i2] - t0) * fSin + 2.0f * fTmp4 * fCos;
				t2 = fSin * t1;
				Diag[i2 + 1] = t0 + t2;
				t0 = fCos * t1 - fTmp4;

				for (int iRow = 0; iRow < 3; ++iRow)
				{
					fTmp3 = mat[iRow][i2 + 1];
					mat[iRow][i2 + 1] = fSin * mat[iRow][i2] +
						fCos * fTmp3;
					mat[iRow][i2] = fCos * mat[iRow][i2] -
						fSin * fTmp3;
				}
			}
			Diag[i0] -= t2;
			SubDiag[i0] = t0;
			SubDiag[i1] = 0.0;
		}

		if (it == iMaxIter)
		{
			// should not get here
			return false;
		}
	}

	return true;
}

void Matrix3d::SolveEigenSymmetric(float EigenValue[3], Vector3d EigenVector[3]) const
{
	Matrix3d kMatrix = *this;
	float SubDiag[3];
	kMatrix.TriDiagonal(EigenValue, SubDiag);
	kMatrix.QRIteration(EigenValue, SubDiag);

	for (int i = 0; i < 3; ++i)
	{
		EigenVector[i][0] = kMatrix[0][i];
		EigenVector[i][1] = kMatrix[1][i];
		EigenVector[i][2] = kMatrix[2][i];
	}

	Vector3d kCross = EigenVector[1].Cross(EigenVector[2]);
	float Det = EigenVector[0].Dot(kCross);
	if (Det < 0.0)
	{
		EigenVector[2][0] = -EigenVector[2][0];
		EigenVector[2][1] = -EigenVector[2][1];
		EigenVector[2][2] = -EigenVector[2][2];
	}
}