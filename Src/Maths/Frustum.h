
#pragma once

#include "Box3d.h"

class Frustum
{
private:
	enum FrustumSide
	{
		RIGHT = 0,
		LEFT = 1,
		BOTTOM = 2,
		TOP = 3,
		BACK = 4,
		FRONT = 5
	};

public:
	void CalculateFrustum(float projMat[16], float modelView[16])
	{
		float   clip[16];

		clip[0] = modelView[0] * projMat[0] + modelView[1] * projMat[4] + modelView[2] * projMat[8] + modelView[3] * projMat[12];
		clip[1] = modelView[0] * projMat[1] + modelView[1] * projMat[5] + modelView[2] * projMat[9] + modelView[3] * projMat[13];
		clip[2] = modelView[0] * projMat[2] + modelView[1] * projMat[6] + modelView[2] * projMat[10] + modelView[3] * projMat[14];
		clip[3] = modelView[0] * projMat[3] + modelView[1] * projMat[7] + modelView[2] * projMat[11] + modelView[3] * projMat[15];

		clip[4] = modelView[4] * projMat[0] + modelView[5] * projMat[4] + modelView[6] * projMat[8] + modelView[7] * projMat[12];
		clip[5] = modelView[4] * projMat[1] + modelView[5] * projMat[5] + modelView[6] * projMat[9] + modelView[7] * projMat[13];
		clip[6] = modelView[4] * projMat[2] + modelView[5] * projMat[6] + modelView[6] * projMat[10] + modelView[7] * projMat[14];
		clip[7] = modelView[4] * projMat[3] + modelView[5] * projMat[7] + modelView[6] * projMat[11] + modelView[7] * projMat[15];

		clip[8] = modelView[8] * projMat[0] + modelView[9] * projMat[4] + modelView[10] * projMat[8] + modelView[11] * projMat[12];
		clip[9] = modelView[8] * projMat[1] + modelView[9] * projMat[5] + modelView[10] * projMat[9] + modelView[11] * projMat[13];
		clip[10] = modelView[8] * projMat[2] + modelView[9] * projMat[6] + modelView[10] * projMat[10] + modelView[11] * projMat[14];
		clip[11] = modelView[8] * projMat[3] + modelView[9] * projMat[7] + modelView[10] * projMat[11] + modelView[11] * projMat[15];

		clip[12] = modelView[12] * projMat[0] + modelView[13] * projMat[4] + modelView[14] * projMat[8] + modelView[15] * projMat[12];
		clip[13] = modelView[12] * projMat[1] + modelView[13] * projMat[5] + modelView[14] * projMat[9] + modelView[15] * projMat[13];
		clip[14] = modelView[12] * projMat[2] + modelView[13] * projMat[6] + modelView[14] * projMat[10] + modelView[15] * projMat[14];
		clip[15] = modelView[12] * projMat[3] + modelView[13] * projMat[7] + modelView[14] * projMat[11] + modelView[15] * projMat[15];

		m_Frustum[RIGHT][0] = clip[3] - clip[0];
		m_Frustum[RIGHT][1] = clip[7] - clip[4];
		m_Frustum[RIGHT][2] = clip[11] - clip[8];
		m_Frustum[RIGHT][3] = clip[15] - clip[12];

		NormalizePlane(m_Frustum, RIGHT);

		m_Frustum[LEFT][0] = clip[3] + clip[0];
		m_Frustum[LEFT][1] = clip[7] + clip[4];
		m_Frustum[LEFT][2] = clip[11] + clip[8];
		m_Frustum[LEFT][3] = clip[15] + clip[12];

		NormalizePlane(m_Frustum, LEFT);

		m_Frustum[BOTTOM][0] = clip[3] + clip[1];
		m_Frustum[BOTTOM][1] = clip[7] + clip[5];
		m_Frustum[BOTTOM][2] = clip[11] + clip[9];
		m_Frustum[BOTTOM][3] = clip[15] + clip[13];

		NormalizePlane(m_Frustum, BOTTOM);

		m_Frustum[TOP][0] = clip[3] - clip[1];
		m_Frustum[TOP][1] = clip[7] - clip[5];
		m_Frustum[TOP][2] = clip[11] - clip[9];
		m_Frustum[TOP][3] = clip[15] - clip[13];

		NormalizePlane(m_Frustum, TOP);

		m_Frustum[BACK][0] = clip[3] - clip[2];
		m_Frustum[BACK][1] = clip[7] - clip[6];
		m_Frustum[BACK][2] = clip[11] - clip[10];
		m_Frustum[BACK][3] = clip[15] - clip[14];

		NormalizePlane(m_Frustum, BACK);

		m_Frustum[FRONT][0] = clip[3] + clip[2];
		m_Frustum[FRONT][1] = clip[7] + clip[6];
		m_Frustum[FRONT][2] = clip[11] + clip[10];
		m_Frustum[FRONT][3] = clip[15] + clip[14];

		NormalizePlane(m_Frustum, FRONT);
	}

	bool FrustumIntersectAABB(const Box3d& box) const
	{
		for (int i = 0; i < 6; i++)
		{
			if (m_Frustum[i][0] * box.Min.x + m_Frustum[i][1] * box.Min.y + m_Frustum[i][2] * box.Min.z + m_Frustum[i][3] > 0)
				continue;
			if (m_Frustum[i][0] * box.Max.x + m_Frustum[i][1] * box.Min.y + m_Frustum[i][2] * box.Min.z + m_Frustum[i][3] > 0)
				continue;
			if (m_Frustum[i][0] * box.Min.x + m_Frustum[i][1] * box.Max.y + m_Frustum[i][2] * box.Min.z + m_Frustum[i][3] > 0)
				continue;
			if (m_Frustum[i][0] * box.Max.x + m_Frustum[i][1] * box.Max.y + m_Frustum[i][2] * box.Min.z + m_Frustum[i][3] > 0)
				continue;
			if (m_Frustum[i][0] * box.Min.x + m_Frustum[i][1] * box.Min.y + m_Frustum[i][2] * box.Max.z + m_Frustum[i][3] > 0)
				continue;
			if (m_Frustum[i][0] * box.Max.x + m_Frustum[i][1] * box.Min.y + m_Frustum[i][2] * box.Max.z + m_Frustum[i][3] > 0)
				continue;
			if (m_Frustum[i][0] * box.Min.x + m_Frustum[i][1] * box.Max.y + m_Frustum[i][2] * box.Max.z + m_Frustum[i][3] > 0)
				continue;
			if (m_Frustum[i][0] * box.Max.x + m_Frustum[i][1] * box.Max.y + m_Frustum[i][2] * box.Max.z + m_Frustum[i][3] > 0)
				continue;
			return false;
		}

		return true;
	}

	bool PointInFrustum(const Vector3d& vec) const
	{
		for (int i = 0; i < 6; i++)
		{
			if (m_Frustum[i][0] * vec.x + m_Frustum[i][1] * vec.y + m_Frustum[i][2] * vec.z + m_Frustum[i][3] <= 0)
			{
				return false;
			}
		}

		return true;
	}

private:
	static void NormalizePlane(float frustum[6][4], int side)
	{
		float magnitude = (float)sqrtf(frustum[side][0] * frustum[side][0] +
			frustum[side][1] * frustum[side][1] +
			frustum[side][2] * frustum[side][2]);

		frustum[side][0] /= magnitude;
		frustum[side][1] /= magnitude;
		frustum[side][2] /= magnitude;
		frustum[side][3] /= magnitude;
	}

	float m_Frustum[6][4];
};
