#pragma once

#include "Box3.h"

namespace Maths
{
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

			m_Planes[RIGHT][0] = clip[3] - clip[0];
			m_Planes[RIGHT][1] = clip[7] - clip[4];
			m_Planes[RIGHT][2] = clip[11] - clip[8];
			m_Planes[RIGHT][3] = clip[15] - clip[12];

			NormalizePlane(m_Planes[RIGHT]);

			m_Planes[LEFT][0] = clip[3] + clip[0];
			m_Planes[LEFT][1] = clip[7] + clip[4];
			m_Planes[LEFT][2] = clip[11] + clip[8];
			m_Planes[LEFT][3] = clip[15] + clip[12];

			NormalizePlane(m_Planes[LEFT]);

			m_Planes[BOTTOM][0] = clip[3] + clip[1];
			m_Planes[BOTTOM][1] = clip[7] + clip[5];
			m_Planes[BOTTOM][2] = clip[11] + clip[9];
			m_Planes[BOTTOM][3] = clip[15] + clip[13];

			NormalizePlane(m_Planes[BOTTOM]);

			m_Planes[TOP][0] = clip[3] - clip[1];
			m_Planes[TOP][1] = clip[7] - clip[5];
			m_Planes[TOP][2] = clip[11] - clip[9];
			m_Planes[TOP][3] = clip[15] - clip[13];

			NormalizePlane(m_Planes[TOP]);

			m_Planes[BACK][0] = clip[3] - clip[2];
			m_Planes[BACK][1] = clip[7] - clip[6];
			m_Planes[BACK][2] = clip[11] - clip[10];
			m_Planes[BACK][3] = clip[15] - clip[14];

			NormalizePlane(m_Planes[BACK]);

			m_Planes[FRONT][0] = clip[3] + clip[2];
			m_Planes[FRONT][1] = clip[7] + clip[6];
			m_Planes[FRONT][2] = clip[11] + clip[10];
			m_Planes[FRONT][3] = clip[15] + clip[14];

			NormalizePlane(m_Planes[FRONT]);

			m_Points[0] = IntersectPlanes(m_Planes[LEFT], m_Planes[BOTTOM], m_Planes[FRONT]);
			m_Points[1] = IntersectPlanes(m_Planes[RIGHT], m_Planes[BOTTOM], m_Planes[FRONT]);
			m_Points[2] = IntersectPlanes(m_Planes[RIGHT], m_Planes[TOP], m_Planes[FRONT]);
			m_Points[3] = IntersectPlanes(m_Planes[LEFT], m_Planes[TOP], m_Planes[FRONT]);
			m_Points[4] = IntersectPlanes(m_Planes[LEFT], m_Planes[BOTTOM], m_Planes[BACK]);
			m_Points[5] = IntersectPlanes(m_Planes[RIGHT], m_Planes[BOTTOM], m_Planes[BACK]);
			m_Points[6] = IntersectPlanes(m_Planes[RIGHT], m_Planes[TOP], m_Planes[BACK]);
			m_Points[7] = IntersectPlanes(m_Planes[LEFT], m_Planes[TOP], m_Planes[BACK]);
		}

		bool FrustumIntersectAABB_Fast(const Vector3& mMin, const Vector3& mMax) const
		{
			for (int i = 0; i < 6; i++)
			{
				if (m_Planes[i][0] * mMin.x + m_Planes[i][1] * mMin.y + m_Planes[i][2] * mMin.z + m_Planes[i][3] > 0)
					continue;
				if (m_Planes[i][0] * mMax.x + m_Planes[i][1] * mMin.y + m_Planes[i][2] * mMin.z + m_Planes[i][3] > 0)
					continue;
				if (m_Planes[i][0] * mMin.x + m_Planes[i][1] * mMax.y + m_Planes[i][2] * mMin.z + m_Planes[i][3] > 0)
					continue;
				if (m_Planes[i][0] * mMax.x + m_Planes[i][1] * mMax.y + m_Planes[i][2] * mMin.z + m_Planes[i][3] > 0)
					continue;
				if (m_Planes[i][0] * mMin.x + m_Planes[i][1] * mMin.y + m_Planes[i][2] * mMax.z + m_Planes[i][3] > 0)
					continue;
				if (m_Planes[i][0] * mMax.x + m_Planes[i][1] * mMin.y + m_Planes[i][2] * mMax.z + m_Planes[i][3] > 0)
					continue;
				if (m_Planes[i][0] * mMin.x + m_Planes[i][1] * mMax.y + m_Planes[i][2] * mMax.z + m_Planes[i][3] > 0)
					continue;
				if (m_Planes[i][0] * mMax.x + m_Planes[i][1] * mMax.y + m_Planes[i][2] * mMax.z + m_Planes[i][3] > 0)
					continue;
				return false;
			}

			return true;
		}

		// https://iquilezles.org/articles/frustumcorrect/
		bool FrustumIntersectAABB(const Vector3& mMin, const Vector3& mMax) const
		{
			if (!FrustumIntersectAABB_Fast(mMin, mMax))
			{
				return false;
			}

			int out;
			out = 0; for (int i = 0; i < 8; i++) out += ((m_Points[i].x > mMax.x) ? 1 : 0); if (out == 8) return false;
			out = 0; for (int i = 0; i < 8; i++) out += ((m_Points[i].x < mMin.x) ? 1 : 0); if (out == 8) return false;
			out = 0; for (int i = 0; i < 8; i++) out += ((m_Points[i].y > mMax.y) ? 1 : 0); if (out == 8) return false;
			out = 0; for (int i = 0; i < 8; i++) out += ((m_Points[i].y < mMin.y) ? 1 : 0); if (out == 8) return false;
			out = 0; for (int i = 0; i < 8; i++) out += ((m_Points[i].z > mMax.z) ? 1 : 0); if (out == 8) return false;
			out = 0; for (int i = 0; i < 8; i++) out += ((m_Points[i].z < mMin.z) ? 1 : 0); if (out == 8) return false;

			return true;
		}

		bool PointInFrustum(const Vector3& vec) const
		{
			for (int i = 0; i < 6; i++)
			{
				if (m_Planes[i][0] * vec.x + m_Planes[i][1] * vec.y + m_Planes[i][2] * vec.z + m_Planes[i][3] <= 0)
				{
					return false;
				}
			}

			return true;
		}

	private:
		static void NormalizePlane(float plane[4])
		{
			const float magnitude = (float)sqrtf(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);

			plane[0] /= magnitude;
			plane[1] /= magnitude;
			plane[2] /= magnitude;
			plane[3] /= magnitude;
		}

		static Vector3 IntersectPlanes(const float plane1[4], const float plane2[4], const float plane3[4])
		{
			float det = plane1[0] * (plane2[1] * plane3[2] - plane2[2] * plane3[1]) -
						plane1[1] * (plane2[0] * plane3[2] - plane2[2] * plane3[0]) +
						plane1[2] * (plane2[0] * plane3[1] - plane2[1] * plane3[0]);

			// check det == 0

			float x = -(plane1[3] * (plane2[1] * plane3[2] - plane2[2] * plane3[1]) -
						plane2[3] * (plane1[1] * plane3[2] - plane1[2] * plane3[1]) +
						plane3[3] * (plane1[1] * plane2[2] - plane1[2] * plane2[1])) / det;

			float y = -(plane1[0] * (plane2[3] * plane3[2] - plane2[2] * plane3[3]) -
						plane2[0] * (plane1[3] * plane3[2] - plane1[2] * plane3[3]) +
						plane3[0] * (plane1[3] * plane2[2] - plane1[2] * plane2[3])) / det;

			float z = -(plane1[0] * (plane2[1] * plane3[3] - plane2[3] * plane3[1]) -
						plane2[0] * (plane1[1] * plane3[3] - plane1[3] * plane3[1]) +
						plane3[0] * (plane1[1] * plane2[3] - plane1[3] * plane2[1])) / det;

			return Vector3(x, y, z);
		}

		float	m_Planes[6][4];
		Vector3	m_Points[8];
	};
}

using Frustum = Maths::Frustum;
