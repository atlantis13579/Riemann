#include "Polygon3d.h"
#include "../CollisionPrimitive/Triangle3d.h"

#define POLY_STACK_SIZE		32

float PolygonArea3D(const Vector3d* polygon, int nvert)
{
	if (nvert < 3)
		return 0.0f;

	float area = 0.0f;
	for (int i = 1; i < nvert - 1; ++i)
	{
		area += Triangle3d::TriangleArea3D(polygon[0], polygon[i], polygon[i+1]);
	}
	return area;
}

void ClipPolygonByPlane3D(const Vector3d* polygon, int n, const Vector3d& Origin, const Vector3d& Normal, Vector3d* clipped, int* nc)
{
	Vector3d e1 = polygon[n - 1];
	float prev_proj = (Origin - e1).Dot(Normal);
	bool prev_inside = prev_proj < 0.0f;

	*nc = 0;

	for (int j = 0; j < n; ++j)
	{
		const Vector3d& e2 = polygon[j];
		float curr_proj = (Origin - e2).Dot(Normal);
		bool cur_inside = curr_proj < 0.0f;

		if (cur_inside != prev_inside)
		{
			Vector3d e12 = e2 - e1;
			float d = e12.Dot(Normal);
			if (d != 0.0f)
			{
				clipped[(*nc)++] = e1 + (prev_proj / d) * e12;
			}
			else
			{
				cur_inside = prev_inside;
			}
		}

		if (cur_inside)
		{
			clipped[(*nc)++] = e2;
		}

		prev_proj = curr_proj;
		prev_inside = cur_inside;
		e1 = e2;
	}
}

void ClipPolygonByProjectPolygon3D(const Vector3d* polygon, int n1, const Vector3d* polyProj, int n2, const Vector3d& projNormal, Vector3d* clipped, int *nc)
{
	Vector3d tmp[2][POLY_STACK_SIZE];
	int ntmp[2] = { 0 };
	int idx = 0;

	*nc = 0;

	const Vector3d *src = nullptr;
	Vector3d *dst = nullptr;
	int *nsrc = nullptr, *ndst = nullptr;

	for (int i = 0; i < n2; ++i)
	{
		Vector3d clip_e1 = polyProj[i];
		Vector3d clip_e2 = polyProj[(i + 1) % n2];
		Vector3d clip_normal = projNormal.Cross(clip_e2 - clip_e1);

		if (i == 0)
		{
			src = polygon;
			nsrc = &n1;
		}
		else
		{
			src = tmp[idx];
			nsrc = &ntmp[idx];
		}

		idx = 1 - idx;

		if (i == n2 - 1)
		{
			dst = clipped;
			ndst = nc;
		}
		else
		{
			dst = tmp[idx];
			ndst = &ntmp[idx];
		}

		ClipPolygonByPlane3D(src, *nsrc, clip_e1, clip_normal, dst, ndst);

		if (*ndst < 3)
		{
			nc = 0;
			break;
		}
	}
}

void ClipPolygonByProjectSegment3D(const Vector3d* polygon, int n, const Vector3d& P0,
									const Vector3d& P1, const Vector3d& Normal, Vector3d* clipped, int *nc)
{
	Vector3d edge = P1 - P0;
	Vector3d edge_normal = Normal.Cross(edge);
	Vector3d polygon_normal = (polygon[2] - polygon[0]).Cross(polygon[1] - polygon[0]);
	float sqr_dist = polygon_normal.SquareLength();
	Vector3d v1 = P0 + polygon_normal.Dot(polygon[0] - P0) * polygon_normal / sqr_dist;
	Vector3d v2 = P1 + polygon_normal.Dot(polygon[0] - P1) * polygon_normal / sqr_dist;
	Vector3d v12 = v2 - v1;
	float v12_sqr_dist = v12.SquareLength();

	Vector3d e1 = polygon[n - 1];
	float prev_proj = (P0 - e1).Dot(edge_normal);
	bool prev_inside = prev_proj < 0.0f;

	*nc = 0;

	for (int j = 0; j < n; ++j)
	{
		Vector3d e2 = polygon[j];
		float curr_proj = (P0 - e2).Dot(edge_normal);
		bool cur_inside = curr_proj < 0.0f;

		if (cur_inside != prev_inside)
		{
			Vector3d e12 = e2 - e1;
			float d = e12.Dot(edge_normal);
			Vector3d clipped_point = e1 + (prev_proj / d) * e12;
			float projection = (clipped_point - v1).Dot(v12);
			if (projection < 0.0f)
				clipped[(*nc)++] = v1;
			else if (projection > v12_sqr_dist)
				clipped[(*nc)++] = v2;
			else
				clipped[(*nc)++] = clipped_point;
		}

		prev_proj = curr_proj;
		prev_inside = cur_inside;
		e1 = e2;
	}
}

void ClipPolygonByAABB3D(const Vector3d* polygon, int n, const Vector3d& Min, const Vector3d &Max, Vector3d* clipped, int *nc)
{
	Vector3d tmp[2][POLY_STACK_SIZE];
	int ntmp[2] = { 0 };
	int idx = 0;

	*nc = 0;

	const Vector3d* src = nullptr;
	Vector3d* dst = nullptr;
	int *nsrc = nullptr, *ndst = nullptr;

	for (int axis = 0; axis < 3; ++axis)
	{
		for (int side = 0; side < 2; ++side)
		{
			Vector3d origin = Vector3d::Zero(), normal = Vector3d::Zero();
			if (side == 0)
			{
				normal[axis] = 1.0f;
				origin[axis] = Min[axis];
			}
			else
			{
				normal[axis] = -1.0f;
				origin[axis] = Max[axis];
			}
		
			if (idx == 0)
			{
				src = polygon;
				nsrc = &n;
			}
			else
			{
				src = tmp[idx & 1];
				nsrc = &ntmp[idx & 1];
			}

			idx++;

			if (idx == 6)
			{
				dst = clipped;
				ndst = nc;
			}
			else
			{
				dst = tmp[idx & 1];
				ndst = &ntmp[idx & 1];
			}

			ClipPolygonByPlane3D(src, *nsrc, origin, normal, dst, ndst);
			if (*ndst < 3)
			{
				*nc = 0;
				return;
			}

			normal = -normal;
		}
	}

}

bool ClipPolygonAgainPolygon3D(const Vector3d* poly1, int n1, const Vector3d* poly2, int n2, const Vector3d& proj_normal,
								float maxDistSqr, Vector3d *clipped1, int *c1, Vector3d* clipped2, int *c2)
{

	if (c1) *c1 = 0;
	if (c2) *c2 = 0;

	if (n1 < 2 || n2 < 3)
		return false;

	Vector3d clipped_face[POLY_STACK_SIZE];
	int nc = 0;

	if (n1 >= 3)
	{
		ClipPolygonByProjectPolygon3D(poly2, n2, poly1, n1, proj_normal, clipped_face, &nc);
	}
	else if (n1 == 2)
	{
		ClipPolygonByProjectSegment3D(poly2, n2, poly1[0], poly1[1], proj_normal, clipped_face, &nc);
	}

	Vector3d plane_origin = poly1[0];
	Vector3d plane_normal;
	Vector3d e = poly1[1] - plane_origin;
	if (n1 >= 3)
	{
		plane_normal = e.Cross(poly1[2] - plane_origin);
	}
	else
	{
		plane_normal = e.Cross(proj_normal).Cross(e);
	}

	bool succ = false;
	const float sqr_dist = plane_normal.SquareLength();
	if (sqr_dist > 0.0f)
	{
		for (int i = 0; i < nc; ++i)
		{
			const Vector3d& p2 = clipped_face[i];
			float distance = (p2 - plane_origin).Dot(plane_normal);
			if (distance <= 0.0f || distance * distance < maxDistSqr * sqr_dist)
			{
				Vector3d p1 = p2 - (distance / sqr_dist) * plane_normal;
				if (clipped1 && c1)
					clipped1[(*c1)++] = p1;
				if (clipped2 && c2)
					clipped2[(*c2)++] = p2;
				succ = true;
			}
		}
	}
	
	return succ;
}
