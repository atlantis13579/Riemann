#include "Polygon3d.h"
#include "../CollisionPrimitive/Triangle3d.h"

#define MAX_POINTS	8		// TODO

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

void ClipPolyVsPlane(const Vector3d* inPolygonToClip, int n1, const Vector3d& inPlaneOrigin, Vector3d& inPlaneNormal, Vector3d* outClippedPolygon, int& nc)
{
	// Determine state of last point
	Vector3d e1 = inPolygonToClip[n1 - 1];
	float prev_num = (inPlaneOrigin - e1).Dot(inPlaneNormal);
	bool prev_inside = prev_num < 0.0f;

	// Loop through all vertices
	for (int j = 0; j < n1; ++j)
	{
		// Check if second point is inside
		const Vector3d& e2 = inPolygonToClip[j];
		float num = (inPlaneOrigin - e2).Dot(inPlaneNormal);
		bool cur_inside = num < 0.0f;

		// In -> Out or Out -> In: Add point on clipping plane
		if (cur_inside != prev_inside)
		{
			// Solve: (X - inPlaneOrigin) . inPlaneNormal = 0 and X = e1 + t * (e2 - e1) for X
			Vector3d e12 = e2 - e1;
			float denom = e12.Dot(inPlaneNormal);
			if (denom != 0.0f)
			{
				outClippedPolygon[nc++] = e1 + (prev_num / denom) * e12;
			}
			else
			{
				cur_inside = prev_inside; // Edge is parallel to plane, treat point as if it were on the same side as the last point
			}
		}

		// Point inside, add it
		if (cur_inside)
		{
			outClippedPolygon[nc++] = e2;
		}

		// Update previous state
		prev_num = num;
		prev_inside = cur_inside;
		e1 = e2;
	}
}

/// Clip polygon versus polygon.
/// Both polygons are assumed to be in counter clockwise order.
/// @param inClippingPolygonNormal is used to create planes of all edges in inClippingPolygon against which inPolygonToClip is clipped, inClippingPolygonNormal does not need to be normalized
/// @param inClippingPolygon is the polygon which inClippedPolygon is clipped against
/// @param inPolygonToClip is the polygon that is clipped
/// @param outClippedPolygon will contain clipped polygon when function returns
void ClipPolyVsPoly(const Vector3d* inPolygonToClip, int n1, const Vector3d* inClippingPolygon, int n2, const Vector3d& inClippingPolygonNormal, Vector3d* outClippedPolygon, int &nc)
{
	Vector3d tmp_vertices[2][MAX_POINTS];
	int ntmp[2] = { 0 };

	int tmp_vertices_idx = 0;

	for (int i = 0; i < n2; ++i)
	{
		// Get edge to clip against
		Vector3d clip_e1 = inClippingPolygon[i];
		Vector3d clip_e2 = inClippingPolygon[(i + 1) % n2];
		Vector3d clip_normal = inClippingPolygonNormal.Cross(clip_e2 - clip_e1); // Pointing inward to the clipping polygon

		// Get source and target polygon
		const Vector3d* src_polygon = nullptr;
		int *nsrc = nullptr;
		if (i == 0)
		{
			src_polygon = inPolygonToClip;
			nsrc = &n1;
		}
		else
		{
			src_polygon = tmp_vertices[tmp_vertices_idx];
			nsrc = &ntmp[tmp_vertices_idx];
		}

		tmp_vertices_idx ^= 1;

		Vector3d* tgt_polygon = nullptr;
		int *ntgt = nullptr;
		if (i == n2 - 1)
		{
			tgt_polygon = outClippedPolygon;
			ntgt = &nc;
		}
		else
		{
			tgt_polygon = tmp_vertices[tmp_vertices_idx];
			ntgt = &ntmp[tmp_vertices_idx];
		}
		*ntgt = 0;

		// Clip against the edge
		ClipPolyVsPlane(src_polygon, *nsrc, clip_e1, clip_normal, tgt_polygon, *ntgt);

		// Break out if no polygon left
		if (*ntgt < 3)
		{
			nc = 0;
			break;
		}
	}
}

/// Clip inPolygonToClip against an edge, the edge is projected on inPolygonToClip using inClippingEdgeNormal.
/// The positive half space (the side on the edge in the direction of inClippingEdgeNormal) is cut away.
void ClipPolyVsEdge(const Vector3d* inPolygonToClip, int n1, const Vector3d& inEdgeVertex1, const Vector3d& inEdgeVertex2, const Vector3d& inClippingEdgeNormal, Vector3d* outClippedPolygon, int &nc)
{
	// Get normal that is perpendicular to the edge and the clipping edge normal
	Vector3d edge = inEdgeVertex2 - inEdgeVertex1;
	Vector3d edge_normal = inClippingEdgeNormal.Cross(edge);

	// Project vertices of edge on inPolygonToClip
	Vector3d polygon_normal = (inPolygonToClip[2] - inPolygonToClip[0]).Cross(inPolygonToClip[1] - inPolygonToClip[0]);
	float polygon_normal_len_sq = polygon_normal.SquareLength();
	Vector3d v1 = inEdgeVertex1 + polygon_normal.Dot(inPolygonToClip[0] - inEdgeVertex1) * polygon_normal / polygon_normal_len_sq;
	Vector3d v2 = inEdgeVertex2 + polygon_normal.Dot(inPolygonToClip[0] - inEdgeVertex2) * polygon_normal / polygon_normal_len_sq;
	Vector3d v12 = v2 - v1;
	float v12_len_sq = v12.SquareLength();

	// Determine state of last point
	Vector3d e1 = inPolygonToClip[n1 - 1];
	float prev_num = (inEdgeVertex1 - e1).Dot(edge_normal);
	bool prev_inside = prev_num < 0.0f;

	// Loop through all vertices
	for (int j = 0; j < n1; ++j)
	{
		// Check if second point is inside
		Vector3d e2 = inPolygonToClip[j];
		float num = (inEdgeVertex1 - e2).Dot(edge_normal);
		bool cur_inside = num < 0.0f;

		// In -> Out or Out -> In: Add point on clipping plane
		if (cur_inside != prev_inside)
		{
			// Solve: (X - inPlaneOrigin) . inPlaneNormal = 0 and X = e1 + t * (e2 - e1) for X
			Vector3d e12 = e2 - e1;
			float denom = e12.Dot(edge_normal);
			Vector3d clipped_point = e1 + (prev_num / denom) * e12;

			// Project point on line segment v1, v2 so see if it falls outside if the edge
			float projection = (clipped_point - v1).Dot(v12);
			if (projection < 0.0f)
				outClippedPolygon[nc++] = v1;
			else if (projection > v12_len_sq)
				outClippedPolygon[nc++] = v2;
			else
				outClippedPolygon[nc++] = clipped_point;
		}

		// Update previous state
		prev_num = num;
		prev_inside = cur_inside;
		e1 = e2;
	}
}

/// Clip polygon vs axis aligned box, inPolygonToClip is assume to be in counter clockwise order.
/// Output will be stored in outClippedPolygon. Everything inside inAABox will be kept.
void ClipPolyVsAABox(const Vector3d* inPolygonToClip, int n1, const Vector3d& Min, const Vector3d &Max, Vector3d* outClippedPolygon, int &nc)
{
	Vector3d tmp_vertices[2][MAX_POINTS];
	int ntmp[2] = { 0 };

	int tmp_vertices_idx = 0;

	for (int coord = 0; coord < 3; ++coord)
	for (int side = 0; side < 2; ++side)
	{
		// Get plane to clip against
		Vector3d origin = Vector3d::Zero(), normal = Vector3d::Zero();
		if (side == 0)
		{
			normal[coord] = 1.0f;
			origin[coord] = Min[coord];
		}
		else
		{
			normal[coord] = -1.0f;
			origin[coord] = Max[coord];
		}
		
		// Get source and target polygon
		const Vector3d* src_polygon = nullptr;
		int* nsrc = nullptr;
		if (tmp_vertices_idx == 0)
		{
			src_polygon = inPolygonToClip;
			nsrc = &n1;
		}
		else
		{
			src_polygon = tmp_vertices[tmp_vertices_idx & 1];
			nsrc = &ntmp[tmp_vertices_idx & 1];
		}
		tmp_vertices_idx++;

		Vector3d* tgt_polygon = nullptr;
		int* ntgt = nullptr;
		if (tmp_vertices_idx == 6)
		{
			tgt_polygon = outClippedPolygon;
			ntgt = &nc;
		}
		else
		{
			tgt_polygon = tmp_vertices[tmp_vertices_idx & 1];
			ntgt = &ntmp[tmp_vertices_idx & 1];
		}
		*ntgt = 0;

		// Clip against the edge
		ClipPolyVsPlane(src_polygon, *nsrc, origin, normal, tgt_polygon, *ntgt);
		
		// Break out if no polygon left
		if (*ntgt < 3)
		{
			nc = 0;
			return;
		}

		// Flip normal
		normal = -normal;
	}
}

bool ClipPolygon3D(const Vector3d* inShape1Face, int n1, const Vector3d* inShape2Face, int n2, const Vector3d& inPenetrationAxis,
		float inSpeculativeContactDistanceSq, Vector3d * outContactPoints1, int &c1, Vector3d* outContactPoints2, int &c2)
{
	if (n1 < 2 || n2 < 3)
		return false;

	Vector3d clipped_face[MAX_POINTS];
	int nc = 0;

	// Clip the polygon of face 2 against that of 1
	if (n1 >= 3)
		ClipPolyVsPoly(inShape2Face, n2, inShape1Face, n1, inPenetrationAxis, clipped_face, nc);
	else if (n1 == 2)
		ClipPolyVsEdge(inShape2Face, n2, inShape1Face[0], inShape1Face[1], inPenetrationAxis, clipped_face, nc);

	// Project the points back onto the plane of shape 1 face and only keep those that are behind the plane
	Vector3d plane_origin = inShape1Face[0];
	Vector3d plane_normal;
	Vector3d first_edge = inShape1Face[1] - plane_origin;
	if (n1 >= 3)
	{
		// Three vertices, can just calculate the normal
		plane_normal = first_edge.Cross(inShape1Face[2] - plane_origin);
	}
	else
	{
		// Two vertices, first find a perpendicular to the edge and penetration axis and then use the perpendicular together with the edge to form a normal
		plane_normal = first_edge.Cross(inPenetrationAxis).Cross(first_edge);
	}

	// Check if the plane normal has any length, if not the clipped shape is so small that we'll just use the contact points
	float plane_normal_len_sq = plane_normal.SquareLength();
	if (plane_normal_len_sq > 0.0f)
	{
		// Discard points of faces that are too far away to collide
		for (int i = 0; i < nc; ++i)
		{
			const Vector3d& p2 = clipped_face[i];
			float distance = (p2 - plane_origin).Dot(plane_normal); // Note should divide by length of plane_normal (unnormalized here)
			if (distance <= 0.0f || distance * distance < inSpeculativeContactDistanceSq * plane_normal_len_sq) // Must be close enough to plane, note we correct for not dividing by plane normal length here
			{
				// Project point back on shape 1 using the normal, note we correct for not dividing by plane normal length here:
				// p1 = p2 - (distance / sqrt(plane_normal_len_sq)) * (plane_normal / sqrt(plane_normal_len_sq));
				Vector3d p1 = p2 - (distance / plane_normal_len_sq) * plane_normal;

				outContactPoints1[c1++] = p1;
				outContactPoints2[c2++] = p2;
			}
		}
	}
	
	return true;
}
