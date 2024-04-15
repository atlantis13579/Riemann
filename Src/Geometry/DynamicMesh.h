#pragma once

#include "assert.h"
#include <string>
#include <map>
#include <vector>
#include <functional>
#include "../Core/ListSet.h"
#include "../Maths/Box3.h"
#include "../Maths/Vector2.h"
#include "../Maths/Vector3.h"
#include "../Maths/Index2.h"
#include "../Maths/Index3.h"
#include "../Maths/Transform.h"

namespace Riemann
{
	class DynamicMeshAABBTree;

	enum class EMeshResult
	{
		Ok = 0,
		Failed_NotAVertex = 1,
		Failed_NotATriangle = 2,
		Failed_NotAnEdge = 3,

		Failed_BrokenTopology = 10,
		Failed_HitValenceLimit = 11,

		Failed_IsBoundaryEdge = 20,
		Failed_FlippedEdgeExists = 21,
		Failed_IsBowtieVertex = 22,
		Failed_InvalidNeighbourhood = 23,       // these are all failures for CollapseEdge
		Failed_FoundDuplicateTriangle = 24,
		Failed_CollapseTetrahedron = 25,
		Failed_CollapseTriangle = 26,
		Failed_NotABoundaryEdge = 27,
		Failed_SameOrientation = 28,

		Failed_WouldCreateBowtie = 30,
		Failed_VertexAlreadyExists = 31,
		Failed_CannotAllocateVertex = 32,
		Failed_VertexStillReferenced = 33,

		Failed_WouldCreateNonmanifoldEdge = 50,
		Failed_TriangleAlreadyExists = 51,
		Failed_CannotAllocateTriangle = 52,

		Failed_UnrecoverableError = 1000,
		Failed_Unsupported = 1001
	};

	#define MAX_UV_CHANNEL	(8)
	struct VertexInfo
	{
		Vector3 Position { Vector3::Zero() };
		Vector3 Normal { Vector3::Zero() };
		Vector3 Color { Vector3::Zero() };
		Vector2 UVs[MAX_UV_CHANNEL];
		bool bHasNormal { false };
		bool bHasColor { false };
		int NumUVs { 0 };
	};

	struct EdgeSplitInfo
	{
		int OriginalEdge;
		Index2 OriginalVertices;
		Index2 OtherVertices;
		Index2 OriginalTriangles;
		bool bIsBoundary;
		int NewVertex;
		Index2 NewTriangles;
		Index3 NewEdges;
		float SplitT;
	};

	struct PokeTriangleInfo
	{
		int OriginalTriangle;
		Index3 TriVertices;
		int NewVertex;
		Index2 NewTriangles;
		Index3 NewEdges;
		Vector3 BaryCoords;
	};

	struct EdgeCollapseInfo
	{
		int KeptVertex;
		int RemovedVertex;
		Index2 OpposingVerts;
		bool bIsBoundary;
		int CollapsedEdge;
		Index2 RemovedTris;
		Index2 RemovedEdges;
		Index2 KeptEdges;
		float CollapseT;
	};

	template<typename T>
	class AttributeBase
	{
	public:
		inline void SetNewValue(int index, const T& val)
		{
			mValues.insert(mValues.begin() + index, val);
		}

		inline T GetValue(int index) const
		{
			return mValues[index];
		}

		inline void SetValue(int index, const T& val)
		{
			mValues[index] = val;
		}

		void clear()
		{
			mValues.clear();
		}

		size_t size() const
		{
			return mValues.size();
		}

	private:
		std::vector<T>	mValues;
	};

	struct GeometryAttributeSet
	{
		void clear()
		{
			MaterialIDAttrib.clear();
		}

		void OnPokeTriangle(const PokeTriangleInfo& PokeResult)
		{
			assert(false);
		}

		void OnSplitEdge(const EdgeSplitInfo& info)
		{
			assert(false);
		}

		void OnRemoveTriangle(int tid)
		{
			assert(false);
		}

		void OnCollapseEdge(const EdgeCollapseInfo& info)
		{
			assert(false);
		}

		AttributeBase<int> MaterialIDAttrib;
	};

	struct IntersectionsQueryResult
	{
		struct PointIntersection
		{
			int TriangleID[2];
			Vector3 Point;
		};

		struct SegmentIntersection
		{
			int TriangleID[2];
			Vector3 Point[2];
		};

		struct PolygonIntersection
		{
			int TriangleID[2];
			Vector3 Point[6];
			int Quantity;
		};

		std::vector<PointIntersection> Points;
		std::vector<SegmentIntersection> Segments;
		std::vector<PolygonIntersection> Polygons;
	};

	class DynamicMesh
	{
	public:
		DynamicMesh();
		~DynamicMesh();

		void Clear()
		{
			VertexPositions.clear();
			VertexNormals.clear();
			VertexColors.clear();
			VertexUVs.clear();
			Triangles.clear();
			TriangleEdges.clear();
			Edges.clear();
			VertexRefCounts.clear();
			EdgeRefCounts.clear();
			TriangleRefCounts.clear();
			VertexEdgeLists.clear();

			bHasVertexColor = false;
			bHasVertexNormals = false;

			Attributes.clear();
			ResourceName = "";
		}

		bool LoadObj(const char* name);
		bool ExportObj(const char* name);
		static DynamicMesh* CreateFromObj(const char* name);

		struct Edge
		{
			int v0;
			int v1;
			int t0;
			int t1;
		};

		int GetVertexCount() const { return (int)VertexPositions.size(); }
		int GetTriangleCount() const { return (int)Triangles.size(); }
		int GetEdgeCount() const { return (int)Edges.size(); }
		Vector3* GetVertexBuffer() { return VertexPositions.data(); }
		Index3* GetIndexBuffer() { return Triangles.data(); }

		int AppendVertex(const Vector3& v)
		{
			int index = (int)VertexPositions.size();
			VertexPositions.push_back(v);
			VertexRefCounts.push_back(1);
			VertexEdgeLists.append();
			return index;
		}

		int AppendVertex(const VertexInfo& info)
		{
			size_t index = VertexPositions.size();

			VertexPositions.push_back(info.Position);
			VertexRefCounts.push_back(1);
			VertexEdgeLists.append();

			if (info.bHasColor)
			{
				bHasVertexColor = true;
			}

			if (info.bHasNormal)
			{
				bHasVertexNormals = true;
			}

			if (info.NumUVs > 0)
			{
				if (info.NumUVs > (int)VertexUVs.size())
					VertexUVs.resize(info.NumUVs);
			}

			if (HasVertexColors())
			{
				Vector3 val = info.bHasColor ? info.Color : Vector3::Zero();
				VectorSetSafe(VertexColors, index, val);
			}

			if (HasVertexNormals())
			{
				Vector3 val = info.bHasNormal ? info.Normal : Vector3::UnitY();
				VectorSetSafe(VertexNormals, index, val);
			}

			for (size_t i = 0; i < VertexUVs.size(); ++i)
			{
				Vector2 val = i < info.NumUVs ? info.UVs[i] : Vector2::Zero();
				VectorSetSafe(VertexUVs[i], index, val);
			}

			return (int)index;
		}

		inline bool HasVertexColors() const { return bHasVertexColor; }
		inline bool HasVertexNormals() const { return bHasVertexNormals; }
		inline bool HasVertexUVs() const { return VertexUVs.size() > 0; }
		inline bool HasAttributes() const { return Attributes.MaterialIDAttrib.size() > 0; }

		inline bool IsVertex(int idx) const
		{
			return 0 <= idx && idx < (int)VertexPositions.size() && VertexRefCounts[idx] > 0;
		}

		inline bool IsBoundaryVertex(int vID) const
		{
			assert(IsVertex(vID));
			if (IsVertex(vID))
			{
				for (int eid : VertexEdgeLists[vID])
				{
					if (Edges[eid].t1 == -1)
					{
						return true;
					}
				}
			}
			return false;
		}


		inline Vector3 GetVertex(int idx) const
		{
			return VertexPositions[idx];
		}

		void SetVertex(int idx, const Vector3& val)
		{
			VectorSetSafe(VertexPositions, idx, val);
		}

		Vector3 GetVertexColor(int idx) const
		{
			if (!HasVertexColors())
			{
				return Vector3::Zero();
			}
			return VertexColors[idx];
		}

		void SetVertexColor(int idx, const Vector3& val)
		{
			bHasVertexColor = true;
			VectorSetSafe(VertexColors, idx, val);
		}

		Vector3 GetVertexNormal(int idx) const
		{
			if (!HasVertexNormals())
			{
				return Vector3::UnitY();
			}
			return VertexNormals[idx];
		}

		void SetVertexNormal(int idx, const Vector3& val)
		{
			bHasVertexNormals = true;
			VectorSetSafe(VertexNormals, idx, val);
		}

		Vector2 GetVertexUV(int idx, int layer) const
		{
			if (!HasVertexUVs())
			{
				return Vector2::Zero();
			}
			return VertexUVs[layer][idx];
		}

		void SetVertexUV(int idx, const Vector2& val, int channel)
		{
			if (channel < 0)
			{
				for (size_t i = 0; i < VertexUVs.size(); ++i)
				{
					VectorSetSafe(VertexUVs[i], idx, val);
				}
				return;
			}

			if (channel >= (int)VertexUVs.size())
			{
				VertexUVs.resize(channel);
			}

			VectorSetSafe(VertexUVs[channel], idx, val);
		}

		inline Index3 GetTriangle(int idx) const { return Triangles[idx]; }

		inline bool IsTriangle(int idx) const
		{
			return 0 <= idx && idx < (int)Triangles.size() && TriangleRefCounts[idx] > 0;
		}

		int AppendTriangle(const Index3& tri)
		{
			int index = (int)Triangles.size();

			Index3 tri_edge;

			for (int j = 0; j < 3; ++j)
			{
				int v0 = tri[j];
				int v1 = tri[(j + 1) % 3];
				if (v0 > v1)
				{
					std::swap(v0, v1);
				}

				int edge_idx = FindEdge(v0, v1);
				if (edge_idx == -1)
				{
					edge_idx = AddEdgeEx(v0, v1, (int)index, -1);
				}
				else
				{
					assert(Edges[edge_idx].t1 == -1);
					if (Edges[edge_idx].t1 == -1)
					{
						Edges[edge_idx].t1 = (int)index;
					}
				}

				tri_edge[j] = edge_idx;
			}

			AddTriangleEx(tri.a, tri.b, tri.c, tri_edge.a, tri_edge.b, tri_edge.c);
			return index;
		}

		EMeshResult RemoveTriangle(int tID, bool bRemoveIsolatedVertices, bool bPreserveManifold);


		EMeshResult SetTriangle(int tID, const Index3& newv)
		{
			// if (HasAttributes() == false)
			//{
			//	assert(false);
			//	return EMeshResult::Failed_Unsupported;
			//}

			const  bool bRemoveIsolatedVertices = true;
			Index3 tv = GetTriangle(tID);
			Index3 te = GetTriangleEdge(tID);
			if (tv[0] == newv[0] && tv[1] == newv[1])
			{
				te[0] = -1;
			}
			if (tv[1] == newv[1] && tv[2] == newv[2])
			{
				te[1] = -1;
			}
			if (tv[2] == newv[2] && tv[0] == newv[0])
			{
				te[2] = -1;
			}

			if (TriangleRefCounts[tID] <= 0)
			{
				assert(false);
				return EMeshResult::Failed_NotATriangle;
			}
			if (IsVertex(newv[0]) == false || IsVertex(newv[1]) == false || IsVertex(newv[2]) == false)
			{
				assert(false);
				return EMeshResult::Failed_NotAVertex;
			}
			if (newv[0] == newv[1] || newv[0] == newv[2] || newv[1] == newv[2])
			{
				assert(false);
				return EMeshResult::Failed_BrokenTopology;
			}
			// look up edges. if any already have two triangles, this would
			// create non-manifold geometry and so we do not allow it
			int e0 = FindEdge(newv[0], newv[1]);
			int e1 = FindEdge(newv[1], newv[2]);
			int e2 = FindEdge(newv[2], newv[0]);
			if ((te[0] != -1 && e0 != -1 && IsBoundaryEdge(e0) == false)
				|| (te[1] != -1 && e1 != -1 && IsBoundaryEdge(e1) == false)
				|| (te[2] != -1 && e2 != -1 && IsBoundaryEdge(e2) == false))
			{
				return EMeshResult::Failed_BrokenTopology;
			}

			for (int j = 0; j < 3; ++j)
			{
				int eid = te[j];
				if (eid == -1)
				{
					continue;
				}
				ReplaceEdgeTriangle(eid, tID, -1);
				const Edge& e = GetEdge(eid);
				if (e.t0 == -1)
				{
					int a = e.v0;
					VertexEdgeLists[a].remove(eid);

					int b = e.v1;
					VertexEdgeLists[b].remove(eid);

					assert(EdgeRefCounts[eid] >= 1);
					EdgeRefCounts[eid]--;
				}
			}

			// Decrement vertex refcounts. If any hit 1 and we got remove-isolated flag,
			// we need to remove that vertex
			for (int j = 0; j < 3; ++j)
			{
				int vid = tv[j];
				if (vid == newv[j])     // we don't need to modify this vertex
				{
					continue;
				}
				assert(VertexRefCounts[vid] >= 1);
				VertexRefCounts[vid]--;
				if (bRemoveIsolatedVertices && VertexRefCounts[vid] == 1)
				{
					VertexRefCounts[vid]--;
					assert(VertexRefCounts[vid] == 0);
					VertexEdgeLists[vid].clear();
				}
			}


			// ok now re-insert with vertices
			for (int j = 0; j < 3; ++j)
			{
				if (newv[j] != tv[j])
				{
					Triangles[tID][j] = newv[j];
					VertexRefCounts[newv[j]]++;
				}
			}

			if (te[0] != -1)
			{
				AddTriangleEdge(tID, newv[0], newv[1], 0, e0);
			}
			if (te[1] != -1)
			{
				AddTriangleEdge(tID, newv[1], newv[2], 1, e1);
			}
			if (te[2] != -1)
			{
				AddTriangleEdge(tID, newv[2], newv[0], 2, e2);
			}

			UpdateChangeStamps();
			return EMeshResult::Ok;
		}

		void SetTriangleEdge(int idx, const Index3& val)
		{
			VectorSetSafe(TriangleEdges, idx, val);
		}

		Vector3 GetTriangleCentroid(int idx) const
		{
			const Index3 ti = Triangles[idx];
			const Vector3 centroid = (VertexPositions[ti.a] + VertexPositions[ti.b] + VertexPositions[ti.c]) / 3.0f;
			return centroid;
		}

		Vector3 GetTriangleNormal(int idx) const
		{
			Vector3 A, B, C;
			GetTriangleVertices(idx, A, B, C);
			Vector3 CA = (C - A).Unit();
			Vector3 BA = (B - A).Unit();
			return CA.Cross(BA).Unit();
		}

		Box3 GetTriangleBounds(int idx) const
		{
			const Index3 ti = Triangles[idx];
			Box3 box(VertexPositions[ti.a], VertexPositions[ti.b], VertexPositions[ti.c]);
			return box;
		}

		void GetTriangleVertices(int idx, Vector3& a, Vector3& b, Vector3& c) const
		{
			const Index3 ti = Triangles[idx];
			a = VertexPositions[ti.a];
			b = VertexPositions[ti.b];
			c = VertexPositions[ti.c];
		}

		VertexInfo GetTringleBaryPoint(int idx, float bary0, float bary1, float bary2) const
		{
			VertexInfo info;
			const Index3& t = Triangles[idx];
			info.Position = bary0 * VertexPositions[t[0]] + bary1 * VertexPositions[t[1]] + bary2 * VertexPositions[t[2]];
			info.bHasNormal = HasVertexNormals();
			if (info.bHasNormal)
			{
				info.Normal = bary0 * VertexNormals[t[0]] + bary1 * VertexNormals[t[1]] + bary2 * VertexNormals[t[2]];
				info.Normal.Normalize();
			}
			info.bHasColor = HasVertexColors();
			if (info.bHasColor)
			{
				info.Color = bary0 * VertexColors[t[0]] + bary1 * VertexColors[t[1]] + bary2 * VertexColors[t[2]];
			}
			info.NumUVs = (int)VertexUVs.size();
			for (size_t i = 0; i < VertexUVs.size(); ++i)
			{
				info.UVs[i] = bary0 * VertexUVs[i][t[0]] + bary1 * VertexUVs[i][t[1]] + bary2 * VertexUVs[i][t[2]];
			}
			return info;
		}

		inline bool TriangleHasVertex(int TriangleID, int VertexID) const
		{
			const Index3& tri = Triangles[TriangleID];
			return tri[0] == VertexID || tri[1] == VertexID || tri[2] == VertexID;
		}

		inline bool IsEdge(int idx) const 
		{
			return 0 <= idx && idx < (int)Edges.size() && EdgeRefCounts[idx] > 0;
		}

		inline bool IsBoundaryEdge(int idx) const {	return Edges[idx].t1 == -1; }
		inline const Edge& GetEdge(int idx) const { return Edges[idx]; }
		inline Index2 GetEdgeV(int idx) const { return Index2(Edges[idx].v0, Edges[idx].v1); }
		inline void GetEdgeV(int idx, Vector3& a, Vector3& b) const { a = VertexPositions[Edges[idx].v0]; b = VertexPositions[Edges[idx].v1]; }
		inline Index2 GetEdgeT(int idx) const { return Index2(Edges[idx].t0, Edges[idx].t1); }
		inline Index3 GetTriangleEdge(int idx) const { return TriangleEdges[idx]; }
		inline int GetTriangleEdge(int idx, int j) const {	return TriangleEdges[idx][j]; }

		int FindEdge(int v0, int v1)
		{
			for (auto it = VertexEdgeLists[v0].begin(); it != VertexEdgeLists[v0].end(); ++it)
			{
				int eid = *it;
				if (Edges[eid].v0 == v1 || Edges[eid].v1 == v1)
				{
					return eid;
				}
			}
			return -1;
		}

		void BuildBounds();
		inline const Box3& GetBounds() const { return Bounds; }

		inline const Transform& GetPose() const { return Pose; }
		inline void SetPose(const Transform& t) { Pose = t; }

		inline uint64_t GetChangeStamps() const { return MeshChangeStamp; }

		GeometryAttributeSet& GetAttributes() { return Attributes; }

		inline int GetOtherEdgeVertex(int EdgeID, int VertexID) const
		{
			const Edge& e = Edges[EdgeID];
			return (e.v0 == VertexID) ? e.v1 : ((e.v1 == VertexID) ? e.v0 : -1);
		}

		inline int GetOtherEdgeTriangle(int EdgeID, int TriangleID) const
		{
			const Edge&e = Edges[EdgeID];
			return (e.t0 == -1) ? e.t1 : ((e.t1 == TriangleID) ? e.t0 : -1);
		}

		inline Index2 GetOrderedOneRingEdgeTris(int VertexID, int EdgeID) const
		{
			const Index2 Tris(Edges[EdgeID].t0, Edges[EdgeID].t1);

			int vOther = GetOtherEdgeVertex(EdgeID, VertexID);
			int et1 = Tris[1];
			et1 = (et1 != -1 && TriHasSequentialVertices(et1, VertexID, vOther)) ? et1 : -1;
			int et0 = Tris[0];
			return TriHasSequentialVertices(et0, VertexID, vOther) ? Index2(et0, et1) : Index2(et1, -1);
		}

		std::vector<int> GetVexTriangles(int VertexID) const;

		bool Simplify(float rate);

		void ApplyTransform(const Transform& trans, bool bReverseOrientationIfNeeded);
		void ApplyTransform(Transform3& trans, bool bReverseOrientationIfNeeded);
		void ReverseOrientation(bool bFlipNormals);

		EMeshResult PokeTriangle(int TriangleID, const Vector3& BaryCoordinates, PokeTriangleInfo& PokeResult);
		EMeshResult SplitEdge(int eab, EdgeSplitInfo& SplitInfo, float split_t);
		EMeshResult CollapseEdge(int vKeep, int vRemove, float collapse_t, EdgeCollapseInfo& CollapseInfo);

		int FindEdgeFromTriangle(int vA, int vB, int tID) const;
		int FindEdgeFromTrianglePair(int TriA, int TriB) const;
		int GetVtxTriangleCount(int vID) const;
		int GetVtxSingleTriangle(int VertexID) const;

	private:

		bool TriHasSequentialVertices(int TriangleID, int vA, int vB) const
		{
			const Index3& Tri = Triangles[TriangleID];
			return ((Tri.a == vA && Tri.b == vB) || (Tri.b == vA && Tri.c == vB) || (Tri.c == vA && Tri.a == vB));
		}

		int ReplaceEdgeTriangle(int idx, int t_old, int t_new)
		{
			Edge& e = Edges[idx];
			int a = e.t0;
			int b = e.t1;
			if (a == t_old)
			{
				if (t_new == -1)
				{
					e.t0 = b;
					e.t1 = -1;
				}
				else
				{
					e.t0 = t_new;
				}
				return 0;
			}
			else if (b == t_old)
			{
				e.t1 = t_new;
				return 1;
			}
			else
			{
				return -1;
			}
		}

		int ReplaceTriangleVertex(int idx, int v_old, int v_new)
		{
			Index3& Triangle = Triangles[idx];
			for (int i = 0; i < 3; ++i)
			{
				if (Triangle[i] == v_old)
				{
					Triangle[i] = v_new;
					return 0;
				}
			}
			return -1;
		}

		int ReplaceTriangleEdge(int idx, int v_old, int v_new)
		{
			Index3& Triangle = TriangleEdges[idx];
			for (int i = 0; i < 3; ++i)
			{
				if (Triangle[i] == v_old)
				{
					Triangle[i] = v_new;
					return 0;
				}
			}
			return -1;
		}

		int ReplaceEdgeVertex(int idx, int v_old, int v_new)
		{
			Edge& e = Edges[idx];
			int a = e.v0, b = e.v1;
			if (a == v_old)
			{
				e.v0 = std::min(b, v_new);
				e.v1 = std::max(b, v_new);
				return 0;
			}
			else if (b == v_old)
			{
				e.v0 = std::min(a, v_new);
				e.v1 = std::max(a, v_new);
				return 1;
			}
			else
			{
				return -1;
			}
		}

		void SetEdgeTriangleEx(int idx, int t0, int t1)
		{
			Edges[idx].t0 = t0;
			Edges[idx].t1 = t1;
		}

		int AddEdgeEx(int v0, int v1, int t0, int t1)
		{
			int eid = (int)Edges.size();
			Edges.push_back({ v0, v1, t0, t1 });
			EdgeRefCounts.push_back(1);
			VertexRefCounts[v0] += 1;
			VertexRefCounts[v1] += 1;
			VertexEdgeLists[v0].push_back(eid);
			VertexEdgeLists[v1].push_back(eid);
			return eid;
		}

		void AddTriangleEdge(int TriangleID, int v0, int v1, int j, int EdgeID)
		{
			Index3& TriEdges = TriangleEdges[TriangleID];
			if (EdgeID != -1)
			{
				Edges[EdgeID].t1 = TriangleID;
				TriEdges[j] = EdgeID;
			}
			else
			{
				TriEdges[j] = AddEdgeEx(v0, v1, TriangleID, -1);
			}
		}

		int AddTriangleEx(int v0, int v1, int v2, int e0, int e1, int e2)
		{
			const Index3 tri(v0, v1, v2);
			const Index3 tri_edge(e0, e1, e2);

			int index = (int)Triangles.size();
			Triangles.push_back(tri);
			TriangleRefCounts.push_back(1);
			VertexRefCounts[tri.a] += 1;
			VertexRefCounts[tri.b] += 1;
			VertexRefCounts[tri.c] += 1;
			TriangleEdges.push_back(tri_edge);
			return index;
		}

		void UpdateChangeStamps()
		{
			MeshChangeStamp++;
		}

	private:
		// vertex
		std::vector<Vector3>				VertexPositions;
		std::vector<Vector3>				VertexNormals;
		std::vector<Vector3>				VertexColors;
		std::vector<std::vector<Vector2>>	VertexUVs;
		std::vector<uint8_t>				VertexRefCounts;
		ListSet<int>						VertexEdgeLists;

		// face
		std::vector<Index3>					Triangles;
		std::vector<Index3>					TriangleEdges;
		std::vector<uint8_t>				TriangleRefCounts;

		// edge
		std::vector<Edge>					Edges;
		std::vector<uint8_t>				EdgeRefCounts;

		GeometryAttributeSet	Attributes;
		Box3					Bounds;
		Transform				Pose;
		uint64_t				MeshChangeStamp = 0;

		std::string				ResourceName;
		bool					bHasVertexColor{ false };
		bool					bHasVertexNormals{ false };

		template<typename T>
		static void VectorSetSafe(std::vector<T>& vec, size_t index, const T& v)
		{
			if (index >= vec.size())
			{
				vec.resize(index + 1);
			}
			vec[index] = v;
		}

		template<typename T>
		static void VectorSet(std::vector<T>& Vertices, size_t index, const T& v)
		{
			Vertices[index] = v;
		}
	};

	struct FQueryOptions
	{
		float MaxDistance = FLT_MAX;

		std::function<bool(int)> TriangleFilterF = nullptr;

		FQueryOptions() {}
		FQueryOptions(float _MaxDistance, std::function<bool(int)> TriangleFilterF = nullptr) : MaxDistance(_MaxDistance), TriangleFilterF(TriangleFilterF) {}
	};

	class DynamicMeshAABBTree
	{
	public:
		DynamicMeshAABBTree(DynamicMesh* data);

		IntersectionsQueryResult FindAllIntersections(const DynamicMeshAABBTree& OtherTree, const Transform * TransformF = nullptr) const;
		
		int FindNearestTriangle(const Vector3& P, float& NearestDistSqr, const FQueryOptions& Options = FQueryOptions()) const;

		void Build();

		Box3 GetBoxEps(int IBox, float Epsilon = 1e-6f) const
		{
			Box3 box = AABB[IBox];
			box.Thicken(Epsilon);
			return box;
		}

		bool IsValid() const { return RootIndex >= 0; }

	public:
		void Build(std::vector<int> &Triangles, std::vector<Vector3> &Centers);

		struct FBoxesSet
		{
			std::vector<int> BoxToIndex;
			std::vector<Box3> AABB;
			std::vector<int> IndexList;
			int IBoxCur;
			int IIndicesCur;
			FBoxesSet()
			{
				IBoxCur = 0;
				IIndicesCur = 0;
			}
		};
		int SplitTriSetMidpoint(std::vector<int>& Triangles, std::vector<Vector3>& Centers, int IStart, int ICount, int Depth, int MinTriCount, FBoxesSet& Tris, FBoxesSet& Nodes, Box3& Box);

		Box3 GetAABB(int idx, const Transform* TransformF) const;

		float BoxDistanceSqr(int IBox, const Vector3& V) const;

		void FindIntersections(
			int iBox, const DynamicMeshAABBTree& OtherTree, const Transform* TransformF,
			int oBox, int depth, IntersectionsQueryResult& result) const;

		bool find_nearest_tri(int IBox, const Vector3& P, float& NearestDistSqr, int& TID, const FQueryOptions& Options) const;

	public:
		DynamicMesh	 *Mesh { nullptr };
		std::vector<int> BoxToIndex;
		std::vector<Box3> AABB;
		std::vector<int> IndexList;
		int TrianglesEnd = -1;
		int RootIndex = -1;
	};
}	// namespace Riemann
