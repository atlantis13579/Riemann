#pragma once

#include "assert.h"
#include <string>
#include <map>
#include <vector>
#include <functional>
#include "GeometryOperation.h"
#include "GeometryAttributes.h"
#include "../Core/ListSet.h"
#include "../Maths/Box3.h"
#include "../Maths/Vector2.h"
#include "../Maths/Vector3.h"
#include "../Maths/Index2.h"
#include "../Maths/Index3.h"
#include "../Maths/Transform.h"

namespace Riemann
{
	template<typename T>
	static void VectorSetSafe(std::vector<T>& vec, int index, const T& v, const T& default_val)
	{
		if (index >= vec.size())
		{
			vec.resize(index + 1, default_val);
		}
		vec[index] = v;
	}

	template<typename T>
	static void VectorSet(std::vector<T>& Vertices, int index, const T& v)
	{
		Vertices[index] = v;
	}
	template<typename T>
	static bool SamePairUnordered(T a0, T a1, T b0, T b1)
	{
		return (a0 == b0) ?
			(a1 == b1) :
			(a0 == b1 && a1 == b0);
	}

	template<typename T, typename Vec>
	static int FindTriIndex(T VertexID, const Vec& TriangleVerts)
	{
		if (TriangleVerts[0] == VertexID) return 0;
		if (TriangleVerts[1] == VertexID) return 1;
		if (TriangleVerts[2] == VertexID) return 2;
		return -1;
	}

	template<typename T, typename Vec>
	static int FindTriOtherVtx(T VertexID1, T VertexID2, const Vec& TriangleVerts)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (SamePairUnordered(VertexID1, VertexID2, TriangleVerts[j], TriangleVerts[(j + 1) % 3]))
			{
				return TriangleVerts[(j + 2) % 3];
			}
		}
		return -1;
	}

	inline int GetOtherTriIndex(int i0, int i1)
	{
		// @todo can we do this with a formula? I don't see it right now.
		static const int values[4] = { 0, 2, 1, 0 };
		return values[i0 + i1];
	}

	struct VertexInfo
	{
		Vector3 Position{ Vector3::Zero() };
		Vector3 Normal{ Vector3::Zero() };
		Vector3 Color{ Vector3::Zero() };
		Vector2 UVs{ Vector2::Zero() };
		bool bHasNormal{ false };
		bool bHasColor{ false };
		bool bHasUV{ false };
	};


	class DynamicMeshAABBTree;

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

		void Clear();

		bool LoadObj(const char* name);
		bool ExportObj(const char* name) const;
		static DynamicMesh* CreateFromObj(const char* name);

		struct Edge
		{
			Index2 Vert;
			Index2 Tri;
		};

		void SetData(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals);

		int GetVertexCount() const { return (int)VertexPositions.size(); }
		Vector3* GetVertexBuffer() { return VertexPositions.data(); }
		Index3* GetIndexBuffer() { return Triangles.data(); }

		int AppendVertex(const Vector3& v);
		int AppendVertex(const VertexInfo& info);
		int AppendVertex(const DynamicMesh& SourceMesh, int SourceVertexID);

		inline bool HasVertexColors() const { return bHasVertexColor; }
		inline bool HasVertexNormals() const { return bHasVertexNormals; }
		inline bool HasVertexUVs() const { return VertexUVs.size() > 0; }
		inline bool HasTriangleGroups() const { return bHasTriangleGroups; }
		inline bool IsVertex(int idx) const { return 0 <= idx && idx < (int)VertexPositions.size() && VertexRefCounts[idx] > 0;	}
		inline bool IsVertexFast(int idx) const { return VertexRefCounts[idx] > 0; }

		inline bool IsBoundaryVertex(int vID) const
		{
			assert(IsVertex(vID));
			if (IsVertex(vID))
			{
				for (int eid : VertexEdgeLists[vID])
				{
					if (Edges[eid].Tri[1] == -1)
					{
						return true;
					}
				}
			}
			return false;
		}

		inline Vector3 GetVertex(int idx) const	{ return VertexPositions[idx]; }
		inline Vector3 GetVertexColor(int idx) const { return HasVertexColors() ? VertexColors[idx] : Vector3::Zero(); }
		Vector3 GetVertexNormal(int idx) const { return HasVertexNormals() ? VertexNormals[idx] : Vector3::UnitY(); }
		inline Vector2 GetVertexUV(int idx) const { return HasVertexUVs() ? VertexUVs[idx] : Vector2::Zero(); }
		inline Index3 GetTriangle(int idx) const { return Triangles[idx]; }

		inline void SetVertex(int idx, const Vector3& val)	{ VectorSetSafe(VertexPositions, idx, val, Vector3::Zero());	}
		inline void SetVertexColor(int idx, const Vector3& val)	{ bHasVertexColor = true; VectorSetSafe(VertexColors, idx, val, Vector3::Zero()); }
		inline void SetVertexNormal(int idx, const Vector3& val) { bHasVertexNormals = true; VectorSetSafe(VertexNormals, idx, val, Vector3::UnitY()); }
		inline void SetVertexUV(int idx, const Vector2& val) {	VectorSetSafe(VertexUVs, idx, val, Vector2::Zero()); }

		void EnableVertexNormals(const Vector3& InitialNormal)
		{
			if (HasVertexNormals())
			{
				return;
			}

			std::vector<Vector3> NewNormals;
			int NV = GetVertexCount();
			NewNormals.resize(NV);
			for (int i = 0; i < NV; ++i)
			{
				NewNormals[i] = InitialNormal;
			}
			VertexNormals = NewNormals;
		}

		void DiscardVertexNormals()
		{
			VertexNormals.clear();
		}

		int GetTriangleCount() const { return (int)Triangles.size(); }
		int AppendTriangle(const Index3& tv, int gid = 0);
		int AppendTriangle(int i0, int i1, int i2, int gid = 0) { return AppendTriangle(Index3(i0, i1, i2), gid); }
		EMeshResult InsertTriangle(int tid, const Index3& tv, int gid = 0, bool bUnsafe = false);
		EMeshResult RemoveTriangle(int tID, bool bRemoveIsolatedVertices, bool bPreserveManifold);
		inline bool IsTriangle(int idx) const {	return 0 <= idx && idx < (int)Triangles.size() && TriangleRefCounts[idx] > 0; }
		inline bool IsTriangleFast(int idx) const { return TriangleRefCounts[idx] > 0; }
		void  SetTriangleInternal(int TriangleID, int v0, int v1, int v2) {	Triangles[TriangleID] = Index3(v0, v1, v2); }
		void SetTriangleEdgesInternal(int TriangleID, int e0, int e1, int e2) {	TriangleEdges[TriangleID] = Index3(e0, e1, e2);	}

		EMeshResult SetTriangle(int tID, const Index3& newv);

		int GetTriangleGroup(int tID) const
		{
			return (HasTriangleGroups() == false) ? -1 :
				(TriangleRefCounts[tID] > 0 ? TriangleGroups[tID] : 0);
		}

		void SetTriangleGroup(int tid, int group_id)
		{
			if (HasTriangleGroups())
			{
				assert(IsTriangle(tid));
				TriangleGroups[tid] = group_id;
				GroupIDCounter = std::max(GroupIDCounter, group_id + 1);
			}
		}

		int AllocateTriangleGroup()
		{
			return GroupIDCounter++;
		}

		Vector3 GetTriCentroid(int idx) const
		{
			const Index3 ti = Triangles[idx];
			const Vector3 centroid = (VertexPositions[ti.a] + VertexPositions[ti.b] + VertexPositions[ti.c]) / 3.0f;
			return centroid;
		}

		Vector3 GetTriNormal(int idx) const
		{
			Vector3 A, B, C;
			GetTriVertices(idx, A, B, C);
			Vector3 CA = (C - A).Unit();
			Vector3 BA = (B - A).Unit();
			return CA.Cross(BA).Unit();
		}

		Box3 GetTriBounds(int idx) const
		{
			const Index3 ti = Triangles[idx];
			Box3 box(VertexPositions[ti.a], VertexPositions[ti.b], VertexPositions[ti.c]);
			return box;
		}

		void GetTriVertices(int idx, Vector3& a, Vector3& b, Vector3& c) const
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
			info.bHasUV = HasVertexUVs();
			if (info.bHasColor)
			{
				info.UVs = bary0 * VertexUVs[t[0]] + bary1 * VertexUVs[t[1]] + bary2 * VertexUVs[t[2]];
			}
			return info;
		}

		inline bool TriangleHasVertex(int TriangleID, int VertexID) const
		{
			const Index3& tri = Triangles[TriangleID];
			return tri[0] == VertexID || tri[1] == VertexID || tri[2] == VertexID;
		}

		int GetEdgeCount() const { return (int)Edges.size(); }
		inline bool IsEdge(int idx) const {	return 0 <= idx && idx < (int)Edges.size() && EdgeRefCounts[idx] > 0; }
		inline bool IsEdgeFast(int idx) const { return EdgeRefCounts[idx] > 0; }
		inline bool IsBoundaryEdge(int idx) const {	return Edges[idx].Tri[1] == -1; }
		inline const Edge& GetEdge(int idx) const { return Edges[idx]; }
		inline Edge& GetEdge(int idx) { return Edges[idx]; }
		inline Index2 GetEdgeV(int idx) const { return Index2(Edges[idx].Vert[0], Edges[idx].Vert[1]); }
		inline void GetEdgeV(int idx, Vector3& a, Vector3& b) const { a = VertexPositions[Edges[idx].Vert[0]]; b = VertexPositions[Edges[idx].Vert[1]]; }
		inline Index2 GetEdgeT(int idx) const { return Index2(Edges[idx].Tri[0], Edges[idx].Tri[1]); }
		inline Index3 GetTriEdges(int idx) const { return TriangleEdges[idx]; }
		inline int GetTriEdges(int idx, int j) const {	return TriangleEdges[idx][j]; }

		int FindEdge(int v0, int v1) const
		{
			if (v0 == v1)
			{
				return -1;
			}
			else if (v0 > v1)
			{
				std::swap(v0, v1);
			}

			if (IsVertex(v0))
			{
				for (int eid : VertexEdgeLists[v0])
				{
					if (Edges[eid].Vert[1] == v1)
					{
						return eid;
					}
				}
			}

			return -1;
		}

		int FindEdgeEx(int v0, int v1, bool& bIsBoundary) const
		{
			int vMax = v0, vMin = v1;
			if (v1 > v0)
			{
				vMax = v1; vMin = v0;
			}
			for (int eid : VertexEdgeLists[vMin])
			{
				const Edge& e = Edges[eid];
				if (e.Vert[1] == vMax)
				{
					bIsBoundary = (e.Tri[1] == -1);
					return eid;
				}
			}

			return -1;
		}

		Index2 GetEdgeOpposingV(int EdgeID) const;

		void BuildBounds();
		inline const Box3& GetBounds() const { return Bounds; }

		inline const Transform& GetPose() const { return Pose; }
		inline void SetPose(const Transform& t) { Pose = t; }

		inline uint64_t GetChangeStamps() const { return MeshChangeStamp; }

		inline bool HasAttributes() const { return mAttributes != nullptr; }
		DynamicMeshAttributeSet* Attributes() { return mAttributes; }
		const DynamicMeshAttributeSet* Attributes() const { return mAttributes; }

		void EnableAttributes()
		{
			if (HasAttributes())
			{
				return;
			}
			mAttributes = new DynamicMeshAttributeSet(this);
			mAttributes->Initialize(GetVertexCount(), GetTriangleCount());
		}

		inline int GetOtherEdgeVertex(int EdgeID, int VertexID) const
		{
			const Edge& e = Edges[EdgeID];
			return (e.Vert[0] == VertexID) ? e.Vert[1] : ((e.Vert[1] == VertexID) ? e.Vert[0] : -1);
		}

		inline int GetOtherEdgeTriangle(int EdgeID, int TriangleID) const
		{
			const Edge&e = Edges[EdgeID];
			return (e.Tri[0] == -1) ? e.Tri[1] : ((e.Tri[1] == TriangleID) ? e.Tri[0] : -1);
		}

		inline Index2 GetOrderedOneRingEdgeTris(int VertexID, int EdgeID) const
		{
			const Index2 Tris(Edges[EdgeID].Tri[0], Edges[EdgeID].Tri[1]);

			int vOther = GetOtherEdgeVertex(EdgeID, VertexID);
			int et1 = Tris[1];
			et1 = (et1 != -1 && TriHasSequentialVertices(et1, VertexID, vOther)) ? et1 : -1;
			int et0 = Tris[0];
			return TriHasSequentialVertices(et0, VertexID, vOther) ? Index2(et0, et1) : Index2(et1, -1);
		}

		std::vector<int> GetVexVertices(int VertexID) const;
		std::vector<int> GetVexEdges(int VertexID) const;
		std::vector<int> GetVexTriangles(int VertexID) const;

		template<typename TFunction>
		void EnumerateVertexVertices(int VertexID, TFunction func) const
		{
			VertexEdgeLists[VertexID].all_iteration(
				[this, &func, VertexID](int eid)
				{
					func(GetOtherEdgeVertex(eid, VertexID));
				});
		}

		template<typename TFunction>
		void EnumerateVertexEdges(int VertexID, TFunction func) const
		{
			VertexEdgeLists[VertexID].all_iteration(func);
		}

		template<typename TFunction>
		void EnumerateVertexTriangles(int VertexID, TFunction func) const
		{
			if (!IsVertex(VertexID))
			{
				return;
			}

			VertexEdgeLists[VertexID].all_iteration(
				[&](int eid)
				{
					const Edge e = Edges[eid];
					const int vOther = e.Vert.a == VertexID ? e.Vert.b : e.Vert.a;
					if (TriHasSequentialVertices(e.Tri[0], VertexID, vOther))
					{
						func(e.Tri[0]);
					}
					if (e.Tri[1] != -1 && TriHasSequentialVertices(e.Tri[1], VertexID, vOther))
					{
						func(e.Tri[1]);
					}
				});
		}

		bool Simplify(float rate);

		void ApplyTransform(const Transform& trans, bool bReverseOrientationIfNeeded);
		void ApplyTransform(Transform3& trans, bool bReverseOrientationIfNeeded);
		void ReverseOrientation(bool bFlipNormals);

		EMeshResult MergeEdges(int eKeep, int eDiscard, FMergeEdgesInfo& MergeInfo, bool bCheckValidOrientation);
		EMeshResult PokeTriangle(int TriangleID, const Vector3& BaryCoordinates, FPokeTriangleInfo& PokeResult);
		EMeshResult SplitEdge(int eab, FEdgeSplitInfo& SplitInfo, float split_t);
		EMeshResult CollapseEdge(int vKeep, int vRemove, float collapse_t, FEdgeCollapseInfo& CollapseInfo);

		int FindEdgeFromTriangle(int vA, int vB, int tID) const;
		int FindEdgeFromTrianglePair(int TriA, int TriB) const;
		int GetVtxTriangleCount(int vID) const;
		int GetVtxSingleTriangle(int VertexID) const;

		EMeshResult ReverseTriOrientation(int tID)
		{
			if (!IsTriangle(tID))
			{
				return EMeshResult::Failed_NotATriangle;
			}
			ReverseTriOrientationInternal(tID);
			UpdateChangeStamps();
			return EMeshResult::Ok;
		}

		void ReverseTriOrientationInternal(int tID)
		{
			Index3 t = GetTriangle(tID);
			SetTriangleInternal(tID, t[1], t[0], t[2]);
			Index3 te = GetTriEdges(tID);
			SetTriangleEdgesInternal(tID, te[0], te[2], te[1]);
			if (HasAttributes())
			{
				mAttributes->OnReverseTriOrientation(tID);
			}
		}

	private:

		bool TriHasSequentialVertices(int TriangleID, int vA, int vB) const
		{
			const Index3& Tri = Triangles[TriangleID];
			return ((Tri.a == vA && Tri.b == vB) || (Tri.b == vA && Tri.c == vB) || (Tri.c == vA && Tri.a == vB));
		}

		int ReplaceEdgeTriangle(int idx, int t_old, int t_new)
		{
			Edge& e = Edges[idx];
			int a = e.Tri[0];
			int b = e.Tri[1];
			if (a == t_old)
			{
				if (t_new == -1)
				{
					e.Tri[0] = b;
					e.Tri[1] = -1;
				}
				else
				{
					e.Tri[0] = t_new;
				}
				return 0;
			}
			else if (b == t_old)
			{
				e.Tri[1] = t_new;
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
			int a = e.Vert[0], b = e.Vert[1];
			if (a == v_old)
			{
				e.Vert[0] = std::min(b, v_new);
				e.Vert[1] = std::max(b, v_new);
				return 0;
			}
			else if (b == v_old)
			{
				e.Vert[0] = std::min(a, v_new);
				e.Vert[1] = std::max(a, v_new);
				return 1;
			}
			else
			{
				return -1;
			}
		}

		inline void SetEdgeVerticesInternal(int idx, int a, int b)
		{
			if (a > b)
			{
				std::swap(a, b);
			}
			Edges[idx].Vert[0] = a;
			Edges[idx].Vert[1] = b;
		}

		void SetEdgeTrianglesInternal(int idx, int t0, int t1)
		{
			Edges[idx].Tri[0] = t0;
			Edges[idx].Tri[1] = t1;
		}

		int AddEdgeInternal(int v0, int v1, int t0, int t1)
		{
			if (v1 < v0)
			{
				std::swap(v0, v1);
			}
			int eid = (int)Edges.size();
			Edge e;
			e.Vert = Index2(v0, v1);
			e.Tri = Index2(t0, t1);
			Edges.push_back(e);
			EdgeRefCounts.push_back(1);
			VertexEdgeLists[v0].push_back(eid);
			VertexEdgeLists[v1].push_back(eid);
			return eid;
		}

		void AddTriangleEdge(int TriangleID, int v0, int v1, int j, int EdgeID)
		{
			if (TriangleID >= (int)TriangleEdges.size())
			{
				TriangleEdges.resize(TriangleID + 1, Index3(-1, -1, -1));
			}
			Index3& TriEdges = TriangleEdges[TriangleID];
			if (EdgeID != -1)
			{
				Edges[EdgeID].Tri[1] = TriangleID;
				TriEdges[j] = EdgeID;
			}
			else
			{
				TriEdges[j] = AddEdgeInternal(v0, v1, TriangleID, -1);
			}
		}

		int AddTriangleInternal(int v0, int v1, int v2, int e0, int e1, int e2)
		{
			const Index3 tri(v0, v1, v2);
			const Index3 tri_edge(e0, e1, e2);

			int index = (int)Triangles.size();
			Triangles.push_back(tri);
			TriangleRefCounts.push_back(1);
			TriangleEdges.push_back(tri_edge);
			return index;
		}

		void UpdateChangeStamps()
		{
			MeshChangeStamp++;
		}

	private:
		// vertex
		std::vector<Vector3>	VertexPositions;
		std::vector<Vector3>	VertexNormals;
		std::vector<Vector3>	VertexColors;
		std::vector<Vector2>	VertexUVs;
		std::vector<uint8_t>	VertexRefCounts;
		ListSet<int>			VertexEdgeLists;
		
		// face
		std::vector<Index3>		Triangles;
		std::vector<Index3>		TriangleEdges;
		std::vector<uint8_t>	TriangleRefCounts;
		std::vector<int>		TriangleGroups;
		int						GroupIDCounter;

		// edge
		std::vector<Edge>		Edges;
		std::vector<uint8_t>	EdgeRefCounts;

		DynamicMeshAttributeSet	*mAttributes { nullptr };
		Box3					Bounds;
		Transform				Pose;
		uint64_t				MeshChangeStamp = 0;

		std::string				ResourceName;
		bool					bHasVertexColor{ false };
		bool					bHasVertexNormals{ false };
		bool					bHasVertexUVs{ false };
		bool					bHasTriangleGroups{ false };
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


	struct FIndexMapi
	{
	protected:
		std::map<int, int> ForwardMap;
		std::map<int, int> ReverseMap;
		bool bWantForward;
		bool bWantReverse;

	public:

		FIndexMapi()
		{
			bWantForward = bWantReverse = true;
		}

		void Reset()
		{
			ForwardMap.clear();
			ReverseMap.clear();
		}

		constexpr int UnmappedID() const { return -1; }

		std::map<int, int>& GetForwardMap() { return ForwardMap; }
		const std::map<int, int>& GetForwardMap() const { return ForwardMap; }

		std::map<int, int>& GetReverseMap() { return ReverseMap; }
		const std::map<int, int>& GetReverseMap() const { return ReverseMap; }

		inline void Add(int FromID, int ToID)
		{
			assert(FromID >= 0 && ToID >= 0);
			ForwardMap.emplace(FromID, ToID);
			ReverseMap.emplace(ToID, FromID);
		}

		inline bool ContainsFrom(int FromID) const
		{
			assert(FromID >= 0);
			assert(bWantForward);
			return ForwardMap.count(FromID) > 0;
		}

		inline bool ContainsTo(int ToID) const
		{
			assert(ToID >= 0);
			assert(bWantReverse);
			return ReverseMap.count(ToID) > 0;
		}


		inline int GetTo(int FromID) const
		{
			assert(FromID >= 0);
			assert(bWantForward);
			auto it = ForwardMap.find(FromID);
			if (it == ForwardMap.end())
			{
				return UnmappedID();
			}
			return it->second;
		}

		inline int GetFrom(int ToID) const
		{
			assert(ToID >= 0);
			assert(bWantReverse);
			auto it = ReverseMap.find(ToID);
			if (it == ReverseMap.end())
			{
				return UnmappedID();
			}
			return it->second;
		}

		inline const int* FindTo(int FromID) const
		{
			assert(FromID >= 0);
			assert(bWantForward);
			auto it = ForwardMap.find(FromID);
			if (it == ForwardMap.end())
			{
				return nullptr;
			}
			return &it->second;
		}

		inline const int* FindFrom(int ToID) const
		{
			assert(ToID >= 0);
			assert(bWantReverse);
			auto it = ReverseMap.find(ToID);
			if (it == ReverseMap.end())
			{
				return nullptr;
			}
			return &it->second;
		}

		void Reserve(int NumElements)
		{
		}
	};

	class FMeshIndexMappings
	{
	protected:
		FIndexMapi VertexMap;
		FIndexMapi TriangleMap;
		FIndexMapi GroupMap;

		FIndexMapi ColorMap;
		std::vector<FIndexMapi> UVMaps;
		std::vector<FIndexMapi> NormalMaps;

	public:
		constexpr int InvalidID() const { return VertexMap.UnmappedID(); }
		void Initialize(DynamicMesh* Mesh);

		void Reset()
		{
			VertexMap.Reset();
			TriangleMap.Reset();
			GroupMap.Reset();
			ColorMap.Reset();
			for (FIndexMapi& UVMap : UVMaps)
			{
				UVMap.Reset();
			}
			for (FIndexMapi& NormalMap : NormalMaps)
			{
				NormalMap.Reset();
			}
		}

		void ResetTriangleMap()
		{
			TriangleMap.Reset();
		}

		FIndexMapi& GetVertexMap() { return VertexMap; }
		const FIndexMapi& GetVertexMap() const { return VertexMap; }
		inline void SetVertex(int FromID, int ToID) { VertexMap.Add(FromID, ToID); }
		inline int GetNewVertex(int FromID) const { return VertexMap.GetTo(FromID); }
		inline bool ContainsVertex(int FromID) const { return VertexMap.ContainsFrom(FromID); }

		FIndexMapi& GetTriangleMap() { return TriangleMap; }
		const FIndexMapi& GetTriangleMap() const { return TriangleMap; }
		void SetTriangle(int FromID, int ToID) { TriangleMap.Add(FromID, ToID); }
		int GetNewTriangle(int FromID) const { return TriangleMap.GetTo(FromID); }
		inline bool ContainsTriangle(int FromID) const { return TriangleMap.ContainsFrom(FromID); }

		FIndexMapi& GetGroupMap() { return GroupMap; }
		const FIndexMapi& GetGroupMap() const { return GroupMap; }
		void SetGroup(int FromID, int ToID) { GroupMap.Add(FromID, ToID); }
		int GetNewGroup(int FromID) const { return GroupMap.GetTo(FromID); }
		inline bool ContainsGroup(int FromID) const { return GroupMap.ContainsFrom(FromID); }

		FIndexMapi& GetUVMap(int UVLayer) { return UVMaps[UVLayer]; }
		void SetUV(int UVLayer, int FromID, int ToID) { UVMaps[UVLayer].Add(FromID, ToID); }
		int GetNewUV(int UVLayer, int FromID) const { return UVMaps[UVLayer].GetTo(FromID); }
		inline bool ContainsUV(int UVLayer, int FromID) const { return UVMaps[UVLayer].ContainsFrom(FromID); }

		FIndexMapi& GetNormalMap(int NormalLayer) { return NormalMaps[NormalLayer]; }
		void SetNormal(int NormalLayer, int FromID, int ToID) { NormalMaps[NormalLayer].Add(FromID, ToID); }
		int GetNewNormal(int NormalLayer, int FromID) const { return NormalMaps[NormalLayer].GetTo(FromID); }
		inline bool ContainsNormal(int NormalLayer, int FromID) const { return NormalMaps[NormalLayer].ContainsFrom(FromID); }

		FIndexMapi& GetColorMap() { return ColorMap; }
		void SetColor(int FromID, int ToID) { ColorMap.Add(FromID, ToID); }
		int GetNewColor(int FromID) const { return ColorMap.GetTo(FromID); }
		inline bool ContainsColor(int FromID) const { return ColorMap.ContainsFrom(FromID); }

	};


	struct FDynamicMeshEditResult
	{
		std::vector<int> NewVertices;
		std::vector<int> NewTriangles;
		std::vector<Index2> NewQuads;
		std::vector<std::vector<int>> NewPolygons;
		std::vector<int> NewGroups;
		std::vector<std::vector<int>> NewNormalOverlayElements;
		std::vector<int> NewColorOverlayElements;

		void Reset()
		{
			NewVertices.clear();
			NewTriangles.clear();
			NewQuads.clear();
			NewPolygons.clear();
			NewGroups.clear();
			NewNormalOverlayElements.clear();
			NewColorOverlayElements.clear();
		}

		void GetAllTriangles(std::vector<int>& TrianglesOut) const;
	};

	class FDynamicMeshEditor
	{
	public:
		DynamicMesh* Mesh;

		FDynamicMeshEditor(DynamicMesh* MeshIn)
		{
			Mesh = MeshIn;
		}

		void ReverseTriangleOrientations(const std::vector<int>& Triangles, bool bInvertNormals);

		void InvertTriangleNormals(const std::vector<int>& Triangles);

		void AppendMesh(const DynamicMesh* AppendMesh, FMeshIndexMappings& IndexMapsOut,
			std::function<Vector3(int, const Vector3&)> PositionTransform = nullptr,
			std::function<Vector3(int, const Vector3&)> NormalTransform = nullptr);

		void AppendNormals(const DynamicMesh* AppendMesh,
			const FDynamicMeshNormalOverlay* FromNormals, FDynamicMeshNormalOverlay* ToNormals,
			const FIndexMapi& VertexMap, const FIndexMapi& TriangleMap,
			std::function<Vector3(int, const Vector3&)> NormalTransform,
			FIndexMapi& NormalMapOut);

		void AppendUVs(const DynamicMesh* AppendMesh,
			const FDynamicMeshUVOverlay* FromUVs, FDynamicMeshUVOverlay* ToUVs,
			const FIndexMapi& VertexMap, const FIndexMapi& TriangleMap,
			FIndexMapi& UVMapOut);

		void AppendColors(const DynamicMesh* AppendMesh,
			const FDynamicMeshColorOverlay* FromOverlay, FDynamicMeshColorOverlay* ToOverlay,
			const FIndexMapi& VertexMap, const FIndexMapi& TriangleMap,
			FIndexMapi& ColorMapOut);

		void AppendTriangles(const DynamicMesh* SourceMesh, const std::vector<int>& SourceTriangles,
			FMeshIndexMappings& IndexMaps, FDynamicMeshEditResult& ResultOut, bool bComputeTriangleMap = true);

		/*
		bool RemoveIsolatedVertices();

		bool StitchVertexLoopsMinimal(const std::vector<int>& VertexLoop1, const std::vector<int>& VertexLoop2, FDynamicMeshEditResult& ResultOut);

		bool StitchVertexLoopToTriVidPairSequence(
			const std::vector<std::pair<int, std::pair<char, char>>>& TriVidPairs1,
			const std::vector<int>& VertexLoop2, FDynamicMeshEditResult& ResultOut);

		static bool ConvertLoopToTriVidPairSequence(const DynamicMesh& Mesh,
			const std::vector<int>& VidLoop, const std::vector<int>& EdgeLoop,
			std::vector<std::pair<int, std::pair<char, char>>>& TriVertPairsOut);

		bool StitchSparselyCorrespondedVertexLoops(const std::vector<int>& VertexIDs1, const std::vector<int>& MatchedIndices1, const std::vector<int>& VertexIDs2, const std::vector<int>& MatchedIndices2, FDynamicMeshEditResult& ResultOut, bool bReverseOrientation = false);

		bool WeldVertexLoops(const std::vector<int>& VertexLoop1, const std::vector<int>& VertexLoop2);

		bool AddTriangleFan_OrderedVertexLoop(int CenterVertex, const std::vector<int>& VertexLoop, int GroupID, FDynamicMeshEditResult& ResultOut);

		void DuplicateTriangles(const std::vector<int>& Triangles, FMeshIndexMappings& IndexMaps, FDynamicMeshEditResult& ResultOut);

		struct FLoopPairSet
		{
			std::vector<int> OuterVertices;
			std::vector<int> OuterEdges;

			std::vector<int> InnerVertices;
			std::vector<int> InnerEdges;

			bool bOuterIncludesIsolatedVertices;
		};

		bool DisconnectTriangles(const std::vector<int>& Triangles, std::vector<FLoopPairSet>& LoopSetOut, bool bHandleBoundaryVertices);

		bool DisconnectTriangles(const TSet<int>& TriangleSet, const std::vector<FEdgeLoop>& BoundaryLoops, std::vector<FLoopPairSet>& LoopSetOut, bool bAllowBoundaryVertices);

		void DisconnectTriangles(const std::vector<int>& Triangles, bool bPreventBowties = true);

		void SplitBowties(FDynamicMeshEditResult& ResultOut);

		void SplitBowties(int VertexID, FDynamicMeshEditResult& ResultOut);

		void SplitBowtiesAtTriangles(const std::vector<int>& TriangleIDs, FDynamicMeshEditResult& ResultOut);

		enum class EDuplicateTriBehavior : unsigned char
		{
			EnsureContinue,
			EnsureAbort, UseExisting, Replace
		};

		bool ReinsertSubmesh(const FDynamicSubmesh3& Submesh, FOptionallySparseIndexMap& SubToNewV, std::vector<int>* NewTris = nullptr,
			EDuplicateTriBehavior DuplicateBehavior = EDuplicateTriBehavior::EnsureAbort);

		bool RemoveTriangles(const std::vector<int>& Triangles, bool bRemoveIsolatedVerts);

		int RemoveSmallComponents(double MinVolume, double MinArea = 0.0, int MinTriangleCount = 0);

		bool RemoveTriangles(const std::vector<int>& Triangles, bool bRemoveIsolatedVerts, TFunctionRef<void(int)> OnRemoveTriFunc);

		Vector3 ComputeAndSetQuadNormal(const Index2& QuadTris, bool bIsPlanar = false);

		void SetQuadNormals(const Index2& QuadTris, const Vector3& Normal);

		void SetTriangleNormals(const std::vector<int>& Triangles, const Vector3& Normal);

		void SetTriangleNormals(const std::vector<int>& Triangles);

		void SetTubeNormals(const std::vector<int>& Triangles, const std::vector<int>& VertexIDs1, const std::vector<int>& MatchedIndices1, const std::vector<int>& VertexIDs2, const std::vector<int>& MatchedIndices2);

		void SetQuadUVsFromProjection(const Index2& QuadTris, const FFrame3d& ProjectionFrame, float UVScaleFactor = 1.0f, const Vector2& UVTranslation = Vector2::Zero(), int UVLayerIndex = 0);

		void SetTriangleUVsFromProjection(const std::vector<int>& Triangles, const FFrame3d& ProjectionFrame,
			float UVScaleFactor = 1.0f, const Vector2& UVTranslation = Vector2::Zero(), bool bShiftToOrigin = true, int UVLayerIndex = 0);

		void SetTriangleUVsFromProjection(const std::vector<int>& Triangles, const FFrame3d& ProjectionFrame,
			const Vector2& UVScale = Vector2::One(), const Vector2& UVTranslation = Vector2::Zero(), int UVLayerIndex = 0,
			bool bShiftToOrigin = true, bool bNormalizeBeforeScaling = false);

		void SetGeneralTubeUVs(const std::vector<int>& Triangles, const std::vector<int>& VertexIDs1, const std::vector<int>& MatchedIndices1, const std::vector<int>& VertexIDs2, const std::vector<int>& MatchedIndices2, const std::vector<float>& UValues, const Vector3& VDir, float UVScaleFactor = 1.0f, const Vector2& UVTranslation = Vector2::Zero(), int UVLayerIndex = 0);

		void RescaleAttributeUVs(float UVScale = 1.0f, bool bWorldSpace = false, int UVLayerIndex = 0, TOptional<FTransformSRT3d> ToWorld = TOptional<FTransformSRT3d>());

		int FindOrCreateDuplicateVertex(int VertexID, FMeshIndexMappings& IndexMaps, FDynamicMeshEditResult& ResultOut);

		int FindOrCreateDuplicateGroup(int TriangleID, FMeshIndexMappings& IndexMaps, FDynamicMeshEditResult& ResultOut);

		int FindOrCreateDuplicateUV(int ElementID, int UVLayerIndex, FMeshIndexMappings& IndexMaps);

		int FindOrCreateDuplicateNormal(int ElementID, int NormalLayerIndex, FMeshIndexMappings& IndexMaps, FDynamicMeshEditResult* ResultOut = nullptr);

		int FindOrCreateDuplicateColor(int ElementID, FMeshIndexMappings& IndexMaps, FDynamicMeshEditResult* ResultOut);

		void CopyAttributes(int FromTriangleID, int ToTriangleID, FMeshIndexMappings& IndexMaps, FDynamicMeshEditResult& ResultOut);

		void AppendMesh(const TTriangleMeshAdapter<double>* AppendMesh, FMeshIndexMappings& IndexMapsOut,
			std::function<Vector3(int, const Vector3&)> PositionTransform = nullptr);

		static bool SplitMesh(const DynamicMesh* SourceMesh, std::vector<DynamicMesh>& SplitMeshes, TFunctionRef<int(int)> TriIDToMeshID, int DeleteMeshID = -1);
		*/
	};
}	// namespace Riemann
