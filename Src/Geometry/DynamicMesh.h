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

	constexpr static int InvalidID = -1;
	constexpr static int NonManifoldID = -2;
	constexpr static int DuplicateTriangleID = -3;

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
			// assert(false);
		}

		void OnSplitEdge(const EdgeSplitInfo& info)
		{
			// assert(false);
		}

		void OnRemoveTriangle(int tid)
		{
			// assert(false);
		}

		void OnCollapseEdge(const EdgeCollapseInfo& info)
		{
			// assert(false);
		}

		void OnNewTriangle(int tid, bool b)
		{
			// assert(false);
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

		void Clear();

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

		int AppendVertex(const Vector3& v);
		int AppendVertex(const VertexInfo& info);

		inline bool HasVertexColors() const { return bHasVertexColor; }
		inline bool HasVertexNormals() const { return bHasVertexNormals; }
		inline bool HasVertexUVs() const { return VertexUVs.size() > 0; }
		inline bool HasAttributes() const { return Attributes.MaterialIDAttrib.size() > 0; }
		inline bool IsVertex(int idx) const { return 0 <= idx && idx < (int)VertexPositions.size() && VertexRefCounts[idx] > 0;	}

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

		inline Vector3 GetVertex(int idx) const	{ return VertexPositions[idx]; }
		inline Vector3 GetVertexColor(int idx) const { return HasVertexColors() ? VertexColors[idx] : Vector3::Zero(); }
		Vector3 GetVertexNormal(int idx) const { return HasVertexNormals() ? VertexNormals[idx] : Vector3::UnitY(); }
		inline Vector2 GetVertexUV(int idx, int layer) const { return HasVertexUVs() ? VertexUVs[layer][idx] : Vector2::Zero(); }
		inline Index3 GetTriangle(int idx) const { return Triangles[idx]; }

		inline void SetVertex(int idx, const Vector3& val)	{ VectorSetSafe(VertexPositions, idx, val);	}
		inline void SetVertexColor(int idx, const Vector3& val)	{ bHasVertexColor = true; VectorSetSafe(VertexColors, idx, val); }
		inline void SetVertexNormal(int idx, const Vector3& val) { bHasVertexNormals = true; VectorSetSafe(VertexNormals, idx, val); }

		inline void SetVertexUV(int idx, const Vector2& val, int channel)
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

		inline bool IsTriangle(int idx) const {	return 0 <= idx && idx < (int)Triangles.size() && TriangleRefCounts[idx] > 0; }
		int AppendTriangle(const Index3& tv);
		EMeshResult RemoveTriangle(int tID, bool bRemoveIsolatedVertices, bool bPreserveManifold);
		void  SetTriangleInternal(int TriangleID, int v0, int v1, int v2) {	Triangles[TriangleID] = Index3(v0, v1, v2); }
		void SetTriangleEdgesInternal(int TriangleID, int e0, int e1, int e2) {	TriangleEdges[TriangleID] = Index3(e0, e1, e2);	}

		EMeshResult SetTriangle(int tID, const Index3& newv);

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

		inline bool IsEdge(int idx) const {	return 0 <= idx && idx < (int)Edges.size() && EdgeRefCounts[idx] > 0; }
		inline bool IsBoundaryEdge(int idx) const {	return Edges[idx].t1 == -1; }
		inline const Edge& GetEdge(int idx) const { return Edges[idx]; }
		inline Index2 GetEdgeV(int idx) const { return Index2(Edges[idx].v0, Edges[idx].v1); }
		inline void GetEdgeV(int idx, Vector3& a, Vector3& b) const { a = VertexPositions[Edges[idx].v0]; b = VertexPositions[Edges[idx].v1]; }
		inline Index2 GetEdgeT(int idx) const { return Index2(Edges[idx].t0, Edges[idx].t1); }
		inline Index3 GetTriangleEdge(int idx) const { return TriangleEdges[idx]; }
		inline int GetTriangleEdge(int idx, int j) const {	return TriangleEdges[idx][j]; }

		int FindEdge(int v0, int v1)
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
					if (Edges[eid].v1 == v1)
					{
						return eid;
					}
				}
			}

			return -1;
		}

		int FindEdgeEx(int vA, int vB, bool& bIsBoundary) const
		{
			int vMax = vA, vMin = vB;
			if (vB > vA)
			{
				vMax = vB; vMin = vA;
			}
			for (int eid : VertexEdgeLists[vMin])
			{
				const Edge& e = Edges[eid];
				if (e.v1 == vMax)
				{
					bIsBoundary = (e.t1 == -1);
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

		int AddEdgeInternal(int v0, int v1, int t0, int t1)
		{
			if (v1 < v0)
			{
				std::swap(v0, v1);
			}
			int eid = (int)Edges.size();
			Edges.push_back({ v0, v1, t0, t1 });
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
				Edges[EdgeID].t1 = TriangleID;
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
