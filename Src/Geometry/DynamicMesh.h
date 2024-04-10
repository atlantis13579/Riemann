#pragma once

#include "assert.h"
#include <string>
#include <map>
#include <vector>
#include "../Maths/Box3.h"
#include "../Maths/Vector2.h"
#include "../Maths/Vector3.h"
#include "../Maths/Transform.h"

namespace Riemann
{
	class DynamicMeshAABBTree;

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
		Vector2i OriginalVertices;
		Vector2i OtherVertices;
		Vector2i OriginalTriangles;
		bool bIsBoundary;
		int NewVertex;
		Vector2i NewTriangles;
		Vector3i NewEdges;
		float SplitT;
	};

	struct PokeTriangleInfo
	{
		int OriginalTriangle;
		Vector3i TriVertices;
		int NewVertex;
		Vector2i NewTriangles;
		Vector3i NewEdges;
		Vector3 BaryCoords;
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
			VertexToEdge.clear();

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

			static uint64_t Pack(int id1, int id2)
			{
				if (id1 > id2)
				{
					int t = id1;
					id1 = id2;
					id2 = t;
				}
				return ((uint64_t)id1 << 32) | id2;
			}

			static void Unpack(uint64_t key, int* id1, int* id2)
			{
				*id1 = key >> 32;
				*id2 = key & 0xFFFFFFFF;
			}
		};

		int GetNumVertices() const
		{
			return (int)VertexPositions.size();
		}

		int GetNumTriangles() const
		{
			return (int)Triangles.size();
		}

		Vector3* GetVertexBuffer()
		{
			return VertexPositions.data();
		}

		Vector3i* GetIndexBuffer()
		{
			return Triangles.data();
		}

		int AppendVertex(const Vector3& v)
		{
			int index = (int)VertexPositions.size();
			VertexPositions.push_back(v);
			VertexRefCounts.push_back(1);
			return index;
		}

		int AppendVertex(const VertexInfo& info)
		{
			size_t index = VertexPositions.size();

			VertexPositions.push_back(info.Position);
			VertexRefCounts.push_back(1);

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

		int AppendTriangle(const Vector3i& tri)
		{
			int index = (int)Triangles.size();

			Vector3i tri_edge;

			for (int j = 0; j < 3; ++j)
			{
				int v0 = tri[j];
				int v1 = tri[(j + 1) % 3];
				if (v0 > v1)
				{
					std::swap(v0, v1);
				}

				uint64_t e = Edge::Pack(v0, v1);

				int edge_idx;
				if (VertexToEdge.find(e) == VertexToEdge.end())
				{
					// new edge
					edge_idx = AppendEdgeEx(v0, v1, (int)index, -1);
					VertexToEdge[e] = edge_idx;
				}
				else
				{
					edge_idx = VertexToEdge[e];
					assert(Edges[edge_idx].t1 == -1);
					if (Edges[edge_idx].t1 == -1)
					{
						Edges[edge_idx].t1 = (int)index;
						TriangleRefCounts[index] += 1;
					}
				}

				tri_edge[j] = edge_idx;
			}

			AppendTriangleEx(tri.x, tri.y, tri.z, tri_edge.x, tri_edge.y, tri_edge.z);
			return index;
		}

		void SetVertex(int idx, const Vector3& val)
		{
			VectorSetSafe(VertexPositions, idx, val);
		}

		void SetVertexColor(int idx, const Vector3& val)
		{
			bHasVertexColor = true;
			VectorSetSafe(VertexColors, idx, val);
		}

		void SetVertexNormal(int idx, const Vector3& val)
		{
			bHasVertexNormals = true;
			VectorSetSafe(VertexNormals, idx, val);
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

		void SetTriangle(int idx, const Vector3i& val)
		{
			VectorSetSafe(Triangles, idx, val);
		}

		void SetTriangleEdge(int idx, const Vector3i& val)
		{
			VectorSetSafe(TriangleEdges, idx, val);
		}

		bool Simplify(float rate);

		void ApplyTransform(const Transform& trans, bool bReverseOrientationIfNeeded)
		{
			for (size_t i = 0; i < VertexPositions.size(); ++i)
			{
				Vector3& Position = VertexPositions[i];
				Position = trans.LocalToWorld(Position);
			}

			if (bHasVertexNormals)
			{
				for (size_t i = 0; i < VertexNormals.size(); ++i)
				{
					Vector3& Normal = VertexNormals[i];
					Normal = trans.LocalToWorldDirection(Normal);
				}
			}

			if (bReverseOrientationIfNeeded && trans.quat.ToRotationMatrix3().Determinant() < 0)
			{
				ReverseOrientation(false);
			}
		}

		void ApplyTransform(Transform3& trans, bool bReverseOrientationIfNeeded)
		{
			for (size_t i = 0; i < VertexPositions.size(); ++i)
			{
				Vector3& Position = VertexPositions[i];
				Position = trans.LocalToWorld(Position);
			}

			if (bHasVertexNormals)
			{
				for (size_t i = 0; i < VertexNormals.size(); ++i)
				{
					Vector3& Normal = VertexNormals[i];
					Normal = trans.LocalToWorldDirection(Normal);
				}
			}

			if (bReverseOrientationIfNeeded && trans.GetRotationMatrix().Determinant() < 0)
			{
				ReverseOrientation(false);
			}
		}

		void ReverseOrientation(bool bFlipNormals)
		{
			for (size_t i = 0; i < Triangles.size(); ++i)
			{
				Vector3i& tri = Triangles[i];
				std::swap(tri.y, tri.z);
			}

			if (bFlipNormals && bHasVertexNormals)
			{
				for (size_t i = 0; i < VertexNormals.size(); ++i)
				{
					Vector3& Normal = VertexNormals[i];
					Normal = -Normal;
				}
			}
		}

		inline Vector3 GetVertex(int idx) const
		{
			return VertexPositions[idx];
		}

		inline Vector3 GetVertexColor(int idx) const
		{
			if (!HasVertexColors())
			{
				return Vector3::Zero();
			}
			return VertexColors[idx];
		}

		inline Vector3 GetVertexNormal(int idx) const
		{
			if (!HasVertexNormals())
			{
				return Vector3::UnitY();
			}
			return VertexNormals[idx];
		}

		inline Vector2 GetVertexUV(int idx, int layer) const
		{
			return VertexUVs[layer][idx];
		}

		inline bool IsEdge(int idx) const
		{
			return 0 <= idx && idx < (int)Edges.size() && EdgeRefCounts[idx] > 0;
		}

		inline bool IsBoundaryEdge(int idx) const
		{
			return Edges[idx].t1 == -1;
		}
				
		inline bool IsTriangle(int idx) const
		{
			return 0 <= idx && idx < (int)Triangles.size() && TriangleRefCounts[idx] > 0;
		}

		inline Vector3i GetTriangle(int idx) const
		{
			return Triangles[idx];
		}

		inline Vector3i GetTriangleEdge(int idx) const
		{
			return TriangleEdges[idx];
		}

		Vector3 GetTriangleCentroid(int idx) const
		{
			const Vector3i ti = Triangles[idx];
			const Vector3 centroid = (VertexPositions[ti.x] + VertexPositions[ti.y] + VertexPositions[ti.z]) / 3.0f;
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
			const Vector3i ti = Triangles[idx];
			Box3 box(VertexPositions[ti.x], VertexPositions[ti.y], VertexPositions[ti.z]);
			return box;
		}

		void GetTriangleVertices(int idx, Vector3& a, Vector3& b, Vector3& c) const
		{
			const Vector3i ti = Triangles[idx];
			a = VertexPositions[ti.x];
			b = VertexPositions[ti.y];
			c = VertexPositions[ti.z];
		}

		VertexInfo GetTringleBaryPoint(int idx, float bary0, float bary1, float bary2) const
		{
			VertexInfo info;
			const Vector3i& t = Triangles[idx];
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

		int FindEdge(int v0, int v1) const
		{
			uint64_t key = Edge::Pack(v0, v1);
			auto it = VertexToEdge.find(key);
			if (it != VertexToEdge.end())
			{
				return it->second;
			}
			return -1;
 		}

		Vector2i GetEdgeV(int idx) const
		{
			return Vector2i(Edges[idx].v0, Edges[idx].v1);
		}

		void GetEdgeV(int idx, Vector3& a, Vector3& b) const
		{
			a = VertexPositions[Edges[idx].v0];
			b = VertexPositions[Edges[idx].v0];
		}

		Vector2i GetEdgeT(int idx) const
		{
			return Vector2i(Edges[idx].t0, Edges[idx].t1);
		}

		int GetTriangleEdge(int idx, int j) const
		{
			return TriangleEdges[idx][j];
		}

		bool HasVertexColors() const
		{
			return bHasVertexColor;
		}

		bool HasVertexNormals() const
		{
			return bHasVertexNormals;
		}

		bool HasVertexUVs() const
		{
			return VertexUVs.size() > 0;
		}

		bool HasAttributes() const
		{
			return Attributes.MaterialIDAttrib.size() > 0;
		}

		bool PokeTriangle(int TriangleID, const Vector3& BaryCoordinates, PokeTriangleInfo& PokeResult);
		bool SplitEdge(int eab, EdgeSplitInfo& SplitInfo, float split_t);

		void BuildBounds()
		{
			Bounds = Box3(VertexPositions.data(), VertexPositions.size());
		}

	private:

		int ReplaceEdgeTriangle(int idx, int t_old, int t_new)
		{
			Edge e = Edges[idx];
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
			Vector3i& Triangle = Triangles[idx];
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
			Vector3i& Triangle = TriangleEdges[idx];
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

		int AppendEdgeEx(int v0, int v1, int t0, int t1)
		{
			int index = (int)Edges.size();
			Edges.push_back({ v0, v1, t0, t1 });
			EdgeRefCounts.push_back(1);
			VertexRefCounts[v0] += 1;
			VertexRefCounts[v1] += 1;
			if (t0 >= 0)
			{
				TriangleRefCounts[t0] += 1;
			}
			if (t1 >= 0)
			{
				TriangleRefCounts[t1] += 1;
			}
			return index;
		}

		int AppendTriangleEx(int v0, int v1, int v2, int e0, int e1, int e2)
		{
			const Vector3i tri(v0, v1, v2);
			const Vector3i tri_edge(e0, e1, e2);

			int index = (int)Triangles.size();
			Triangles.push_back(tri);
			TriangleRefCounts.push_back(1);
			VertexRefCounts[tri.x] += 1;
			VertexRefCounts[tri.y] += 1;
			VertexRefCounts[tri.z] += 1;
			TriangleEdges.push_back(tri_edge);
			EdgeRefCounts[tri_edge.x] += 1;
			EdgeRefCounts[tri_edge.y] += 1;
			EdgeRefCounts[tri_edge.z] += 1;
			return index;
		}

	public:
		std::vector<Vector3>				VertexPositions;
		std::vector<Vector3>				VertexNormals;
		std::vector<Vector3>				VertexColors;
		std::vector<std::vector<Vector2>>	VertexUVs;
		std::vector<Vector3i>				Triangles;
		std::vector<Vector3i>				TriangleEdges;
		std::vector<Edge>					Edges;
		std::vector<uint8_t>				VertexRefCounts;
		std::vector<uint8_t>				EdgeRefCounts;
		std::vector<uint8_t>				TriangleRefCounts;
		std::map<uint64_t, int>				VertexToEdge;

		GeometryAttributeSet	Attributes;
		Box3					Bounds;
		Transform				Pose;

	private:
		std::string				ResourceName;
		bool					bHasVertexColor{ false };
		bool					bHasVertexNormals{ false };

		template<typename T>
		static void VectorSetSafe(std::vector<T>& Vertices, size_t index, const T& v)
		{
			if (index >= Vertices.size())
			{
				Vertices.resize(index + 1);
			}
			Vertices[index] = v;
		}

		template<typename T>
		static void VectorSet(std::vector<T>& Vertices, size_t index, const T& v)
		{
			Vertices[index] = v;
		}
	};

	class DynamicMeshAABBTree
	{
	public:
		DynamicMeshAABBTree(DynamicMesh* data);

		IntersectionsQueryResult FindAllIntersections(const DynamicMeshAABBTree& OtherTree, const Transform * TransformF = nullptr) const;

	private:
		void Build();
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

		void FindIntersections(
			int iBox, const DynamicMeshAABBTree& OtherTree, const Transform* TransformF,
			int oBox, int depth, IntersectionsQueryResult& result) const;

	private:
		DynamicMesh	 *Mesh { nullptr };
		std::vector<int> BoxToIndex;
		std::vector<Box3> AABB;
		std::vector<int> IndexList;
		int TrianglesEnd = -1;
		int RootIndex = -1;
	};
}	// namespace Riemann
