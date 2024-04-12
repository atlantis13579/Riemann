
#include <assert.h>
#include "DynamicMesh.h"
#include "MeshSimplification.h"
#include "../Core/SmallSet.h"
#include "../Maths/Box1.h"
#include "../Maths/Maths.h"
#include "../CollisionPrimitive/Triangle3.h"

namespace Riemann
{
	static char* ParseRow(char* buf, char* bufEnd, char* row, int len)
	{
		bool start = true;
		bool done = false;
		int n = 0;
		while (!done && buf < bufEnd)
		{
			char c = *buf++;
			switch (c)
			{
			case '\\':
			case '\r':
				break;
			case '\n':
				if (start) break;
				done = true;
				break;
			case '\t':
			case ' ':
				if (start) break;
			default:
				start = false;
				row[n++] = c;
				if (n >= len - 1)
					done = true;
				break;
			}
		}
		row[n] = '\0';
		return buf;
	}

	static int ParseFace(int NumVertices, char* row, int* data, int n)
	{
		int j = 0;
		while (*row != '\0')
		{
			while (*row != '\0' && (*row == ' ' || *row == '\t'))
				row++;
			char* s = row;
			while (*row != '\0' && *row != ' ' && *row != '\t')
			{
				if (*row == '/') *row = '\0';
				row++;
			}
			if (*s == '\0')
				continue;
			int vi = atoi(s);
			data[j++] = vi < 0 ? vi + NumVertices : vi - 1;
			if (j >= n) return j;
		}
		return j;
	}

	static int ParseNumTokens(const char* str)
	{
		const char* p = str;
		int tokens = 0;
		bool parsing_token = false;
		while (*p != '\n' && *p != 0)
		{
			char c = *p++;
			if (c == ' ' || c == '\t')
			{
				if (parsing_token)
				{
					parsing_token = false;
				}
			}
			else
			{
				if (!parsing_token)
				{
					tokens++;
					parsing_token = true;
				}
			}
		}
		return tokens;
	}

	DynamicMesh::DynamicMesh()
	{
	}

	DynamicMesh::~DynamicMesh()
	{
	}

	bool DynamicMesh::Simplify(float rate)
	{
		std::vector<Vector3> new_v;
		std::vector<int> new_i;

		if (!SimplifyMesh(GetVertexBuffer(), (const void*)GetIndexBuffer(), GetVertexCount(), true, GetTriangleCount(), rate, new_v, new_i))
		{
			return false;
		}

		Clear();

		VertexPositions = new_v;
		VertexRefCounts.resize(new_v.size(), 1);
		VertexEdgeLists.resize((int)new_v.size());

		for (size_t i = 0; i < new_i.size() / 3; ++i)
		{
			AppendTriangle(Index3(new_i[3 * i], new_i[3 * i + 1], new_i[3 * i + 2]));
		}

		BuildBounds();

		return true;
	}

	void DynamicMesh::ApplyTransform(const Transform& trans, bool bReverseOrientationIfNeeded)
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

	void DynamicMesh::ApplyTransform(Transform3& trans, bool bReverseOrientationIfNeeded)
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

	void DynamicMesh::ReverseOrientation(bool bFlipNormals)
	{
		for (size_t i = 0; i < Triangles.size(); ++i)
		{
			Index3& tri = Triangles[i];
			std::swap(tri.b, tri.c);
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

	std::vector<int> DynamicMesh::GetVexTriangles(int VertexID) const
	{
		SmallSet<int> s;
		for (int eid : VertexEdgeLists[VertexID])
		{
			const Edge e = Edges[eid];
			s.insert(e.t0);
			s.insert(e.t1);
		}
		return s.to_vector();
	}

	bool DynamicMesh::PokeTriangle(int TriangleID, const Vector3& BaryCoordinates, PokeTriangleInfo& PokeResult)
	{
		PokeResult = PokeTriangleInfo();

		if (!IsTriangle(TriangleID))
		{
			return false;
		}

		Index3 tv = GetTriangle(TriangleID);
		Index3 te = GetTriangleEdge(TriangleID);

		VertexInfo vinfo = GetTringleBaryPoint(TriangleID, BaryCoordinates[0], BaryCoordinates[1], BaryCoordinates[2]);
		int center = AppendVertex(vinfo);

		int eaC = AppendEdgeEx(tv[0], center, -1, -1);
		int ebC = AppendEdgeEx(tv[1], center, -1, -1);
		int ecC = AppendEdgeEx(tv[2], center, -1, -1);
		VertexRefCounts[tv[0]] += 1;
		VertexRefCounts[tv[1]] += 1;
		VertexRefCounts[tv[2]] += 1;
		VertexRefCounts[center] += 3;

		SetTriangle(TriangleID, Index3(tv[0], tv[1], center));
		SetTriangleEdge(TriangleID, Index3(te[0], ebC, eaC));

		int t1 = AppendTriangleEx(tv[1], tv[2], center, te[1], ecC, ebC);
		int t2 = AppendTriangleEx(tv[2], tv[0], center, te[2], eaC, ecC);

		ReplaceEdgeTriangle(te[1], TriangleID, t1);
		ReplaceEdgeTriangle(te[2], TriangleID, t2);

		SetEdgeTriangleEx(eaC, TriangleID, t2);
		SetEdgeTriangleEx(ebC, TriangleID, t1);
		SetEdgeTriangleEx(ecC, t1, t2);

		PokeResult.OriginalTriangle = TriangleID;
		PokeResult.TriVertices = tv;
		PokeResult.NewVertex = center;
		PokeResult.NewTriangles = Index2(t1, t2);
		PokeResult.NewEdges = Index3(eaC, ebC, ecC);
		PokeResult.BaryCoords = BaryCoordinates;

		if (HasAttributes())
		{
			Attributes.OnPokeTriangle(PokeResult);
		}

		return true;
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
	static int FindEdgeIndexInTri(T VertexID1, T VertexID2, const Vec& TriangleVerts)
	{
		if (SamePairUnordered(VertexID1, VertexID2, TriangleVerts[0], TriangleVerts[1])) return 0;
		if (SamePairUnordered(VertexID1, VertexID2, TriangleVerts[1], TriangleVerts[2])) return 1;
		if (SamePairUnordered(VertexID1, VertexID2, TriangleVerts[2], TriangleVerts[0])) return 2;
		return -1;
	}

	template<typename T, typename Vec>
	static int FindTriOrderedEdge(T VertexID1, T VertexID2, const Vec& TriangleVerts)
	{
		if (TriangleVerts[0] == VertexID1 && TriangleVerts[1] == VertexID2)	return 0;
		if (TriangleVerts[1] == VertexID1 && TriangleVerts[2] == VertexID2)	return 1;
		if (TriangleVerts[2] == VertexID1 && TriangleVerts[0] == VertexID2)	return 2;
		return -1;
	}

	template<typename T, typename Vec>
	static int OrientTriEdgeAndFindOtherVtx(T& Vertex1, T& Vertex2, const Vec& TriangleVerts)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (SamePairUnordered(Vertex1, Vertex2, TriangleVerts[j], TriangleVerts[(j + 1) % 3]))
			{
				Vertex1 = TriangleVerts[j];
				Vertex2 = TriangleVerts[(j + 1) % 3];
				return TriangleVerts[(j + 2) % 3];
			}
		}
		return -1;
	}

	template<typename T, typename Vec>
	inline int FindTriOtherVtx(T VertexID1, T VertexID2, const Vec& TriangleVerts)
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

	bool DynamicMesh::SplitEdge(int eab, EdgeSplitInfo& SplitInfo, float split_t)
	{
		SplitInfo = EdgeSplitInfo();

		if (!IsEdge(eab))
		{
			return false;
		}

		const Edge e = Edges[eab];
		int a = e.v0, b = e.v1;
		int t0 = e.t0;
		if (t0 == -1)
		{
			return false;
		}
		Index3 T0tv = GetTriangle(t0);
		int c = OrientTriEdgeAndFindOtherVtx(a, b, T0tv);

		if (VertexRefCounts[c] > 250)
		{
			assert(false);
			return false;
		}
		if (a != e.v0)
		{
			split_t = 1.0f - split_t;
		}

		SplitInfo.OriginalEdge = eab;
		SplitInfo.OriginalVertices = Index2(a, b);
		SplitInfo.OriginalTriangles = Index2(t0, -1);
		SplitInfo.SplitT = split_t;

		if (IsBoundaryEdge(eab))
		{
			Vector3 vNew = Maths::LinearInterp(GetVertex(a), GetVertex(b), split_t);
			int f = AppendVertex(vNew);
			if (HasVertexNormals())
			{
				SetVertexNormal(f, Maths::LinearInterp(GetVertexNormal(a), GetVertexNormal(b), split_t).Unit());
			}
			if (HasVertexColors())
			{
				SetVertexColor(f, Maths::LinearInterp(GetVertexColor(a), GetVertexColor(b), split_t));
			}
			for (int i = 0; i < (int)VertexUVs.size(); ++i)
			{
				SetVertexUV(f, Maths::LinearInterp(GetVertexUV(a, i), GetVertexUV(b, i), split_t), i);
			}

			Index3 T0te = GetTriangleEdge(t0);
			int ebc = T0te[FindEdgeIndexInTri(b, c, T0tv)];

			ReplaceTriangleVertex(t0, b, f);

			int t2 = AppendTriangleEx(f, b, c, -1, -1, -1);

			ReplaceEdgeTriangle(ebc, t0, t2);
			int eaf = eab;
			ReplaceEdgeVertex(eaf, b, f);
			VertexEdgeLists[b].remove(eab);
			VertexEdgeLists[f].push_back(eaf);

			int efb = AppendEdgeEx(f, b, t2, -1);
			int efc = AppendEdgeEx(f, c, t0, t2);

			ReplaceTriangleEdge(t0, ebc, efc);
			SetTriangleEdge(t2, Index3(efb, ebc, efc));

			VertexRefCounts[c] += 1;
			VertexRefCounts[f] += 2;

			SplitInfo.bIsBoundary = true;
			SplitInfo.OtherVertices = Index2(c, -1);
			SplitInfo.NewVertex = f;
			SplitInfo.NewEdges = Index3(efb, efc, -1);
			SplitInfo.NewTriangles = Index2(t2, -1);

			if (HasAttributes())
			{
				Attributes.OnSplitEdge(SplitInfo);
			}

			return true;

		}
		else
		{
			int t1 = Edges[eab].t1;
			SplitInfo.OriginalTriangles.b = t1;
			Index3 T1tv = GetTriangle(t1);
			int d = FindTriOtherVtx(a, b, T1tv);

			if (VertexRefCounts[d] > 250)
			{
				return false;
			}

			Vector3 vNew = Maths::LinearInterp(GetVertex(a), GetVertex(b), split_t);
			int f = AppendVertex(vNew);
			if (HasVertexNormals())
			{
				SetVertexNormal(f, Maths::LinearInterp(GetVertexNormal(a), GetVertexNormal(b), split_t).Unit());
			}
			if (HasVertexColors())
			{
				SetVertexColor(f, Maths::LinearInterp(GetVertexColor(a), GetVertexColor(b), split_t));
			}
			for (int i = 0; i < (int)VertexUVs.size(); ++i)
			{
				SetVertexUV(f, Maths::LinearInterp(GetVertexUV(a, i), GetVertexUV(b, i), split_t), i);
			}

			Index3 T0te = GetTriangleEdge(t0);
			int ebc = T0te[FindEdgeIndexInTri(b, c, T0tv)];
			Index3 T1te = GetTriangleEdge(t1);
			int edb = T1te[FindEdgeIndexInTri(d, b, T1tv)];

			ReplaceTriangleVertex(t0, b, f);
			ReplaceTriangleVertex(t1, b, f);

			int t2 = AppendTriangleEx(f, b, c, -1, -1, -1);
			int t3 = AppendTriangleEx(f, d, b, -1, -1, -1);

			ReplaceEdgeTriangle(ebc, t0, t2);
			ReplaceEdgeTriangle(edb, t1, t3);

			int eaf = eab;
			ReplaceEdgeVertex(eaf, b, f);

			VertexEdgeLists[b].remove(eab);
			VertexEdgeLists[f].push_back(eaf);

			int efb = AppendEdgeEx(f, b, t2, t3);
			int efc = AppendEdgeEx(f, c, t0, t2);
			int edf = AppendEdgeEx(d, f, t1, t3);

			ReplaceTriangleEdge(t0, ebc, efc);
			ReplaceTriangleEdge(t1, edb, edf);
			SetTriangleEdge(t2, Index3(efb, ebc, efc));
			SetTriangleEdge(t3, Index3(edf, edb, efb));

			VertexRefCounts[c] += 1;
			VertexRefCounts[d] += 1;
			VertexRefCounts[f] += 4;

			SplitInfo.bIsBoundary = false;
			SplitInfo.OtherVertices = Index2(c, d);
			SplitInfo.NewVertex = f;
			SplitInfo.NewEdges = Index3(efb, efc, edf);
			SplitInfo.NewTriangles = Index2(t2, t3);

			if (HasAttributes())
			{
				Attributes.OnSplitEdge(SplitInfo);
			}

			return true;
		}
	}

	EMeshResult DynamicMesh::CollapseEdge(int vKeep, int vRemove, float collapse_t, EdgeCollapseInfo& CollapseInfo)
	{
		CollapseInfo = EdgeCollapseInfo();

		if (IsVertex(vKeep) == false || IsVertex(vRemove) == false)
		{
			return EMeshResult::Failed_NotAnEdge;
		}

		int b = vKeep;		// renaming for sanity. We remove a and keep b
		int a = vRemove;

		int eab = FindEdge(a, b);
		if (eab == -1)
		{
			return EMeshResult::Failed_NotAnEdge;
		}

		const Edge EdgeAB = Edges[eab];
		int t0 = EdgeAB.v0;
		if (t0 == -1)
		{
			return EMeshResult::Failed_BrokenTopology;
		}
		Index3 T0tv = GetTriangle(t0);
		int c = FindTriOtherVtx(a, b, T0tv);

		// look up opposing triangle/vtx if we are not in boundary case
		bool bIsBoundaryEdge = false;
		int d = -1;
		int t1 = EdgeAB.v1;
		if (t1 != -1)
		{
			Index3 T1tv = GetTriangle(t1);
			d = FindTriOtherVtx(a, b, T1tv);
			if (c == d)
			{
				return EMeshResult::Failed_FoundDuplicateTriangle;
			}
		}
		else
		{
			bIsBoundaryEdge = true;
		}

		CollapseInfo.OpposingVerts = Index2(c, d);

		// We cannot collapse if edge lists of a and b share vertices other
		//  than c and d  (because then we will make a triangle [x b b].
		//  Unfortunately I cannot see a way to do this more efficiently than brute-force search
		//  [TODO] if we had tri iterator for a, couldn't we check each tri for b  (skipping t0 and t1) ?

		/*
		int edges_a_count = VertexEdgeLists.GetCount(a);
		int eac = InvalidID, ead = InvalidID, ebc = InvalidID, ebd = InvalidID;
		for (int eid_a : VertexEdgeLists.Values(a))
		{
			int vax = GetOtherEdgeVertex(eid_a, a);
			if (vax == c)
			{
				eac = eid_a;
				continue;
			}
			if (vax == d)
			{
				ead = eid_a;
				continue;
			}
			if (vax == b)
			{
				continue;
			}
			for (int eid_b : VertexEdgeLists.Values(b))
			{
				if (GetOtherEdgeVertex(eid_b, b) == vax)
				{
					return EMeshResult::Failed_InvalidNeighbourhood;
				}
			}
		}

		// I am not sure this tetrahedron case will detect bowtie vertices.
		// But the single-triangle case does

		// We cannot collapse if we have a tetrahedron. In this case a has 3 nbr edges,
		//  and edge cd exists. But that is not conclusive, we also have to check that
		//  cd is an internal edge, and that each of its tris contain a or b
		if (edges_a_count == 3 && bIsBoundaryEdge == false)
		{
			int edc = FindEdge(d, c);
			if (edc != InvalidID)
			{
				const FEdge EdgeDC = Edges[edc];
				if (EdgeDC.Tri[1] != InvalidID)
				{
					int edc_t0 = EdgeDC.Tri[0];
					int edc_t1 = EdgeDC.Tri[1];

					if ((TriangleHasVertex(edc_t0, a) && TriangleHasVertex(edc_t1, b))
						|| (TriangleHasVertex(edc_t0, b) && TriangleHasVertex(edc_t1, a)))
					{
						return EMeshResult::Failed_CollapseTetrahedron;
					}
				}
			}
		}
		else if (bIsBoundaryEdge == true && IsBoundaryEdge(eac))
		{
			// Cannot collapse edge if we are down to a single triangle
			ebc = FindEdgeFromTri(b, c, t0);
			if (IsBoundaryEdge(ebc))
			{
				return EMeshResult::Failed_CollapseTriangle;
			}
		}

		// TODO: it's unclear how this case would ever be encountered; check if it is needed
		//
		// cannot collapse an edge where both vertices are boundary vertices
		// because that would create a bowtie
		//
		// NOTE: potentially scanning all edges here...couldn't we
		//  pick up eac/bc/ad/bd as we go? somehow?
		if (bIsBoundaryEdge == false && IsBoundaryVertex(a) && IsBoundaryVertex(b))
		{
			return EMeshResult::Failed_InvalidNeighbourhood;
		}

		// save vertex positions before we delete removed (can defer kept?)
		FVector3d KeptPos = GetVertex(vKeep);
		FVector3d RemovedPos = GetVertex(vRemove);
		FVector2f RemovedUV;
		if (HasVertexUVs())
		{
			RemovedUV = GetVertexUV(vRemove);
		}
		FVector3f RemovedNormal;
		if (HasVertexNormals())
		{
			RemovedNormal = GetVertexNormal(vRemove);
		}
		FVector3f RemovedColor;
		if (HasVertexColors())
		{
			RemovedColor = GetVertexColor(vRemove);
		}

		// 1) remove edge ab from vtx b
		// 2) find edges ad and ac, and tris tad, tac across those edges  (will use later)
		// 3) for other edges, replace a with b, and add that edge to b
		// 4) replace a with b in all triangles connected to a
		int tad = InvalidID, tac = InvalidID;
		for (int eid : VertexEdgeLists.Values(a))
		{
			int o = GetOtherEdgeVertex(eid, a);
			if (o == b)
			{
				if (VertexEdgeLists.Remove(b, eid) != true)
				{
					checkfSlow(false, TEXT("FDynamicMesh3::CollapseEdge: failed at remove case o == b"));
					return EMeshResult::Failed_UnrecoverableError;
				}
			}
			else if (o == c)
			{
				if (VertexEdgeLists.Remove(c, eid) != true)
				{
					checkfSlow(false, TEXT("FDynamicMesh3::CollapseEdge: failed at remove case o == c"));
					return EMeshResult::Failed_UnrecoverableError;
				}
				tac = GetOtherEdgeTriangle(eid, t0);
			}
			else if (o == d)
			{
				if (VertexEdgeLists.Remove(d, eid) != true)
				{
					checkfSlow(false, TEXT("FDynamicMesh3::CollapseEdge: failed at remove case o == c, step 1"));
					return EMeshResult::Failed_UnrecoverableError;
				}
				tad = GetOtherEdgeTriangle(eid, t1);
			}
			else
			{
				if (ReplaceEdgeVertex(eid, a, b) == -1)
				{
					checkfSlow(false, TEXT("FDynamicMesh3::CollapseEdge: failed at remove case else"));
					return EMeshResult::Failed_UnrecoverableError;
				}
				VertexEdgeLists.Insert(b, eid);
			}

			// [TODO] perhaps we can already have unique tri list because of the manifold-nbrhood check we need to do...
			const FEdge Edge = Edges[eid];
			for (int j = 0; j < 2; ++j)
			{
				int t_j = Edge.Tri[j];
				if (t_j != InvalidID && t_j != t0 && t_j != t1)
				{
					if (TriangleHasVertex(t_j, a))
					{
						if (ReplaceTriangleVertex(t_j, a, b) == -1)
						{
							checkfSlow(false, TEXT("FDynamicMesh3::CollapseEdge: failed at remove last check"));
							return EMeshResult::Failed_UnrecoverableError;
						}
						VertexRefCounts.Increment(b);
						VertexRefCounts.Decrement(a);
					}
				}
			}
		}

		if (bIsBoundaryEdge == false)
		{
			// remove all edges from vtx a, then remove vtx a
			VertexEdgeLists.Clear(a);
			checkSlow(VertexRefCounts.GetRefCount(a) == 3);		// in t0,t1, and initial ref
			VertexRefCounts.Decrement(a, 3);
			checkSlow(VertexRefCounts.IsValid(a) == false);

			// remove triangles T0 and T1, and update b/c/d refcounts
			TriangleRefCounts.Decrement(t0);
			TriangleRefCounts.Decrement(t1);
			VertexRefCounts.Decrement(c);
			VertexRefCounts.Decrement(d);
			VertexRefCounts.Decrement(b, 2);
			checkSlow(TriangleRefCounts.IsValid(t0) == false);
			checkSlow(TriangleRefCounts.IsValid(t1) == false);

			// remove edges ead, eab, eac
			EdgeRefCounts.Decrement(ead);
			EdgeRefCounts.Decrement(eab);
			EdgeRefCounts.Decrement(eac);
			checkSlow(EdgeRefCounts.IsValid(ead) == false);
			checkSlow(EdgeRefCounts.IsValid(eab) == false);
			checkSlow(EdgeRefCounts.IsValid(eac) == false);

			// replace t0 and t1 in edges ebd and ebc that we kept
			ebd = FindEdgeFromTri(b, d, t1);
			if (ebc == InvalidID)   // we may have already looked this up
			{
				ebc = FindEdgeFromTri(b, c, t0);
			}

			if (ReplaceEdgeTriangle(ebd, t1, tad) == -1)
			{
				checkfSlow(false, TEXT("FDynamicMesh3::CollapseEdge: failed at isboundary=false branch, ebd replace triangle"));
				return EMeshResult::Failed_UnrecoverableError;
			}

			if (ReplaceEdgeTriangle(ebc, t0, tac) == -1)
			{
				checkfSlow(false, TEXT("FDynamicMesh3::CollapseEdge: failed at isboundary=false branch, ebc replace triangle"));
				return EMeshResult::Failed_UnrecoverableError;
			}

			// update tri-edge-nbrs in tad and tac
			if (tad != InvalidID)
			{
				if (ReplaceTriangleEdge(tad, ead, ebd) == -1)
				{
					checkfSlow(false, TEXT("FDynamicMesh3::CollapseEdge: failed at isboundary=false branch, ebd replace triangle"));
					return EMeshResult::Failed_UnrecoverableError;
				}
			}
			if (tac != InvalidID)
			{
				if (ReplaceTriangleEdge(tac, eac, ebc) == -1)
				{
					checkfSlow(false, TEXT("FDynamicMesh3::CollapseEdge: failed at isboundary=false branch, ebd replace triangle"));
					return EMeshResult::Failed_UnrecoverableError;
				}
			}

		}
		else
		{
			//  boundary-edge path. this is basically same code as above, just not referencing t1/d

			// remove all edges from vtx a, then remove vtx a
			VertexEdgeLists.Clear(a);
			checkSlow(VertexRefCounts.GetRefCount(a) == 2);		// in t0 and initial ref
			VertexRefCounts.Decrement(a, 2);
			checkSlow(VertexRefCounts.IsValid(a) == false);

			// remove triangle T0 and update b/c refcounts
			TriangleRefCounts.Decrement(t0);
			VertexRefCounts.Decrement(c);
			VertexRefCounts.Decrement(b);
			checkSlow(TriangleRefCounts.IsValid(t0) == false);

			// remove edges eab and eac
			EdgeRefCounts.Decrement(eab);
			EdgeRefCounts.Decrement(eac);
			checkSlow(EdgeRefCounts.IsValid(eab) == false);
			checkSlow(EdgeRefCounts.IsValid(eac) == false);

			// replace t0 in edge ebc that we kept
			ebc = FindEdgeFromTri(b, c, t0);
			if (ReplaceEdgeTriangle(ebc, t0, tac) == -1)
			{
				checkfSlow(false, TEXT("FDynamicMesh3::CollapseEdge: failed at isboundary=false branch, ebc replace triangle"));
				return EMeshResult::Failed_UnrecoverableError;
			}

			// update tri-edge-nbrs in tac
			if (tac != InvalidID)
			{
				if (ReplaceTriangleEdge(tac, eac, ebc) == -1)
				{
					checkfSlow(false, TEXT("FDynamicMesh3::CollapseEdge: failed at isboundary=true branch, ebd replace triangle"));
					return EMeshResult::Failed_UnrecoverableError;
				}
			}
		}

		// set kept vertex to interpolated collapse position
		SetVertex(vKeep, Lerp(KeptPos, RemovedPos, collapse_t));
		if (HasVertexUVs())
		{
			SetVertexUV(vKeep, Lerp(GetVertexUV(vKeep), RemovedUV, (float)collapse_t));
		}
		if (HasVertexNormals())
		{
			SetVertexNormal(vKeep, Normalized(Lerp(GetVertexNormal(vKeep), RemovedNormal, (float)collapse_t)));
		}
		if (HasVertexColors())
		{
			SetVertexColor(vKeep, Lerp(GetVertexColor(vKeep), RemovedColor, (float)collapse_t));
		}

		CollapseInfo.KeptVertex = vKeep;
		CollapseInfo.RemovedVertex = vRemove;
		CollapseInfo.bIsBoundary = bIsBoundaryEdge;
		CollapseInfo.CollapsedEdge = eab;
		CollapseInfo.RemovedTris = FIndex2i(t0, t1);
		CollapseInfo.RemovedEdges = FIndex2i(eac, ead);
		CollapseInfo.KeptEdges = FIndex2i(ebc, ebd);
		CollapseInfo.CollapseT = collapse_t;

		if (HasAttributes())
		{
			Attributes()->OnCollapseEdge(CollapseInfo);
		}
		*/
		return EMeshResult::Ok;
	}

	int DynamicMesh::GetVtxTriangleCount(int vID) const
	{
		if (!IsVertex(vID))
		{
			return -1;
		}
		int N = 0;

		for (auto it = VertexEdgeLists[vID].begin(); it != VertexEdgeLists[vID].end(); ++it)
		{
			int eid = *it;
			const Edge e = Edges[eid];
			const int vOther = e.v0 == vID ? e.v1 : e.v0;
			if (TriHasSequentialVertices(e.t0, vID, vOther))
			{
				N++;
			}
			const int et1 = e.t1;
			if (e.t1 != -1 && TriHasSequentialVertices(e.t1, vID, vOther))
			{
				N++;
			}
		}

		return N;
	}

	int DynamicMesh::GetVtxSingleTriangle(int VertexID) const
	{
		if (!IsVertex(VertexID))
		{
			return -1;
		}

		for (int EID : VertexEdgeLists[VertexID])
		{
			return Edges[EID].t0;
		}

		return -1;
	}


	void DynamicMesh::BuildBounds()
	{
		Bounds = Box3(VertexPositions.data(), VertexPositions.size());
	}

	bool DynamicMesh::LoadObj(const char* name)
	{
		char* buf = 0;
		FILE* fp = fopen(name, "rb");
		if (!fp)
		{
			return false;
		}
		fseek(fp, 0, SEEK_END);
		size_t bufSize = (size_t)ftell(fp);
		fseek(fp, 0, SEEK_SET);
		buf = new char[bufSize];
		if (!buf)
		{
			fclose(fp);
			return false;
		}
		uint64_t readLen = fread(buf, bufSize, 1, fp);
		fclose(fp);

		if (readLen != 1)
		{
			delete[] buf;
			return false;
		}

		int VectexTokens = 0;

		char* p = buf, * pEnd = buf + bufSize;
		while (p < pEnd)
		{
			char row[512];
			row[0] = '\0';
			p = ParseRow(p, pEnd, row, sizeof(row) / sizeof(char));
			if (row[0] == '#') continue;
			if (row[0] == 'v' && row[1] != 'n' && row[1] != 't')
			{
				if (VectexTokens == 0)
				{
					VectexTokens = ParseNumTokens(row + 1);
				}

				if (VectexTokens == 6)
				{
					Vector3 v, c;
					sscanf(row + 1, "%f %f %f %f %f %f", &v.x, &v.y, &v.z, &c.x, &c.y, &c.z);
					VertexPositions.push_back(v);
					VertexColors.push_back(c);
					bHasVertexColor = true;
				}
				else
				{
					Vector3 v;
					sscanf(row + 1, "%f %f %f", &v.x, &v.y, &v.z);
					VertexPositions.push_back(v);
				}
				VertexRefCounts.push_back(1);
				VertexEdgeLists.append();
			}
			else if (row[0] == 'v' && row[1] == 'n')
			{
				Vector3 v;
				sscanf(row + 2, "%f %f %f", &v.x, &v.y, &v.z);
				VertexNormals.push_back(v);
				bHasVertexNormals = true;
			}
			else if (row[0] == 'v' && row[1] == 't')
			{
				Vector2 v;
				sscanf(row + 2, "%f %f", &v.x, &v.y);
				if (VertexUVs.empty())
					VertexUVs.resize(1);
				VertexUVs[0].push_back(v);
			}
			else if (row[0] == 'f')
			{
				int face[32];
				int NumVertices = (int)VertexPositions.size();
				int nv = ParseFace(NumVertices, row + 1, face, 32);
				for (int i = 2; i < nv; ++i)
				{
					const int a = face[0];
					const int b = face[i - 1];
					const int c = face[i];
					if (a < 0 || a >= NumVertices || b < 0 || b >= NumVertices || c < 0 || c >= NumVertices)
						continue;
					AppendTriangle(Index3(a, b, c));
				}
			}
		}

		delete[] buf;

		BuildBounds();

		ResourceName = name;
		return true;
	}

	bool DynamicMesh::ExportObj(const char* name)
	{
		FILE* fp = fopen(name, "w");
		if (!fp)
		{
			return false;
		}

		fprintf(fp, "# %d vertices, %d faces\n", (int)VertexPositions.size(), (int)Triangles.size());

		fprintf(fp, "# List of geometric vertices, with (x,y,z[,w]) coordinates, w is optional and defaults to 1.0. \n");
		if (!VertexColors.empty())
		{
			for (size_t i = 0; i < VertexPositions.size(); ++i)
			{
				const Vector3 v = VertexPositions[i];
				const Vector3 c = VertexColors[i];
				fprintf(fp, "v %.3f %.3f %.3f %.3f %.3f %.3f\n", v.x, v.y, v.z, c.x, c.y, c.z);
			}
		}
		else
		{
			for (size_t i = 0; i < VertexPositions.size(); ++i)
			{
				const Vector3 v = VertexPositions[i];
				fprintf(fp, "v %.3f %.3f %.3f\n", v.x, v.y, v.z);
			}
		}

		fprintf(fp, "# Polygonal face element (see below) \n");
		for (size_t i = 0; i < Triangles.size(); ++i)
		{
			const Index3 v = Triangles[i];
			fprintf(fp, "f %d %d %d\n", v.a + 1, v.b + 1, v.c + 1);
		}

		if (!VertexNormals.empty())
		{
			fprintf(fp, "# List of vertex normals in (x,y,z) form; normals might not be unit vectors. \n");
			for (size_t i = 0; i < VertexNormals.size(); ++i)
			{
				const Vector3 v = VertexNormals[i];
				fprintf(fp, "vn %.3f %.3f %.3f\n", v.x, v.y, v.z);
			}
		}

		if (!VertexUVs.empty())
		{
			fprintf(fp, "# List of texture coordinates, in (u, v [,w]) coordinates, these will vary between 0 and 1, w is optional and defaults to 0. \n");
			for (size_t i = 0; i < VertexUVs[0].size(); ++i)
			{
				const Vector2 v = VertexUVs[0][i];
				fprintf(fp, "vt %.3f %.3f\n", v.x, v.y);
			}
		}

		fclose(fp);
		return true;
	}

	DynamicMesh* DynamicMesh::CreateFromObj(const char* name)
	{
		DynamicMesh *p = new DynamicMesh;
		if (!p->LoadObj(name))
		{
			delete p;
			return nullptr;
		}
		return p;
	}

	DynamicMeshAABBTree::DynamicMeshAABBTree(DynamicMesh* data)
	{
		Mesh = data;
		Build();
	}

	IntersectionsQueryResult DynamicMeshAABBTree::FindAllIntersections(const DynamicMeshAABBTree& OtherTree, const Transform* TransformF) const
	{
		IntersectionsQueryResult result;
		FindIntersections(RootIndex, OtherTree, TransformF, OtherTree.RootIndex, 0, result);
		return result;
	}

	static void AddTriTriIntersectionResult(Triangle3::Triangle3IntersectionResult& Intr, int TID_A, int TID_B, IntersectionsQueryResult& Result)
	{
		if (Intr.Quantity == 1)
		{
			Result.Points.push_back(IntersectionsQueryResult::PointIntersection{ {TID_A, TID_B}, Intr.Points[0] });
		}
		else if (Intr.Quantity == 2)
		{
			Result.Segments.push_back(IntersectionsQueryResult::SegmentIntersection{ {TID_A, TID_B}, {Intr.Points[0], Intr.Points[1]} });
		}
		else if (Intr.Quantity > 2)
		{
			if (Intr.Type == IntersectionType::MultiSegment)
			{
				Result.Segments.push_back(IntersectionsQueryResult::SegmentIntersection{ {TID_A, TID_B}, {Intr.Points[0], Intr.Points[1]} });
				Result.Segments.push_back(IntersectionsQueryResult::SegmentIntersection{ {TID_A, TID_B}, {Intr.Points[2], Intr.Points[3]} });
				if (Intr.Quantity > 4)
				{
					Result.Segments.push_back(IntersectionsQueryResult::SegmentIntersection{ {TID_A, TID_B}, {Intr.Points[4], Intr.Points[5]} });
				}
			}
			else
			{
				Result.Polygons.push_back(IntersectionsQueryResult::PolygonIntersection{ {TID_A, TID_B},
					{Intr.Points[0], Intr.Points[1], Intr.Points[2], Intr.Points[3], Intr.Points[4], Intr.Points[5]}, Intr.Quantity });
			}
		}
	}

	void DynamicMeshAABBTree::FindIntersections(int iBox, const DynamicMeshAABBTree& OtherTree, const Transform* TransformF, int oBox, int depth, IntersectionsQueryResult& result) const
	{
		int idx = BoxToIndex[iBox];
		int odx = OtherTree.BoxToIndex[oBox];

		if (idx < TrianglesEnd && odx < OtherTree.TrianglesEnd)
		{
			Triangle3 Tri, otri;
			int num_tris = IndexList[idx], onum_tris = OtherTree.IndexList[odx];

			for (int j = 1; j <= onum_tris; ++j)
			{
				int tj = OtherTree.IndexList[odx + j];
				OtherTree.Mesh->GetTriangleVertices(tj, otri.v0, otri.v1, otri.v2);
				if (TransformF != nullptr)
				{
					otri.v0 = TransformF->LocalToWorld(otri.v0);
					otri.v1 = TransformF->LocalToWorld(otri.v1);
					otri.v2 = TransformF->LocalToWorld(otri.v2);
				}

				for (int i = 1; i <= num_tris; ++i)
				{
					int ti = IndexList[idx + i];
					Mesh->GetTriangleVertices(ti, Tri.v0, Tri.v1, Tri.v2);

					Triangle3::Triangle3IntersectionResult intr;

					if (otri.CalculateIntersectionTriangle3(Tri, intr))
					{
						AddTriTriIntersectionResult(intr, ti, tj, result);
					}
				}
			}

			return;
		}

		bool bDescendOther = (idx < TrianglesEnd || depth % 2 == 0);
		if (bDescendOther && odx < OtherTree.TrianglesEnd)
			bDescendOther = false;

		if (bDescendOther)
		{
			Box3 bounds = AABB[iBox];
			bounds.Thicken(1e-3f);

			int oChild1 = OtherTree.IndexList[odx];
			if (oChild1 < 0)
			{
				oChild1 = (-oChild1) - 1;
				Box3 oChild1Box = OtherTree.GetAABB(oChild1, TransformF);
				if (oChild1Box.Intersect(bounds))
					FindIntersections(iBox, OtherTree, TransformF, oChild1, depth + 1, result);

			}
			else
			{
				oChild1 = oChild1 - 1;

				Box3 oChild1Box = OtherTree.GetAABB(oChild1, TransformF);
				if (oChild1Box.Intersect(bounds))
				{
					FindIntersections(iBox, OtherTree, TransformF, oChild1, depth + 1, result);
				}

				int oChild2 = OtherTree.IndexList[odx + 1] - 1;
				Box3 oChild2Box = OtherTree.GetAABB(oChild2, TransformF);
				if (oChild2Box.Intersect(bounds))
				{
					FindIntersections(iBox, OtherTree, TransformF, oChild2, depth + 1, result);
				}
			}
		}
		else
		{
			Box3 oBounds = OtherTree.GetAABB(oBox, TransformF);

			int iChild1 = IndexList[idx];
			if (iChild1 < 0)
			{
				iChild1 = (-iChild1) - 1;
				if (AABB[iChild1].Intersect(oBounds))
				{
					FindIntersections(iChild1, OtherTree, TransformF, oBox, depth + 1, result);
				}

			}
			else
			{
				iChild1 = iChild1 - 1;
				if (AABB[iChild1].Intersect(oBounds))
				{
					FindIntersections(iChild1, OtherTree, TransformF, oBox, depth + 1, result);
				}

				int iChild2 = IndexList[idx + 1] - 1;
				if (AABB[iChild2].Intersect(oBounds))
				{
					FindIntersections(iChild2, OtherTree, TransformF, oBox, depth + 1, result);
				}
			}
		}
	}

	Box3 DynamicMeshAABBTree::GetAABB(int idx, const Transform* TransformF) const
	{
		Box3 box = AABB[idx];
		if (TransformF)
		{
			box = Box3::Transform(box, TransformF->pos, TransformF->quat);
		}
		return box;
	}

	void DynamicMeshAABBTree::Build()
	{
		std::vector<int> Triangles;
		std::vector<Vector3> Centers;
		for (int i = 0 ; i < Mesh->Triangles.size(); ++i)
		{
			const Vector3 centroid = Mesh->GetTriangleCentroid((int)i);
			float d2 = centroid.SquareLength();
			if (d2 > 1e-3f)
			{
				Triangles.push_back(i);
				Centers.push_back(centroid);
			}
		}

		Build(Triangles, Centers);
	}

	void DynamicMeshAABBTree::Build(std::vector<int>& Triangles, std::vector<Vector3>& Centers)
	{
		FBoxesSet Tris;
		FBoxesSet Nodes;
		Box3 rootBox;
		const int TopDownLeafMaxTriCount = 3;
		int rootnode = SplitTriSetMidpoint(Triangles, Centers, 0, (int)Triangles.size(), 0, TopDownLeafMaxTriCount, Tris, Nodes, rootBox);

		BoxToIndex = Tris.BoxToIndex;
		AABB = Tris.AABB;
		IndexList = Tris.IndexList;
		TrianglesEnd = Tris.IIndicesCur;
		int iIndexShift = TrianglesEnd;
		int iBoxShift = Tris.IBoxCur;

		for (int i = 0; i < Nodes.IBoxCur; ++i)
		{
			const Box3& box = Nodes.AABB[i];
			AABB.insert(AABB.begin() + iBoxShift + i, box);
			int NodeBoxIndex = Nodes.BoxToIndex[i];
			BoxToIndex.insert(BoxToIndex.begin() + iBoxShift + i, iIndexShift + NodeBoxIndex);
		}

		for (int i = 0; i < Nodes.IIndicesCur; ++i)
		{
			int child_box = Nodes.IndexList[i];
			if (child_box < 0)
			{
				child_box = (-child_box) - 1;
			}
			else
			{
				child_box += iBoxShift;
			}
			child_box = child_box + 1;
			IndexList.insert(IndexList.begin() + iIndexShift + i, child_box);
		}

		RootIndex = rootnode + iBoxShift;
	}

	int DynamicMeshAABBTree::SplitTriSetMidpoint(std::vector<int>& Triangles, std::vector<Vector3>& Centers, int IStart, int ICount, int Depth, int MinTriCount, FBoxesSet& Tris, FBoxesSet& Nodes, Box3& Box)
	{
		Box = (Triangles.size() > 0) ?
			Box3::Empty() : Box3(Vector3::Zero(), 0.0);
		int IBox = -1;

		if (ICount <= MinTriCount)
		{
			IBox = Tris.IBoxCur++;
			Tris.BoxToIndex.insert(Tris.BoxToIndex.begin() + IBox, Tris.IIndicesCur);
			Tris.IndexList.insert(Tris.IndexList.begin() + (Tris.IIndicesCur++), ICount);
			for (int i = 0; i < ICount; ++i)
			{
				Tris.IndexList.insert(Tris.IndexList.begin() + (Tris.IIndicesCur++), Triangles[IStart + i]);
				Box.Encapsulate(Mesh->GetTriangleBounds(Triangles[IStart + i]));
			}

			Tris.AABB.insert(Tris.AABB.begin() + IBox, Box);

			return -(IBox + 1);
		}

		int axis = (Depth % 3);
		Box1 interval = Box1::Empty();
		for (int i = 0; i < ICount; ++i)
		{
			interval.Encapsulate(Centers[IStart + i][axis]);
		}
		float midpoint = interval.GetCenter();

		int n0, n1;
		if (interval.GetLength() > 1e-3f)
		{
			int l = 0;
			int r = ICount - 1;
			while (l < r)
			{
				while (Centers[IStart + l][axis] <= midpoint)
				{
					l++;
				}
				while (Centers[IStart + r][axis] > midpoint)
				{
					r--;
				}
				if (l >= r)
				{
					break;
				}
				Vector3 tmpc = Centers[IStart + l];
				Centers[IStart + l] = Centers[IStart + r];
				Centers[IStart + r] = tmpc;
				int tmpt = Triangles[IStart + l];
				Triangles[IStart + l] = Triangles[IStart + r];
				Triangles[IStart + r] = tmpt;
			}

			n0 = l;
			n1 = ICount - n0;
			assert(n0 >= 1 && n1 >= 1);
		}
		else
		{
			n0 = ICount / 2;
			n1 = ICount - n0;
		}

		Box3 box1;
		int child0 = SplitTriSetMidpoint(Triangles, Centers, IStart, n0, Depth + 1, MinTriCount, Tris, Nodes, Box);
		int child1 = SplitTriSetMidpoint(Triangles, Centers, IStart + n0, n1, Depth + 1, MinTriCount, Tris, Nodes, box1);
		Box.Encapsulate(box1);

		IBox = Nodes.IBoxCur++;
		Nodes.BoxToIndex.insert(Nodes.BoxToIndex.begin() + IBox, Nodes.IIndicesCur);
		Nodes.IndexList.insert(Nodes.IndexList.begin() + Nodes.IIndicesCur++, child0);
		Nodes.IndexList.insert(Nodes.IndexList.begin() + Nodes.IIndicesCur++, child1);

		Nodes.AABB.insert(Nodes.AABB.begin() + IBox, Box);

		return IBox;
	}


}
