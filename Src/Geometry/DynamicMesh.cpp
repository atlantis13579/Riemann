#include <assert.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include "DynamicMesh.h"
#include "MeshSimplification.h"
#include "../Core/Base.h"
#include "../Core/BitSet.h"
#include "../Core/File.h"
#include "../Core/SmallSet.h"
#include "../Maths/Box1.h"
#include "../Maths/Box2.h"
#include "../Maths/Maths.h"
#include "../CollisionPrimitive/Triangle3.h"

namespace Riemann
{
	constexpr static int InvalidID = -1;
	constexpr static int NonManifoldID = -2;
	constexpr static int DuplicateTriangleID = -3;

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

	static int ParseFace(int NumVertices, char* row, int* v, int* vt, int* vn, int n)
	{
		int num = 0, pos = 0;
		char buf[32];
		while (*row != '\0')
		{
			while (*row != '\0' && (*row == ' ' || *row == '\t'))
			{
				pos = 0;
				row++;
			}
			char* s = buf;
			bool slash = false;
			while (*row != '\0' && *row != ' ' && *row != '\t')
			{
				if (*row == '/')
				{
					slash = true;
					row++;
					break;
				}
				*s++ = *row++;
			}
			if (s == buf)
				continue;
			*s = '\0';

			int value = atoi(buf);
			int *pv = (pos == 0) ? v : ((pos == 1) ? vt : vn);
			pv[num] = value < 0 ? value + NumVertices : value - 1;

			if (slash)
			{
				pos++;
				assert(pos <= 2);
				continue;
			}

			num++;
			if (num >= n)
			{
				assert(false);
				return num;
			}
		}
		return num;
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

	static inline int FindTriOrientation(const Index3& tri1, const Index3& tr12)
	{
		if (FindTriOrderedEdge(tri1[0], tri1[1], tr12) >= 0)
			return -1;
		if (FindTriOrderedEdge(tri1[1], tri1[2], tr12) >= 0)
			return -1;
		if (FindTriOrderedEdge(tri1[2], tri1[0], tr12) >= 0)
			return -1;

		if (FindTriOrderedEdge(tri1[1], tri1[0], tr12) >= 0)
			return 1;
		if (FindTriOrderedEdge(tri1[2], tri1[1], tr12) >= 0)
			return 1;
		if (FindTriOrderedEdge(tri1[0], tri1[2], tr12) >= 0)
			return 1;

		return 0;
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
	inline int FindTriOtherVtxUnsafe(T VertexID1, T VertexID2, const Vec& TriangleVerts)
	{
		if (TriangleVerts[0] == VertexID1)
		{
			return (TriangleVerts[1] == VertexID2) ? TriangleVerts[2] : TriangleVerts[1];
		}
		else if (TriangleVerts[1] == VertexID1)
		{
			return (TriangleVerts[0] == VertexID2) ? TriangleVerts[2] : TriangleVerts[0];
		}
		else
		{
			return (TriangleVerts[0] == VertexID2) ? TriangleVerts[1] : TriangleVerts[0];
		}
	}

	void FMeshIndexMappings::Initialize(DynamicMesh* Mesh)
	{
		if (Mesh->HasAttributes())
		{
			DynamicMeshAttributeSet* Attribs = Mesh->Attributes();
			UVMaps.resize(Attribs->NumUVLayers());
			NormalMaps.resize(Attribs->NumNormalLayers());
		}
	}

	DynamicMesh::DynamicMesh() : mAttributes(new DynamicMeshAttributeSet(this))
	{
	}

	DynamicMesh::DynamicMesh(const DynamicMesh& rhs) : mAttributes(nullptr)
	{
		*this = rhs;
	}

	DynamicMesh& DynamicMesh::operator=(const DynamicMesh& rhs)
	{
		if (this == &rhs)
		{
			return *this;
		}

		VertexPositions = rhs.VertexPositions;
		VertexNormals = rhs.VertexNormals;
		VertexColors = rhs.VertexColors;
		VertexUVs = rhs.VertexUVs;
		Triangles = rhs.Triangles;
		TriangleEdges = rhs.TriangleEdges;
		TriangleGroups = rhs.TriangleGroups;
		Edges = rhs.Edges;
		VertexRefCounts = rhs.VertexRefCounts;
		EdgeRefCounts = rhs.EdgeRefCounts;
		TriangleRefCounts = rhs.TriangleRefCounts;
		VertexEdgeLists = rhs.VertexEdgeLists;
		GroupIDCounter = rhs.GroupIDCounter;
		Bounds = rhs.Bounds;
		Pose = rhs.Pose;
		MeshChangeStamp = rhs.MeshChangeStamp;
		ResourceName = rhs.ResourceName;
		bHasVertexColor = rhs.bHasVertexColor;
		bHasVertexNormals = rhs.bHasVertexNormals;
		bHasVertexUVs = rhs.bHasVertexUVs;
		bHasTriangleGroups = rhs.bHasTriangleGroups;

		delete mAttributes;
		mAttributes = new DynamicMeshAttributeSet(this);
		mAttributes->Initialize(GetVertexCount(), GetTriangleCount());
		return *this;
	}

	DynamicMesh::~DynamicMesh()
	{
		delete mAttributes;
		mAttributes = nullptr;
	}

	void DynamicMesh::Clear()
	{
		VertexPositions.clear();
		VertexNormals.clear();
		VertexColors.clear();
		VertexUVs.clear();
		Triangles.clear();
		TriangleEdges.clear();
		TriangleGroups.clear();
		Edges.clear();
		VertexRefCounts.clear();
		EdgeRefCounts.clear();
		TriangleRefCounts.clear();
		VertexEdgeLists.clear();

		bHasVertexColor = false;
		bHasVertexNormals = false;
		bHasVertexUVs = false;
		bHasTriangleGroups = false;
		GroupIDCounter = 0;

		delete mAttributes;
		mAttributes = new DynamicMeshAttributeSet(this);
		ResourceName = "";
	}


	bool DynamicMesh::LoadObj(const char* name)
	{
		Clear();

		int error = 0;
		MemoryFile file(name);
		if (file.GetSize() <= 0)
		{
			return false;
		}

		int VectexTokens = 0;

		char* p = (char*)file.GetData();
		char* pEnd = p + file.GetSize();
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
				VertexUVs.push_back(v);
				bHasVertexUVs = true;
			}
			else if (row[0] == 'f')
			{
				int v[32];
				int vt[32];
				int vn[32];
				int NumVertices = (int)VertexPositions.size();
				int nv = ParseFace(NumVertices, row + 1, v, vt, vn, 32);
				for (int i = 2; i < nv; ++i)
				{
					const int a = v[0];
					const int b = v[i - 1];
					const int c = v[i];
					if (a == b || a == c || b == c)
					{
						error = 1;
						continue;
					}
					if (a < 0 || a >= NumVertices || b < 0 || b >= NumVertices || c < 0 || c >= NumVertices)
						continue;
					AppendTriangle(Index3(a, b, c));
				}
			}
		}

		BuildBounds();

		ResourceName = name;
		return true;
	}

	bool DynamicMesh::ExportObj(const char* name) const
	{
		FILE* fp = fopen(name, "w");
		if (!fp)
		{
			return false;
		}

		int NumFaces = 0;
		for (size_t i = 0; i < Triangles.size(); ++i)
		{
			if (IsTriangleFast((int)i))
			{
				NumFaces++;
			}
		}

		fprintf(fp, "# %d vertices, %d faces\n", (int)VertexPositions.size(), NumFaces);

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
				const Vector3& v = VertexPositions[i];
				fprintf(fp, "v %.3f %.3f %.3f\n", v.x, v.y, v.z);
			}
		}

		const bool bWriteVertexNormals = HasVertexNormals() && VertexNormals.size() == VertexPositions.size();
		const bool bWriteVertexUVs = HasVertexUVs() && VertexUVs.size() == VertexPositions.size();

		if (bWriteVertexUVs)
		{
			fprintf(fp, "# List of texture coordinates, in (u, v [,w]) coordinates, these will vary between 0 and 1, w is optional and defaults to 0. \n");
			for (size_t i = 0; i < VertexUVs.size(); ++i)
			{
				const Vector2& v = VertexUVs[i];
				fprintf(fp, "vt %.3f %.3f\n", v.x, v.y);
			}
		}

		if (bWriteVertexNormals)
		{
			fprintf(fp, "# List of vertex normals in (x,y,z) form; normals might not be unit vectors. \n");
			for (size_t i = 0; i < VertexNormals.size(); ++i)
			{
				const Vector3& v = VertexNormals[i];
				fprintf(fp, "vn %.3f %.3f %.3f\n", v.x, v.y, v.z);
			}
		}

		fprintf(fp, "# Polygonal face element (see below) \n");
		for (size_t i = 0; i < Triangles.size(); ++i)
		{
			if (!IsTriangleFast((int)i))
			{
				continue;
			}
			const Index3& v = Triangles[i];
			const int i0 = v.a + 1;
			const int i1 = v.b + 1;
			const int i2 = v.c + 1;
			if (bWriteVertexUVs && bWriteVertexNormals)
			{
				fprintf(fp, "f %d/%d/%d %d/%d/%d %d/%d/%d\n", i0, i0, i0, i1, i1, i1, i2, i2, i2);
			}
			else if (bWriteVertexUVs)
			{
				fprintf(fp, "f %d/%d %d/%d %d/%d\n", i0, i0, i1, i1, i2, i2);
			}
			else if (bWriteVertexNormals)
			{
				fprintf(fp, "f %d//%d %d//%d %d//%d\n", i0, i0, i1, i1, i2, i2);
			}
			else
			{
				fprintf(fp, "f %d %d %d\n", i0, i1, i2);
			}
		}

		fclose(fp);
		return true;
	}

	DynamicMesh* DynamicMesh::CreateFromObj(const char* name)
	{
		DynamicMesh* p = new DynamicMesh;
		if (!p->LoadObj(name))
		{
			delete p;
			return nullptr;
		}
		return p;
	}

	void DynamicMesh::SetData(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
	{
		Clear();

		int nv = (int)Vertices.size();
		int nt = (int)Indices.size() / 3;

		VertexInfo info;
		info.bHasColor = false;
		info.bHasNormal = true;
		info.bHasUV = false;

		for (int i = 0; i < nv; ++i)
		{
			info.Position = Vertices[i];
			info.Normal = Normals[i];
			AppendVertex(info);
		}

		for (int i = 0; i < nt; ++i)
		{
			AppendTriangle((int)Indices[3*i], (int)Indices[3 * i + 1], (int)Indices[3 * i + 2]);
		}

		BuildBounds();
	}

	int DynamicMesh::AppendVertex(const Vector3& v)
	{
		int index = (int)VertexPositions.size();
		VertexPositions.push_back(v);
		VertexRefCounts.push_back(1);
		VertexEdgeLists.append();

		if (HasVertexColors())
		{
			VectorSetSafe(VertexColors, index, Vector3::Zero(), Vector3::Zero());
		}
		if (HasVertexNormals())
		{
			VectorSetSafe(VertexNormals, index, Vector3::UnitY(), Vector3::UnitY());
		}
		if (HasVertexUVs())
		{
			VectorSetSafe(VertexUVs, index, Vector2::Zero(), Vector2::Zero());
		}

		if (HasAttributes())
		{
			Attributes()->OnNewVertex(index, false);
		}

		UpdateChangeStamps();
		return index;
	}

	int DynamicMesh::AppendVertex(const VertexInfo& info)
	{
		int index = (int)VertexPositions.size();

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

		if (info.bHasUV)
		{
			bHasVertexUVs = true;
		}

		if (HasVertexColors())
		{
			Vector3 val = info.bHasColor ? info.Color : Vector3::Zero();
			VectorSetSafe(VertexColors, index, val, Vector3::Zero());
		}

		if (HasVertexNormals())
		{
			Vector3 val = info.bHasNormal ? info.Normal : Vector3::UnitY();
			VectorSetSafe(VertexNormals, index, val, Vector3::UnitY());
		}

		if (HasVertexUVs())
		{
			Vector2 val = info.bHasUV ? info.UVs : Vector2::Zero();
			VectorSetSafe(VertexUVs, index, val, Vector2::Zero());
		}

		if (HasAttributes())
		{
			Attributes()->OnNewVertex(index, false);
		}

		UpdateChangeStamps();
		return index;
	}

	int DynamicMesh::AppendVertex(const DynamicMesh& from, int fromVID)
	{
		if (!from.IsVertex(fromVID))
		{
			assert(false);
			return -1;
		}

		const int vid = (int)VertexPositions.size();
		VertexPositions.push_back(from.VertexPositions[fromVID]);
		VertexRefCounts.push_back(1);

		if (HasVertexNormals())
		{
			if (from.HasVertexNormals())
			{
				const std::vector<Vector3>&FromNormals = from.VertexNormals;
				VertexNormals.push_back(FromNormals[fromVID]);
			}
			else
			{
				VertexNormals.push_back(Vector3::UnitY());
			}
		}

		if (HasVertexColors())
		{
			if (from.HasVertexColors())
			{
				const std::vector<Vector3>& FromColors = from.VertexColors;
				VertexColors.push_back(FromColors[fromVID]);
			}
			else
			{
				VertexColors.push_back(Vector3::Zero());
			}
		}

		if (HasVertexUVs())
		{
			if (from.HasVertexUVs())
			{
				const std::vector<Vector2>& FromUVs = from.VertexUVs;
				VertexUVs.push_back(FromUVs[fromVID]);
			}
			else
			{
				VertexUVs.push_back(Vector2::Zero());
			}
		}

		VertexEdgeLists.append();

		if (HasAttributes())
		{
			Attributes()->OnNewVertex(vid, false);
		}

		UpdateChangeStamps();
		return vid;
	}


	bool DynamicMesh::Simplify(const SimplificationConfig *cfg)
	{
		std::vector<Vector3> new_v;
		std::vector<int> new_i;

		if (!SimplifyMesh(GetVertexBuffer(), (const void*)GetIndexBuffer(), GetVertexCount(), true, GetTriangleCount(), *cfg, new_v, new_i))
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
		for (int tid = 0; tid < GetTriangleCount(); ++tid)
		{
			if (IsTriangleFast(tid))
			{
				ReverseTriOrientationInternal(tid);
			}
		}

		if (bFlipNormals && bHasVertexNormals)
		{
			for (size_t i = 0; i < VertexNormals.size(); ++i)
			{
				Vector3& Normal = VertexNormals[i];
				Normal = -Normal;
			}
		}

		if (bFlipNormals && HasAttributes())
		{
			for (int NormalLayerIndex = 0; NormalLayerIndex < mAttributes->NumNormalLayers(); ++NormalLayerIndex)
			{
				FDynamicMeshNormalOverlay* NormalOverlay = mAttributes->GetNormalLayer(NormalLayerIndex);
				for (int ElementID = 0; ElementID < NormalOverlay->MaxElementID(); ++ElementID)
				{
					if (NormalOverlay->IsElement(ElementID))
					{
						NormalOverlay->SetElement(ElementID, -NormalOverlay->GetElement(ElementID));
					}
				}
			}
		}

		UpdateChangeStamps();
	}

	std::vector<int> DynamicMesh::GetVexVertices(int VertexID) const
	{
		std::vector<int> s;
		for (int eid : VertexEdgeLists[VertexID])
		{
			s.push_back(GetOtherEdgeVertex(eid, VertexID));
		}
		return s;
	}

	std::vector<int> DynamicMesh::GetVexEdges(int VertexID) const
	{
		std::vector<int> s;
		for (int eid : VertexEdgeLists[VertexID])
		{
			s.push_back(eid);
		}
		return s;
	}

	std::vector<int> DynamicMesh::GetVexTriangles(int VertexID) const
	{
		SmallSet<int> s;
		for (int eid : VertexEdgeLists[VertexID])
		{
			const Edge& e = Edges[eid];
			if (e.Tri[0] != InvalidID)
			{
				s.insert(e.Tri[0]);
			}
			if (e.Tri[1] != InvalidID)
			{
				s.insert(e.Tri[1]);
			}
		}
		return s.to_vector();
	}

	EMeshResult DynamicMesh::MergeEdges(int eKeep, int eDiscard, FMergeEdgesInfo& MergeInfo, bool bCheckValidOrientation)
	{
		MergeInfo = FMergeEdgesInfo();

		if (IsEdge(eKeep) == false || IsEdge(eDiscard) == false)
		{
			return EMeshResult::Failed_NotAnEdge;
		}

		const Edge edgeinfo_keep = GetEdge(eKeep);
		const Edge edgeinfo_discard = GetEdge(eDiscard);
		if (edgeinfo_keep.Tri[1] != InvalidID || edgeinfo_discard.Tri[1] != InvalidID)
		{
			return EMeshResult::Failed_NotABoundaryEdge;
		}

		int a = edgeinfo_keep.Vert[0], b = edgeinfo_keep.Vert[1];
		int tab = edgeinfo_keep.Tri[0];
		int eab = eKeep;
		int c = edgeinfo_discard.Vert[0], d = edgeinfo_discard.Vert[1];
		int tcd = edgeinfo_discard.Tri[0];
		int ecd = eDiscard;

		// Need to correctly orient a,b and c,d and then check that
		// we will not join triangles with incompatible winding order
		// I can't see how to do this purely topologically.
		// So relying on closest-pairs testing.
		int OppAB = OrientTriEdgeAndFindOtherVtx(a, b, GetTriangle(tab));
		int OppCD = OrientTriEdgeAndFindOtherVtx(c, d, GetTriangle(tcd));

		// Refuse to merge if doing so would create a duplicate triangle
		if (OppAB == OppCD)
		{
			return EMeshResult::Failed_InvalidNeighbourhood;
		}

		int x = c; c = d; d = x;   // joinable bdry edges have opposing orientations, so flip to get ac and b/d correspondences
		Vector3 Va = GetVertex(a), Vb = GetVertex(b), Vc = GetVertex(c), Vd = GetVertex(d);
		if (bCheckValidOrientation &&
			((Va - Vc).SquareLength() + (Vb - Vd).SquareLength()) >
			((Va - Vd).SquareLength() + (Vb - Vc).SquareLength()))
		{
			return EMeshResult::Failed_SameOrientation;
		}

		// alternative that detects normal flip of triangle tcd. This is a more
		// robust geometric test, but fails if tri is degenerate...also more expensive
		//Vector3 otherv = GetVertex(tcd_otherv);
		//Vector3 Ncd = VectorUtil::NormalDirection(GetVertex(c), GetVertex(d), otherv);
		//Vector3 Nab = VectorUtil::NormalDirection(GetVertex(a), GetVertex(b), otherv);
		//if (Ncd.Dot(Nab) < 0)
		//return EMeshResult::Failed_SameOrientation;

		MergeInfo.KeptEdge = eab;
		MergeInfo.RemovedEdge = ecd;

		// if a/c or b/d are connected by an existing edge, we can't merge
		if (a != c && FindEdge(a, c) != InvalidID)
		{
			return EMeshResult::Failed_InvalidNeighbourhood;
		}
		if (b != d && FindEdge(b, d) != InvalidID)
		{
			return EMeshResult::Failed_InvalidNeighbourhood;
		}

		// if vertices at either end already share a common neighbour vertex, and we
		// do the merge, that would create duplicate edges. This is something like the
		// 'link condition' in edge collapses.
		// Note that we have to catch cases where both edges to the shared vertex are
		// boundary edges, in that case we will also merge this edge later on
		int MaxAdjBoundaryMerges[2]{ 0,0 };
		if (a != c)
		{
			int ea = 0, ec = 0, other_v = (b == d) ? b : -1;
			std::vector<int> VtxVerticesItr = GetVexVertices(c);
			for (int cnbr : VtxVerticesItr)
			{
				if (cnbr != other_v && (ea = FindEdge(a, cnbr)) != InvalidID)
				{
					ec = FindEdge(c, cnbr);
					if (IsBoundaryEdge(ea) == false || IsBoundaryEdge(ec) == false)
					{
						return EMeshResult::Failed_InvalidNeighbourhood;
					}
					else
					{
						MaxAdjBoundaryMerges[0]++;
					}
				}
			}
		}
		if (b != d)
		{
			int eb = 0, ed = 0, other_v = (a == c) ? a : -1;
			std::vector<int> VtxVerticesItr = GetVexVertices(d);
			for (int dnbr : VtxVerticesItr)
			{
				if (dnbr != other_v && (eb = FindEdge(b, dnbr)) != InvalidID)
				{
					ed = FindEdge(d, dnbr);
					if (IsBoundaryEdge(eb) == false || IsBoundaryEdge(ed) == false)
					{
						return EMeshResult::Failed_InvalidNeighbourhood;
					}
					else
					{
						MaxAdjBoundaryMerges[1]++;
					}
				}
			}
		}

		// [TODO] this acts on each interior tri twice. could avoid using vtx-tri iterator?
		if (a != c)
		{
			// replace c w/ a in edges and tris connected to c, and move edges to a
			for (int eid : VertexEdgeLists[c])
			{
				if (eid == eDiscard)
				{
					continue;
				}
				ReplaceEdgeVertex(eid, c, a);
				short rc = 0;
				const Edge Edge = Edges[eid];
				if (ReplaceTriangleVertex(Edge.Tri[0], c, a) >= 0)
				{
					rc++;
				}
				if (Edge.Tri[1] != InvalidID)
				{
					if (ReplaceTriangleVertex(Edge.Tri[1], c, a) >= 0)
					{
						rc++;
					}
				}
				VertexEdgeLists[a].push_back(eid);
				if (rc > 0)
				{
					VertexRefCounts[a] += rc;
					assert(VertexRefCounts[c] >= rc);
					VertexRefCounts[c] -= rc;
				}
			}
			VertexEdgeLists[c].clear();
			assert(VertexRefCounts[c] > 0);
			VertexRefCounts[c]--;
			MergeInfo.RemovedVerts[0] = c;
		}
		else
		{
			VertexEdgeLists[a].remove(ecd);
			MergeInfo.RemovedVerts[0] = InvalidID;
		}
		MergeInfo.KeptVerts[0] = a;

		if (d != b)
		{
			// replace d w/ b in edges and tris connected to d, and move edges to b
			for (int eid : VertexEdgeLists[d])
			{
				if (eid == eDiscard)
				{
					continue;
				}
				ReplaceEdgeVertex(eid, d, b);
				short rc = 0;
				const Edge Edge = Edges[eid];
				if (ReplaceTriangleVertex(Edge.Tri[0], d, b) >= 0)
				{
					rc++;
				}
				if (Edge.Tri[1] != InvalidID)
				{
					if (ReplaceTriangleVertex(Edge.Tri[1], d, b) >= 0)
					{
						rc++;
					}
				}
				VertexEdgeLists[b].push_back(eid);
				if (rc > 0)
				{
					VertexRefCounts[b] += rc;
					assert(VertexRefCounts[d] >= rc);
					VertexRefCounts[d] -= rc;
				}

			}
			VertexEdgeLists[d].clear();
			assert(VertexRefCounts[d] > 0);
			VertexRefCounts[d]--;
			MergeInfo.RemovedVerts[1] = d;
		}
		else
		{
			VertexEdgeLists[b].remove(ecd);
			MergeInfo.RemovedVerts[1] = InvalidID;
		}
		MergeInfo.KeptVerts[1] = b;

		// replace edge cd with edge ab in triangle tcd
		ReplaceTriangleEdge(tcd, ecd, eab);
		assert(EdgeRefCounts[ecd] > 0);
		EdgeRefCounts[ecd] --;

		// update edge-tri adjacency
		SetEdgeTrianglesInternal(eab, tab, tcd);

		// Once we merge ab to cd, there may be additional edges (now) connected
		// to either a or b that are connected to the same vertex on their 'other' side.
		// So we now have two boundary edges connecting the same two vertices - disaster!
		// We need to find and merge these edges.
		MergeInfo.ExtraRemovedEdges = Index2(InvalidID, InvalidID);
		MergeInfo.ExtraKeptEdges = Index2(InvalidID, InvalidID);
		for (int vi = 0; vi < 2; ++vi)
		{
			int v1 = a, v2 = c;   // vertices of merged edge
			if (vi == 1)
			{
				v1 = b; v2 = d;
			}
			if (v1 == v2)
			{
				continue;
			}

			std::vector<int> edges_v = GetVexEdges(v1);
			int Nedges = (int)edges_v.size();
			int FoundNum = 0;
			// in this loop, we compare 'other' vert_1 and vert_2 of edges around v1.
			// problem case is when vert_1 == vert_2  (ie two edges w/ same other vtx).
			for (int i = 0; i < Nedges && FoundNum < MaxAdjBoundaryMerges[vi]; ++i)
			{
				int edge_1 = edges_v[i];
				// Skip any non-boundary edge, or edge we've already removed via merging
				if (!(EdgeRefCounts[edge_1] > 0) || !IsBoundaryEdge(edge_1))
				{
					continue;
				}
				int vert_1 = GetOtherEdgeVertex(edge_1, v1);
				for (int j = i + 1; j < Nedges; ++j)
				{
					int edge_2 = edges_v[j];
					// Skip any non-boundary edge, or edge we've already removed via merging
					if (!(EdgeRefCounts[edge_2] > 0) || !IsBoundaryEdge(edge_2))
					{
						continue;
					}
					int vert_2 = GetOtherEdgeVertex(edge_2, v1);
					if (vert_1 == vert_2)
					{
						// replace edge_2 w/ edge_1 in tri, update edge and vtx-edge-nbr lists
						int tri_1 = Edges[edge_1].Tri[0];
						int tri_2 = Edges[edge_2].Tri[0];
						ReplaceTriangleEdge(tri_2, edge_2, edge_1);
						SetEdgeTrianglesInternal(edge_1, tri_1, tri_2);
						VertexEdgeLists[v1].remove(edge_2);
						VertexEdgeLists[vert_1].remove(edge_2);
						assert(EdgeRefCounts[edge_2] > 0);
						EdgeRefCounts[edge_2] --;
						if (FoundNum == 0)
						{
							MergeInfo.ExtraRemovedEdges[vi] = edge_2;
							MergeInfo.ExtraKeptEdges[vi] = edge_1;
						}
						else
						{
							MergeInfo.BowtiesRemovedEdges.push_back(edge_2);
							MergeInfo.BowtiesKeptEdges.push_back(edge_1);
						}

						FoundNum++;				  // exit outer i loop if we've found all possible merge edges
						break;					  // exit inner j loop; we won't merge anything else to edge_1
					}
				}
			}
		}

		if (HasAttributes())
		{
			mAttributes->OnMergeEdges(MergeInfo);
		}

		UpdateChangeStamps();
		return EMeshResult::Ok;
	}

	EMeshResult DynamicMesh::PokeTriangle(int TriangleID, const Vector3& BaryCoordinates, FPokeTriangleInfo& PokeResult)
	{
		PokeResult = FPokeTriangleInfo();

		if (!IsTriangle(TriangleID))
		{
			return EMeshResult::Failed_NotATriangle;
		}

		Index3 tv = GetTriangle(TriangleID);
		Index3 te = GetTriEdges(TriangleID);

		VertexInfo vinfo = GetTringleBaryPoint(TriangleID, BaryCoordinates[0], BaryCoordinates[1], BaryCoordinates[2]);
		int center = AppendVertex(vinfo);

		int eaC = AddEdgeInternal(tv[0], center, -1, -1);
		int ebC = AddEdgeInternal(tv[1], center, -1, -1);
		int ecC = AddEdgeInternal(tv[2], center, -1, -1);
		VertexRefCounts[tv[0]] += 1;
		VertexRefCounts[tv[1]] += 1;
		VertexRefCounts[tv[2]] += 1;
		VertexRefCounts[center] += 3;

		SetTriangleInternal(TriangleID, tv[0], tv[1], center);
		SetTriangleEdgesInternal(TriangleID, te[0], ebC, eaC);

		int t1 = AddTriangleInternal(tv[1], tv[2], center, te[1], ecC, ebC);
		int t2 = AddTriangleInternal(tv[2], tv[0], center, te[2], eaC, ecC);

		ReplaceEdgeTriangle(te[1], TriangleID, t1);
		ReplaceEdgeTriangle(te[2], TriangleID, t2);

		SetEdgeTrianglesInternal(eaC, TriangleID, t2);
		SetEdgeTrianglesInternal(ebC, TriangleID, t1);
		SetEdgeTrianglesInternal(ecC, t1, t2);

		if (HasTriangleGroups())
		{
			int g = TriangleGroups[TriangleID];
			VectorSetSafe(TriangleGroups, t1, g, 0);
			VectorSetSafe(TriangleGroups, t2, g, 0);
		}

		PokeResult.OriginalTriangle = TriangleID;
		PokeResult.TriVertices = tv;
		PokeResult.NewVertex = center;
		PokeResult.NewTriangles = Index2(t1, t2);
		PokeResult.NewEdges = Index3(eaC, ebC, ecC);
		PokeResult.BaryCoords = BaryCoordinates;

		if (HasAttributes())
		{
			mAttributes->OnPokeTriangle(PokeResult);
		}

		UpdateChangeStamps();
		return EMeshResult::Ok;
	}


	int DynamicMesh::FindEdgeFromTri(int vA, int vB, int tID) const
	{
		const Index3& Triangle = Triangles[tID];
		const Index3& TriangleEdgeIDs = TriangleEdges[tID];
		if (SamePairUnordered(vA, vB, Triangle[0], Triangle[1]))
		{
			return TriangleEdgeIDs[0];
		}
		if (SamePairUnordered(vA, vB, Triangle[1], Triangle[2]))
		{
			return TriangleEdgeIDs[1];
		}
		if (SamePairUnordered(vA, vB, Triangle[2], Triangle[0]))
		{
			return TriangleEdgeIDs[2];
		}
		return -1;
	}

	int DynamicMesh::FindEdgeFromTrianglePair(int TriA, int TriB) const
	{
		if (TriangleRefCounts[TriA] > 0 && TriangleRefCounts[TriB] > 0)
		{
			for (int j = 0; j < 3; ++j)
			{
				int EdgeID = TriangleEdges[TriA][j];
				const Edge e = Edges[EdgeID];
				int NbrT = (e.Tri[0] == TriA) ? e.Tri[1] : e.Tri[0];
				if (NbrT == TriB)
				{
					return EdgeID;
				}
			}
		}
		return -1;
	}

	EMeshResult DynamicMesh::SplitEdge(int eab, FEdgeSplitInfo& SplitInfo, float split_t)
	{
		SplitInfo = FEdgeSplitInfo();

		if (!IsEdge(eab))
		{
			return EMeshResult::Failed_NotAnEdge;
		}

		const Edge& e = Edges[eab];
		int a = e.Vert[0], b = e.Vert[1];
		int t0 = e.Tri[0];
		if (t0 == -1)
		{
			return EMeshResult::Failed_BrokenTopology;
		}
		Index3 T0tv = GetTriangle(t0);
		int c = OrientTriEdgeAndFindOtherVtx(a, b, T0tv);

		assert(c >= 0);
		if (VertexRefCounts[c] > 250)
		{
			assert(false);
			return EMeshResult::Failed_HitValenceLimit;
		}
		if (a != e.Vert[0])
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
			if (HasVertexUVs())
			{
				SetVertexUV(f, Maths::LinearInterp(GetVertexUV(a), GetVertexUV(b), split_t));
			}

			Index3 T0te = GetTriEdges(t0);
			int ebc = T0te[FindEdgeIndexInTri(b, c, T0tv)];

			ReplaceTriangleVertex(t0, b, f);

			int t2 = AddTriangleInternal(f, b, c, -1, -1, -1);
			if (HasTriangleGroups())
			{
				int group0 = TriangleGroups[t0];
				VectorSetSafe(TriangleGroups, t2, group0, 0);
			}

			ReplaceEdgeTriangle(ebc, t0, t2);
			int eaf = eab;
			ReplaceEdgeVertex(eaf, b, f);
			VertexEdgeLists[b].remove(eab);
			VertexEdgeLists[f].push_back(eaf);

			int efb = AddEdgeInternal(f, b, t2, -1);
			int efc = AddEdgeInternal(f, c, t0, t2);

			ReplaceTriangleEdge(t0, ebc, efc);
			SetTriangleEdgesInternal(t2, efb, ebc, efc);

			VertexRefCounts[c] += 1;
			VertexRefCounts[f] += 2;

			SplitInfo.bIsBoundary = true;
			SplitInfo.OtherVertices = Index2(c, -1);
			SplitInfo.NewVertex = f;
			SplitInfo.NewEdges = Index3(efb, efc, -1);
			SplitInfo.NewTriangles = Index2(t2, -1);

			if (HasAttributes())
			{
				mAttributes->OnSplitEdge(SplitInfo);
			}

			UpdateChangeStamps();
			return EMeshResult::Ok;

		}
		else
		{
			int t1 = Edges[eab].Tri[1];
			SplitInfo.OriginalTriangles.b = t1;
			Index3 T1tv = GetTriangle(t1);
			int d = FindTriOtherVtx(a, b, T1tv);

			if (VertexRefCounts[d] > 250)
			{
				return EMeshResult::Failed_HitValenceLimit;
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
			if (HasVertexUVs())
			{
				SetVertexUV(f, Maths::LinearInterp(GetVertexUV(a), GetVertexUV(b), split_t));
			}

			Index3 T0te = GetTriEdges(t0);
			int ebc = T0te[FindEdgeIndexInTri(b, c, T0tv)];
			Index3 T1te = GetTriEdges(t1);
			int edb = T1te[FindEdgeIndexInTri(d, b, T1tv)];

			ReplaceTriangleVertex(t0, b, f);
			ReplaceTriangleVertex(t1, b, f);

			int t2 = AddTriangleInternal(f, b, c, -1, -1, -1);
			int t3 = AddTriangleInternal(f, d, b, -1, -1, -1);
			if (HasTriangleGroups())
			{
				int group0 = TriangleGroups[t0];
				VectorSetSafe(TriangleGroups, t2, group0, 0);
				int group1 = TriangleGroups[t1];
				VectorSetSafe(TriangleGroups, t3, group1, 0);
			}

			ReplaceEdgeTriangle(ebc, t0, t2);
			ReplaceEdgeTriangle(edb, t1, t3);

			int eaf = eab;
			ReplaceEdgeVertex(eaf, b, f);

			VertexEdgeLists[b].remove(eab);
			VertexEdgeLists[f].push_back(eaf);

			int efb = AddEdgeInternal(f, b, t2, t3);
			int efc = AddEdgeInternal(f, c, t0, t2);
			int edf = AddEdgeInternal(d, f, t1, t3);

			ReplaceTriangleEdge(t0, ebc, efc);
			ReplaceTriangleEdge(t1, edb, edf);
			SetTriangleEdgesInternal(t2, efb, ebc, efc);
			SetTriangleEdgesInternal(t3, edf, edb, efb);

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
				mAttributes->OnSplitEdge(SplitInfo);
			}

			UpdateChangeStamps();
			return EMeshResult::Ok;
		}
	}

	EMeshResult DynamicMesh::CollapseEdge(int vKeep, int vRemove, float collapse_t, FEdgeCollapseInfo& CollapseInfo)
	{
		CollapseInfo = FEdgeCollapseInfo();

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

		const Edge& EdgeAB = Edges[eab];
		int t0 = EdgeAB.Tri[0];
		if (t0 == -1)
		{
			return EMeshResult::Failed_BrokenTopology;
		}
		Index3 T0tv = GetTriangle(t0);
		int c = FindTriOtherVtx(a, b, T0tv);

		// look up opposing triangle/vtx if we are not in boundary case
		bool bIsBoundaryEdge = false;
		int d = -1;
		int t1 = EdgeAB.Tri[1];
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
		//  [TODO] if we had tri iterator for a, couldn't we assert each tri for b  (skipping t0 and t1) ?

		int edges_a_count = VertexEdgeLists[a].size();
		int eac = -1, ead = -1, ebc = -1, ebd = -1;
		for (int eid_a : VertexEdgeLists[a])
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
			for (int eid_b : VertexEdgeLists[b])
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
		//  and edge cd exists. But that is not conclusive, we also have to assert that
		//  cd is an internal edge, and that each of its tris contain a or b
		if (edges_a_count == 3 && bIsBoundaryEdge == false)
		{
			int edc = FindEdge(d, c);
			if (edc != -1)
			{
				const Edge& EdgeDC = Edges[edc];
				if (EdgeDC.Tri[1] != -1)
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

		// TODO: it's unclear how this case would ever be encountered; assert if it is needed
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
		Vector3 KeptPos = GetVertex(vKeep);
		Vector3 RemovedPos = GetVertex(vRemove);
		Vector2 RemovedUV;
		if (HasVertexUVs())
		{
			RemovedUV = GetVertexUV(vRemove);
		}
		Vector3 RemovedNormal;
		if (HasVertexNormals())
		{
			RemovedNormal = GetVertexNormal(vRemove);
		}
		Vector3 RemovedColor;
		if (HasVertexColors())
		{
			RemovedColor = GetVertexColor(vRemove);
		}

		// 1) remove edge ab from vtx b
		// 2) find edges ad and ac, and tris tad, tac across those edges  (will use later)
		// 3) for other edges, replace a with b, and add that edge to b
		// 4) replace a with b in all triangles connected to a
		int tad = -1, tac = -1;
		for (int eid : VertexEdgeLists[a])
		{
			int o = GetOtherEdgeVertex(eid, a);
			if (o == b)
			{
				if (VertexEdgeLists[b].remove(eid) == -1)
				{
					assert(false);
					return EMeshResult::Failed_UnrecoverableError;
				}
			}
			else if (o == c)
			{
				if (VertexEdgeLists[c].remove(eid) == -1)
				{
					assert(false);
					return EMeshResult::Failed_UnrecoverableError;
				}
				tac = GetOtherEdgeTriangle(eid, t0);
			}
			else if (o == d)
			{
				if (VertexEdgeLists[d].remove(eid) == -1)
				{
					assert(false);
					return EMeshResult::Failed_UnrecoverableError;
				}
				tad = GetOtherEdgeTriangle(eid, t1);
			}
			else
			{
				if (ReplaceEdgeVertex(eid, a, b) == -1)
				{
					assert(false);
					return EMeshResult::Failed_UnrecoverableError;
				}
				VertexEdgeLists[b].push_back(eid);
			}

			// [TODO] perhaps we can already have unique tri list because of the manifold-nbrhood assert we need to do...
			const Edge& e = Edges[eid];
			for (int j = 0; j < 2; ++j)
			{
				int t_j = e.Tri[j];
				if (t_j != -1 && t_j != t0 && t_j != t1)
				{
					if (TriangleHasVertex(t_j, a))
					{
						if (ReplaceTriangleVertex(t_j, a, b) == -1)
						{
							assert(false);
							return EMeshResult::Failed_UnrecoverableError;
						}
						VertexRefCounts[b]++;
						assert(VertexRefCounts[a] > 0);
						VertexRefCounts[a]--;
					}
				}
			}
		}

		if (bIsBoundaryEdge == false)
		{
			// remove all edges from vtx a, then remove vtx a
			VertexEdgeLists[a].clear();
			assert(VertexRefCounts[a] == 3);		// in t0,t1, and initial ref
			VertexRefCounts[a] -= 3;
			assert(VertexRefCounts[a] == 0);

			// remove triangles T0 and T1, and update b/c/d refcounts
			TriangleRefCounts[t0]--;
			TriangleRefCounts[t1]--;
			VertexRefCounts[c]--;
			VertexRefCounts[d]--;
			VertexRefCounts[b] -= 2;
			assert(TriangleRefCounts[t0] == 0);
			assert(TriangleRefCounts[t1] == 0);

			// remove edges ead, eab, eac
			EdgeRefCounts[ead]--;
			EdgeRefCounts[eab]--;
			EdgeRefCounts[eac]--;
			assert(EdgeRefCounts[ead] == 0);
			assert(EdgeRefCounts[eab] == 0);
			assert(EdgeRefCounts[eac] == 0);

			// replace t0 and t1 in edges ebd and ebc that we kept
			ebd = FindEdgeFromTri(b, d, t1);
			if (ebc == -1)   // we may have already looked this up
			{
				ebc = FindEdgeFromTri(b, c, t0);
			}

			if (ReplaceEdgeTriangle(ebd, t1, tad) == -1)
			{
				assert(false);
				return EMeshResult::Failed_UnrecoverableError;
			}

			if (ReplaceEdgeTriangle(ebc, t0, tac) == -1)
			{
				assert(false);
				return EMeshResult::Failed_UnrecoverableError;
			}

			// update tri-edge-nbrs in tad and tac
			if (tad != -1)
			{
				if (ReplaceTriangleEdge(tad, ead, ebd) == -1)
				{
					assert(false);
					return EMeshResult::Failed_UnrecoverableError;
				}
			}
			if (tac != -1)
			{
				if (ReplaceTriangleEdge(tac, eac, ebc) == -1)
				{
					assert(false);
					return EMeshResult::Failed_UnrecoverableError;
				}
			}

		}
		else
		{
			//  boundary-edge path. this is basically same code as above, just not referencing t1/d

			// remove all edges from vtx a, then remove vtx a
			VertexEdgeLists[a].clear();
			assert(VertexRefCounts[a] == 2);		// in t0 and initial ref
			VertexRefCounts[a] -= 2;
			assert(VertexRefCounts[a] == 0);

			// remove triangle T0 and update b/c refcounts
			TriangleRefCounts[t0]--;
			VertexRefCounts[c]--;
			VertexRefCounts[b]--;
			assert(TriangleRefCounts[t0] == 0);

			// remove edges eab and eac
			EdgeRefCounts[eab]--;
			EdgeRefCounts[eac]--;
			assert(EdgeRefCounts[eab] == 0);
			assert(EdgeRefCounts[eac] == 0);

			// replace t0 in edge ebc that we kept
			ebc = FindEdgeFromTri(b, c, t0);
			if (ReplaceEdgeTriangle(ebc, t0, tac) == -1)
			{
				assert(false);
				return EMeshResult::Failed_UnrecoverableError;
			}

			// update tri-edge-nbrs in tac
			if (tac != -1)
			{
				if (ReplaceTriangleEdge(tac, eac, ebc) == -1)
				{
					assert(false);
					return EMeshResult::Failed_UnrecoverableError;
				}
			}
		}

		// set kept vertex to interpolated collapse position
		SetVertex(vKeep, LinearInterp(KeptPos, RemovedPos, collapse_t));
		if (HasVertexUVs())
		{
			SetVertexUV(vKeep, LinearInterp(GetVertexUV(vKeep), RemovedUV, collapse_t));
		}
		if (HasVertexNormals())
		{
			SetVertexNormal(vKeep, LinearInterp(GetVertexNormal(vKeep), RemovedNormal, collapse_t).Unit());
		}
		if (HasVertexColors())
		{
			SetVertexColor(vKeep, LinearInterp(GetVertexColor(vKeep), RemovedColor, collapse_t));
		}

		CollapseInfo.KeptVertex = vKeep;
		CollapseInfo.RemovedVertex = vRemove;
		CollapseInfo.bIsBoundary = bIsBoundaryEdge;
		CollapseInfo.CollapsedEdge = eab;
		CollapseInfo.RemovedTris = Index2(t0, t1);
		CollapseInfo.RemovedEdges = Index2(eac, ead);
		CollapseInfo.KeptEdges = Index2(ebc, ebd);
		CollapseInfo.CollapseT = collapse_t;

		if (HasAttributes())
		{
			mAttributes->OnCollapseEdge(CollapseInfo);
		}

		UpdateChangeStamps();
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
			const int vOther = e.Vert[0] == vID ? e.Vert[1] : e.Vert[0];
			if (TriHasSequentialVertices(e.Tri[0], vID, vOther))
			{
				N++;
			}
			const int et1 = e.Tri[1];
			if (et1 != -1 && TriHasSequentialVertices(et1, vID, vOther))
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
			return Edges[EID].Tri[0];
		}

		return -1;
	}

	int DynamicMesh::AppendTriangle(const Index3& tv, int gid)
	{
		if (IsVertex(tv[0]) == false || IsVertex(tv[1]) == false || IsVertex(tv[2]) == false)
		{
			assert(false);
			return -1;
		}
		if (tv[0] == tv[1] || tv[0] == tv[2] || tv[1] == tv[2])
		{
			assert(false);
			return -1;
		}

		bool boundary0, boundary1, boundary2;
		int e0 = FindEdgeInternal(tv[0], tv[1], boundary0);
		int e1 = FindEdgeInternal(tv[1], tv[2], boundary1);
		int e2 = FindEdgeInternal(tv[2], tv[0], boundary2);
		if ((e0 != -1 && boundary0 == false)
			|| (e1 != -1 && boundary1 == false)
			|| (e2 != -1 && boundary2 == false))
		{
			return NonManifoldID;
		}

		if (e0 != -1 && e1 != -1 && e2 != -1)
		{
			int ti = Edges[e0].Tri[0];
			if (Triangles[ti][0] == tv[2] || Triangles[ti][1] == tv[2] || Triangles[ti][2] == tv[2])
			{
				return DuplicateTriangleID;
			}
			assert(Edges[e0].Tri[1] == -1);
		}

		bool bHasGroups = HasTriangleGroups();  // have to check before changing .triangles

		// now safe to insert triangle
		int tid = (int)Triangles.size();
		Triangles.push_back(tv);
		TriangleRefCounts.push_back(1);

		if (bHasGroups)
		{
			TriangleGroups.push_back(gid);
			GroupIDCounter = std::max(GroupIDCounter, gid + 1);
		}

		VertexRefCounts[tv[0]]++;
		VertexRefCounts[tv[1]]++;
		VertexRefCounts[tv[2]]++;

		AddTriangleEdge(tid, tv[0], tv[1], 0, e0);
		AddTriangleEdge(tid, tv[1], tv[2], 1, e1);
		AddTriangleEdge(tid, tv[2], tv[0], 2, e2);

		if (HasAttributes())
		{
			mAttributes->OnNewTriangle(tid, false);
		}

		UpdateChangeStamps();
		return tid;
	}

	EMeshResult DynamicMesh::InsertTriangle(int tid, const Index3& tv, int gid, bool bUnsafe)
	{
		if (tid < 0)
		{
			assert(false);
			return EMeshResult::Failed_CannotAllocateTriangle;
		}

		if (tid < (int)TriangleRefCounts.size() && TriangleRefCounts[tid] > 0)
		{
			return EMeshResult::Failed_TriangleAlreadyExists;
		}

		if (IsVertex(tv[0]) == false || IsVertex(tv[1]) == false || IsVertex(tv[2]) == false)
		{
			assert(false);
			return EMeshResult::Failed_NotAVertex;
		}
		if (tv[0] == tv[1] || tv[0] == tv[2] || tv[1] == tv[2])
		{
			assert(false);
			return EMeshResult::Failed_InvalidNeighbourhood;
		}

		// look up edges. if any already have two triangles, this would
		// create non-manifold geometry and so we do not allow it
		int e0 = FindEdge(tv[0], tv[1]);
		int e1 = FindEdge(tv[1], tv[2]);
		int e2 = FindEdge(tv[2], tv[0]);
		if ((e0 != InvalidID && IsBoundaryEdge(e0) == false)
			|| (e1 != InvalidID && IsBoundaryEdge(e1) == false)
			|| (e2 != InvalidID && IsBoundaryEdge(e2) == false))
		{
			return EMeshResult::Failed_WouldCreateNonmanifoldEdge;
		}

		if (tid >= (int)TriangleRefCounts.size())
		{
			TriangleRefCounts.resize((size_t)tid + 1, 0);
		}
		TriangleRefCounts[tid] = 1;

		// now safe to insert triangle
		VectorSetSafe(Triangles, tid, tv, Index3(-1, -1, -1));
		if (HasTriangleGroups())
		{
			VectorSetSafe(TriangleGroups, tid, gid, 0);
			GroupIDCounter = std::max(GroupIDCounter, gid + 1);
		}

		// increment ref counts and update/create edges
		VertexRefCounts[tv[0]]++;
		VertexRefCounts[tv[1]]++;
		VertexRefCounts[tv[2]]++;

		AddTriangleEdge(tid, tv[0], tv[1], 0, e0);
		AddTriangleEdge(tid, tv[1], tv[2], 1, e1);
		AddTriangleEdge(tid, tv[2], tv[0], 2, e2);

		if (HasAttributes())
		{
			mAttributes->OnNewTriangle(tid, true);
		}

		UpdateChangeStamps();
		return EMeshResult::Ok;
	}

	EMeshResult DynamicMesh::RemoveVertex(int vID, bool bPreserveManifold)
	{
		if (!IsVertex(vID))
		{
			return EMeshResult::Failed_NotAVertex;
		}

		std::vector<int> triangles = GetVexTriangles(vID);
		if (bPreserveManifold)
		{
			for (int tID : triangles)
			{
				const Index3 tri = GetTriangle(tID);
				const int j = FindTriIndex(vID, tri);
				if (j < 0)
				{
					return EMeshResult::Failed_BrokenTopology;
				}

				const int oa = tri[(j + 1) % 3];
				const int ob = tri[(j + 2) % 3];
				const int eid = FindEdge(oa, ob);
				if (eid >= 0 && IsBoundaryEdge(eid))
				{
					continue;
				}
				if (IsBoundaryVertex(oa) || IsBoundaryVertex(ob))
				{
					return EMeshResult::Failed_WouldCreateBowtie;
				}
			}
		}

		for (int tID : triangles)
		{
			const EMeshResult result = RemoveTriangle(tID, false, bPreserveManifold);
			if (result != EMeshResult::Ok)
			{
				return result;
			}
		}

		if (VertexRefCounts[vID] != 1)
		{
			return EMeshResult::Failed_VertexStillReferenced;
		}

		VertexRefCounts[vID]--;
		VertexEdgeLists[vID].clear();

		if (HasAttributes())
		{
			mAttributes->OnRemoveVertex(vID);
		}

		UpdateChangeStamps();
		return EMeshResult::Ok;
	}

	EMeshResult DynamicMesh::RemoveTriangle(int tID, bool bRemoveIsolatedVertices, bool bPreserveManifold)
	{
		if (TriangleRefCounts[tID] <= 0)
		{
			assert(false);
			return EMeshResult::Failed_NotATriangle;
		}

		Index3 tv = GetTriangle(tID);
		Index3 te = GetTriEdges(tID);

		// if any tri vtx is a boundary vtx connected to two interior edges, then
		// we cannot remove this triangle because it would create a bowtie vertex!
		// (that vtx already has 2 boundary edges, and we would add two more)
		if (bPreserveManifold)
		{
			for (int j = 0; j < 3; ++j)
			{
				if (IsBoundaryVertex(tv[j]))
				{
					if (IsBoundaryEdge(te[j]) == false && IsBoundaryEdge(te[(j + 2) % 3]) == false)
					{
						return EMeshResult::Failed_WouldCreateBowtie;
					}
				}
			}
		}

		// Remove triangle from its edges. if edge has no triangles left,
		// then it is removed.
		for (int j = 0; j < 3; ++j)
		{
			int eid = te[j];
			ReplaceEdgeTriangle(eid, tID, -1);
			const Edge& e = Edges[eid];
			if (e.Tri[0] == -1)
			{
				int a = e.Vert[0];
				VertexEdgeLists[a].remove(eid);

				int b = e.Vert[1];
				VertexEdgeLists[b].remove(eid);

				assert(EdgeRefCounts[eid] > 0);
				EdgeRefCounts[eid]--;
			}
		}

		// free this triangle

		TriangleRefCounts[tID]--;
		assert(TriangleRefCounts[tID] == 0);

		// Decrement vertex refcounts. If any hit 1 and we got remove-isolated flag,
		// we need to remove that vertex
		for (int j = 0; j < 3; ++j)
		{
			int vid = tv[j];
			assert(VertexRefCounts[vid] > 0);
			VertexRefCounts[vid]--;
			if (bRemoveIsolatedVertices && VertexRefCounts[vid] == 1)
			{
				VertexRefCounts[vid]--;
				assert(VertexRefCounts[vid] == 0);
				VertexEdgeLists[vid].clear();
			}
		}

		if (HasAttributes())
		{
			mAttributes->OnRemoveTriangle(tID);
		}

		UpdateChangeStamps();
		return EMeshResult::Ok;
	}

	EMeshResult DynamicMesh::SetTriangle(int tID, const Index3& newv)
	{
		if (HasAttributes() == false)
		{
			assert(false);
			return EMeshResult::Failed_Unsupported;
		}

		const  bool bRemoveIsolatedVertices = true;
		Index3 tv = GetTriangle(tID);
		Index3 te = GetTriEdges(tID);
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
			if (e.Tri[0] == -1)
			{
				int a = e.Vert[0];
				VertexEdgeLists[a].remove(eid);

				int b = e.Vert[1];
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

	Index2 DynamicMesh::GetEdgeOpposingV(int eID) const
	{
		const Edge& e = Edges[eID];
		int a = e.Vert[0];
		int b = e.Vert[1];

		// ** it is important that verts returned maintain [c,d] order!!
		int c = FindTriOtherVtxUnsafe(a, b, Triangles[e.Tri[0]]);
		if (e.Tri[1] != InvalidID)
		{
			int d = FindTriOtherVtxUnsafe(a, b, Triangles[e.Tri[1]]);
			return Index2(c, d);
		}
		else
		{
			return Index2(c, -1);
		}
	}

	void DynamicMesh::BuildBounds()
	{
		Bounds = Box3::Empty();
		for (int tid = 0; tid < GetTriangleCount(); ++tid)
		{
			if (!IsTriangleFast(tid))
			{
				continue;
			}

			Index3 tri = GetTriangle(tid);
			Bounds.Encapsulate(VertexPositions[tri[0]], VertexPositions[tri[1]], VertexPositions[tri[2]]);
		}
	}

	void DynamicMesh::CalculateWeightAverageNormals()
	{
		if (Triangles.empty())
		{
			return;
		}

		int NumVertices = GetVertexCount();
		int NumTriangles = GetTriangleCount();

		bHasVertexNormals = true;

		std::vector<float> Weight;
		Weight.resize(NumVertices, 0.0f);
		VertexNormals.resize(NumVertices);
		std::fill(VertexNormals.begin(), VertexNormals.end(), Vector3::Zero());

		auto SafeAcos = [](float Value)
		{
			return acosf(std::max(-1.0f, std::min(1.0f, Value)));
		};

		for (int tid = 0; tid < NumTriangles; ++tid)
		{
			if (!IsTriangleFast(tid))
			{
				continue;
			}

			Index3 tv = GetTriangle(tid);

			Triangle3 tri;
			GetTriVertices(tid, tri.v0, tri.v1, tri.v2);

			const Vector3 v20Raw = tri.v2 - tri.v0;
			const Vector3 v10Raw = tri.v1 - tri.v0;
			const Vector3 v21Raw = tri.v2 - tri.v1;
			const Vector3 Cross = v20Raw.Cross(v10Raw);
			if (Cross.SquareLength() <= 1e-12f)
			{
				continue;
			}

			Vector3 Nor = Cross.SafeUnit();
			const Vector3 v20 = v20Raw.SafeUnit();
			const Vector3 v10 = v10Raw.SafeUnit();
			const Vector3 v21 = v21Raw.SafeUnit();

			const float w0 = SafeAcos(v20.Dot(v10));
			const float w1 = SafeAcos(v21.Dot(-v10));
			const float w2 = SafeAcos(v21.Dot(v20));

			VertexNormals[tv.a] += w0 * Nor;
			Weight[tv.a] += w0;

			VertexNormals[tv.b] += w1 * Nor;
			Weight[tv.b] += w1;

			VertexNormals[tv.c] += w2 * Nor;
			Weight[tv.c] += w2;
		}

		for (size_t i = 0; i < VertexNormals.size(); ++i)
		{
			if (Weight[i] > 1e-12f)
			{
				VertexNormals[i] *= 1.0f / Weight[i];
				VertexNormals[i].SafeNormalize();
			}
			else
			{
				VertexNormals[i] = Vector3::UnitY();
			}
		}
	}



	int DynamicMesh::FixTriangleOrientation(bool right_handed)
	{
		int fix_count = 0;
		float sign = right_handed ? 1.0f : -1.0f;

		std::vector<std::vector<int>> islands;

		BitSet processed(Triangles.size());

		BuildIslands(islands);

		for (size_t i = 0; i < islands.size(); ++i)
		{
			const std::vector<int>& island = islands[i];

			if (island.size() <= 1)
			{
				continue;
			}

			Vector3 SeedNormal;
			int seed_tid = FindIslandSeedNormal(island, SeedNormal);

			if (seed_tid == -1)
			{
				continue;
			}

			assert(seed_tid != -1);
			Triangle3 tri;
			GetTriVertices(seed_tid, tri.v0, tri.v1, tri.v2);
			Vector3 TriNormal = tri.GetNormal(true);

			if (TriNormal.Dot(SeedNormal) * sign < 0)
			{
				ReverseTriOrientation(seed_tid);
				fix_count += 1;
			}

			struct TriLink
			{
				TriLink(int _id, int _parent)
				{
					id = _id;
					parent = _parent;
				}

				int id;
				int parent;
			};

			std::deque<TriLink> qu;
			qu.emplace_back(seed_tid, -1);

			processed.set(seed_tid, true);

			while (!qu.empty())
			{
				TriLink node = qu.front();
				qu.pop_front();

				if (node.parent != -1)
				{
					if (FindTriOrientation(Triangles[node.parent], Triangles[node.id]) < 0)
					{
						ReverseTriOrientation(node.id);
						fix_count += 1;
					}
				}

				const Index3& te = TriangleEdges[node.id];
				for (int j = 0; j < 3; ++j)
				{
					const Edge& e = Edges[te[j]];
					if (e.Tri[0] == node.parent || e.Tri[1] == node.parent)
					{
						continue;
					}

					int next = e.Tri[0] == node.id ? e.Tri[1] : e.Tri[0];
					if (next == -1)
					{
						continue;
					}

					if (processed.get(next))
					{
						continue;
					}
					processed.set(next, true);

					qu.emplace_back(next, node.id);
				}
			}
		}

		return fix_count;
	}

	void DynamicMesh::BuildIslands(std::vector<std::vector<int>>& islands)
	{
		islands.clear();

		int NumTriangles = GetTriangleCount();

		BitSet processed((size_t)NumTriangles);

		for (int tid = 0; tid < NumTriangles; ++tid)
		{
			int seed = tid;

			if (processed.get(seed))
			{
				continue;
			}
			processed.set(seed, true);

			std::vector<int> island;
			std::vector<int> queue;

			island.push_back(seed);
			queue.push_back(seed);

			while (!queue.empty())
			{
				int next = queue.back();
				queue.pop_back();

				Index3 te = TriangleEdges[next];

				for (int i = 0; i < 3; ++i)
				{
					const Edge &e = Edges[te[i]];

					for (int j = 0; j < 2; ++j)
					{
						int t_next = e.Tri[j];

						if (t_next == -1 || t_next == next || processed.get(t_next))
						{
							continue;
						}

						processed.set(t_next, true);
						island.push_back(t_next);
						queue.push_back(t_next);
					}
				}
			}

			islands.push_back(island);
		}
	}

	int DynamicMesh::FindIslandSeedNormal(const std::vector<int>& island, Vector3 &normal)
	{
		if (island.empty())
		{
			return -1;
		}

		BuildBounds();

		int i = -1, j = -1, max_axis = -1;

		Vector3 Origin;
		Vector3 Direction;

		{
			Triangle3 tri;
			GetTriVertices(island[0], tri.v0, tri.v1, tri.v2);

			Vector3 Nor = tri.GetNormal();

			max_axis = 0;
			float max_val = fabsf(Nor.x);
			float val = fabsf(Nor.y);
			if (val > max_val)
			{
				max_axis = 1;
				max_val = val;
			}
			val = fabsf(Nor.z);
			if (val > max_val)
			{
				max_axis = 2;
			}

			j = (max_axis + 1) % 3;
			i = (j + 1) % 3;

			Direction = Vector3::Zero();
			Direction[max_axis] = -1.0;

			Origin = tri.GetCenter();
			Origin[max_axis] = Bounds.Max[max_axis] + 1.0f;
		}

		int seed_tid = -1;
		float min_dist = FLT_MAX;
		for (int tid : island)
		{
			Triangle3 tri;
			GetTriVertices(tid, tri.v0, tri.v1, tri.v2);

			Box3 box = tri.GetBounds();

			if (Origin[i] < box.Min[i] || Origin[i] > box.Max[i] || Origin[j] < box.Min[j] || Origin[j] > box.Max[j])
			{
				continue;
			}

			float t;
			if (tri.IntersectRay(Origin, Direction, &t) && t < min_dist)
			{
				min_dist = t;
				seed_tid = tid;
				// normal = tri.GetNormal();
			}
		}

		normal = -Direction;
		return seed_tid;
	}

	Vector3 DynamicMesh::CalculateIslandCenter(const std::vector<int>& island)
	{
		Vector3 Center = Vector3::Zero();
		if (island.size() > 0)
		{
			for (int tid : island)
			{
				Triangle3 tri;
				GetTriVertices(tid, tri.v0, tri.v1, tri.v2);

				Vector3 c = tri.GetCenter();
				Center += c;
			}
			Center *= (1.0f / island.size());
		}

		return Center;
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

	int DynamicMeshAABBTree::FindNearestTriangle(const Vector3& P, float& NearestDistSqr, const FQueryOptions& Options /*= FQueryOptions()*/) const
	{
		NearestDistSqr = (Options.MaxDistance < FLT_MAX) ? Options.MaxDistance * Options.MaxDistance : FLT_MAX;
		int tNearID = -1;
		find_nearest_tri(RootIndex, P, NearestDistSqr, tNearID, Options);
		return tNearID;
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
				OtherTree.Mesh->GetTriVertices(tj, otri.v0, otri.v1, otri.v2);
				if (TransformF != nullptr)
				{
					otri.v0 = TransformF->LocalToWorld(otri.v0);
					otri.v1 = TransformF->LocalToWorld(otri.v1);
					otri.v2 = TransformF->LocalToWorld(otri.v2);
				}

				for (int i = 1; i <= num_tris; ++i)
				{
					int ti = IndexList[idx + i];
					Mesh->GetTriVertices(ti, Tri.v0, Tri.v1, Tri.v2);

					Triangle3::Triangle3IntersectionResult intr;

					if (otri.IntersectTriangle(Tri, intr))
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
			Box3 bounds = GetBoxEps(iBox);

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

	static float TriDistanceSqr(const DynamicMesh& Mesh, int TriIdx, const Vector3& Point)
	{
		Triangle3 Triangle;
		Mesh.GetTriVertices(TriIdx, Triangle[0], Triangle[1], Triangle[2]);

		Triangle3::PointDistanceQueryResult Result = Triangle.PointDistanceQuery(Point);
		return Result.SqrDistance;
	}

	bool DynamicMeshAABBTree::find_nearest_tri(int IBox, const Vector3& P, float& NearestDistSqr, int& TID, const FQueryOptions& Options) const
	{
		const bool bEarlyStop = false;
		int idx = BoxToIndex[IBox];
		if (idx < TrianglesEnd)
		{ // triangle-list case, array is [N t1 t2 ... tN]
			int num_tris = IndexList[idx];
			for (int i = 1; i <= num_tris; ++i)
			{
				int ti = IndexList[idx + i];
				if (Options.TriangleFilterF != nullptr && Options.TriangleFilterF(ti) == false)
				{
					continue;
				}
				float fTriDistSqr = TriDistanceSqr(*Mesh, ti, P);
				if (fTriDistSqr < NearestDistSqr)
				{
					NearestDistSqr = fTriDistSqr;
					TID = ti;
					if (bEarlyStop)
					{
						return true;
					}
				}
			}
		}
		else
		{ // internal node, either 1 or 2 child boxes
			int iChild1 = IndexList[idx];
			if (iChild1 < 0)
			{ // 1 child, descend if nearer than cur min-dist
				iChild1 = (-iChild1) - 1;
				float fChild1DistSqr = BoxDistanceSqr(iChild1, P);
				if (fChild1DistSqr <= NearestDistSqr)
				{
					bool bFoundEarly = find_nearest_tri(iChild1, P, NearestDistSqr, TID, Options);
					if (bEarlyStop && bFoundEarly)
					{
						return true;
					}
				}
			}
			else
			{ // 2 children, descend closest first
				iChild1 = iChild1 - 1;
				int iChild2 = IndexList[idx + 1] - 1;

				float fChild1DistSqr = BoxDistanceSqr(iChild1, P);
				float fChild2DistSqr = BoxDistanceSqr(iChild2, P);
				if (fChild1DistSqr < fChild2DistSqr)
				{
					if (fChild1DistSqr < NearestDistSqr)
					{
						bool bFoundEarly1 = find_nearest_tri(iChild1, P, NearestDistSqr, TID, Options);
						if (bEarlyStop && bFoundEarly1)
						{
							return true;
						}
						if (fChild2DistSqr < NearestDistSqr)
						{
							bool bFoundEarly2 = find_nearest_tri(iChild2, P, NearestDistSqr, TID, Options);
							if (bEarlyStop && bFoundEarly2)
							{
								return true;
							}
						}
					}
				}
				else
				{
					if (fChild2DistSqr < NearestDistSqr)
					{
						bool bFoundEarly1 = find_nearest_tri(iChild2, P, NearestDistSqr, TID, Options);
						if (bEarlyStop && bFoundEarly1)
						{
							return true;
						}
						if (fChild1DistSqr < NearestDistSqr)
						{
							bool bFoundEarly2 = find_nearest_tri(iChild1, P, NearestDistSqr, TID, Options);
							if (bEarlyStop && bFoundEarly2)
							{
								return true;
							}
						}
					}
				}
			}
		}
		return false;
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

	float DynamicMeshAABBTree::BoxDistanceSqr(int IBox, const Vector3& V) const
	{
		const Vector3& c = AABB[IBox].GetCenter();
		const Vector3& e = AABB[IBox].GetExtent();

		const float dx = std::max(fabs(V.x - c.x) - e.x, 0.0f);
		const float dy = std::max(fabs(V.y - c.y) - e.y, 0.0f);
		const float dz = std::max(fabs(V.z - c.z) - e.z, 0.0f);
		return dx * dx + dy * dy + dz * dz;
	}

	void DynamicMeshAABBTree::Build()
	{
		std::vector<int> Triangles;
		std::vector<Vector3> Centers;
		int numTris = Mesh->GetTriangleCount();
		for (int i = 0 ; i < numTris; ++i)
		{
			if (!Mesh->IsTriangleFast(i))
			{
				continue;
			}

			const Vector3 centroid = Mesh->GetTriCentroid((int)i);
			Triangles.push_back(i);
			Centers.push_back(centroid);
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
				Box.Encapsulate(Mesh->GetTriBounds(Triangles[IStart + i]));
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



	void FDynamicMeshEditor::ReverseTriangleOrientations(const std::vector<int>& Triangles, bool bInvertNormals)
	{
		for (int tid : Triangles)
		{
			Mesh->ReverseTriOrientation(tid);
		}
		if (bInvertNormals)
		{
			InvertTriangleNormals(Triangles);
		}
	}

	void FDynamicMeshEditor::InvertTriangleNormals(const std::vector<int>& Triangles)
	{
		// @todo re-use the TBitA

		if (Mesh->HasVertexNormals())
		{
			std::vector<bool> DoneVertices(Mesh->GetVertexCount());
			for (int TriangleID : Triangles)
			{
				Index3 Tri = Mesh->GetTriangle(TriangleID);
				for (int j = 0; j < 3; ++j)
				{
					if (DoneVertices[Tri[j]] == false)
					{
						Mesh->SetVertexNormal(Tri[j], -Mesh->GetVertexNormal(Tri[j]));
						DoneVertices[Tri[j]] = true;
					}
				}
			}
		}


		if (Mesh->HasAttributes())
		{
			for (int NormalLayerIndex = 0; NormalLayerIndex < Mesh->Attributes()->NumNormalLayers(); NormalLayerIndex++)
			{
				FDynamicMeshNormalOverlay* NormalOverlay = Mesh->Attributes()->GetNormalLayer(NormalLayerIndex);
				std::vector<bool> DoneNormals(NormalOverlay->MaxElementID());
				for (int TriangleID : Triangles)
				{
					Index3 ElemTri = NormalOverlay->GetTriangle(TriangleID);
					for (int j = 0; j < 3; ++j)
					{
						if (NormalOverlay->IsElement(ElemTri[j]) && DoneNormals[ElemTri[j]] == false)
						{
							NormalOverlay->SetElement(ElemTri[j], -NormalOverlay->GetElement(ElemTri[j]));
							DoneNormals[ElemTri[j]] = true;
						}
					}
				}
			}
		}
	}


	void FDynamicMeshEditor::AppendMesh(const DynamicMesh* AppendMesh,
		FMeshIndexMappings& IndexMapsOut,
		std::function<Vector3(int, const Vector3&)> PositionTransform,
		std::function<Vector3(int, const Vector3&)> NormalTransform)
	{
		// todo: handle this case by making a copy?
		assert(AppendMesh != Mesh);

		IndexMapsOut.Reset();
		IndexMapsOut.Initialize(Mesh);

		FIndexMapi& VertexMap = IndexMapsOut.GetVertexMap();
		VertexMap.Reserve(AppendMesh->GetVertexCount());
		for (int VertID = 0; VertID < AppendMesh->GetVertexCount(); ++VertID)
		{
			if (!AppendMesh->IsVertexFast(VertID))
			{
				continue;
			}

			Vector3 Position = AppendMesh->GetVertex(VertID);
			if (PositionTransform != nullptr)
			{
				Position = PositionTransform(VertID, Position);
			}
			int NewVertID = Mesh->AppendVertex(Position);
			VertexMap.Add(VertID, NewVertID);

			if (AppendMesh->HasVertexNormals() && Mesh->HasVertexNormals())
			{
				Vector3 Normal = AppendMesh->GetVertexNormal(VertID);
				if (NormalTransform != nullptr)
				{
					Normal = (Vector3)NormalTransform(VertID, (Vector3)Normal);
				}
				Mesh->SetVertexNormal(NewVertID, Normal);
			}

			if (AppendMesh->HasVertexUVs() && Mesh->HasVertexUVs())
			{
				Vector2 UV = AppendMesh->GetVertexUV(VertID);
				Mesh->SetVertexUV(NewVertID, UV);
			}

			if (AppendMesh->HasVertexColors() && Mesh->HasVertexColors())
			{
				Vector3 Color = AppendMesh->GetVertexColor(VertID);
				Mesh->SetVertexColor(NewVertID, Color);
			}
		}

		FIndexMapi& TriangleMap = IndexMapsOut.GetTriangleMap();
		bool bAppendGroups = AppendMesh->HasTriangleGroups() && Mesh->HasTriangleGroups();
		FIndexMapi& GroupsMap = IndexMapsOut.GetGroupMap();

		for (int TriID = 0; TriID < AppendMesh->GetTriangleCount(); ++TriID)
		{
			if (!AppendMesh->IsTriangleFast(TriID))
			{
				continue;
			}

			// append trigroup
			int GroupID = -1;
			if (bAppendGroups)
			{
				GroupID = AppendMesh->GetTriangleGroup(TriID);
				if (GroupID != -1)
				{
					const int* FoundNewGroupID = GroupsMap.FindTo(GroupID);
					if (FoundNewGroupID == nullptr)
					{
						int NewGroupID = Mesh->AllocateTriangleGroup();
						GroupsMap.Add(GroupID, NewGroupID);
						GroupID = NewGroupID;
					}
					else
					{
						GroupID = *FoundNewGroupID;
					}
				}
			}

			Index3 Tri = AppendMesh->GetTriangle(TriID);
			int NewTriID = Mesh->AppendTriangle(Index3(VertexMap.GetTo(Tri.a), VertexMap.GetTo(Tri.b), VertexMap.GetTo(Tri.c)), GroupID);
			if (NewTriID >= 0)
			{
				TriangleMap.Add(TriID, NewTriID);
			}
		}

		// @todo can we have a template fn that does this?

		if (AppendMesh->HasAttributes() && Mesh->HasAttributes())
		{
			int NumNormalLayers = std::min(Mesh->Attributes()->NumNormalLayers(), AppendMesh->Attributes()->NumNormalLayers());
			for (int NormalLayerIndex = 0; NormalLayerIndex < NumNormalLayers; NormalLayerIndex++)
			{
				const FDynamicMeshNormalOverlay* FromNormals = AppendMesh->Attributes()->GetNormalLayer(NormalLayerIndex);
				FDynamicMeshNormalOverlay* ToNormals = Mesh->Attributes()->GetNormalLayer(NormalLayerIndex);
				if (FromNormals != nullptr && ToNormals != nullptr)
				{
					FIndexMapi& NormalMap = IndexMapsOut.GetNormalMap(NormalLayerIndex);
					NormalMap.Reserve(FromNormals->ElementCount());
					AppendNormals(AppendMesh, FromNormals, ToNormals,
						VertexMap, TriangleMap, NormalTransform, NormalMap);
				}
			}

			int NumUVLayers = std::min(Mesh->Attributes()->NumUVLayers(), AppendMesh->Attributes()->NumUVLayers());
			for (int UVLayerIndex = 0; UVLayerIndex < NumUVLayers; UVLayerIndex++)
			{
				const FDynamicMeshUVOverlay* FromUVs = AppendMesh->Attributes()->GetUVLayer(UVLayerIndex);
				FDynamicMeshUVOverlay* ToUVs = Mesh->Attributes()->GetUVLayer(UVLayerIndex);
				if (FromUVs != nullptr && ToUVs != nullptr)
				{
					FIndexMapi& UVMap = IndexMapsOut.GetUVMap(UVLayerIndex);
					UVMap.Reserve(FromUVs->ElementCount());
					AppendUVs(AppendMesh, FromUVs, ToUVs,
						VertexMap, TriangleMap, UVMap);
				}
			}

			if (AppendMesh->Attributes()->HasMaterialID() && Mesh->Attributes()->HasMaterialID())
			{
				const FDynamicMeshMaterialAttribute* FromMaterialIDs = AppendMesh->Attributes()->GetMaterialID();
				FDynamicMeshMaterialAttribute* ToMaterialIDs = Mesh->Attributes()->GetMaterialID();
				for (const std::pair<int, int>& MapTID : TriangleMap.GetForwardMap())
				{
					ToMaterialIDs->SetValue(MapTID.second, FromMaterialIDs->GetValue(MapTID.first));
				}
			}

			if (AppendMesh->Attributes()->HasPrimaryColors() && Mesh->Attributes()->HasPrimaryColors())
			{
				const FDynamicMeshColorOverlay* FromColors = AppendMesh->Attributes()->PrimaryColors();
				FDynamicMeshColorOverlay* ToColors = Mesh->Attributes()->PrimaryColors();
				if (FromColors != nullptr && ToColors != nullptr)
				{
					FIndexMapi& ColorMap = IndexMapsOut.GetColorMap();
					ColorMap.Reserve(FromColors->ElementCount());
					AppendColors(AppendMesh, FromColors, ToColors,
						VertexMap, TriangleMap, ColorMap);
				}
			}

			int NumWeightLayers = std::min(Mesh->Attributes()->NumWeightLayers(), AppendMesh->Attributes()->NumWeightLayers());
			for (int WeightLayerIndex = 0; WeightLayerIndex < NumWeightLayers; WeightLayerIndex++)
			{
				const FDynamicMeshWeightAttribute* FromWeights = AppendMesh->Attributes()->GetWeightLayer(WeightLayerIndex);
				FDynamicMeshWeightAttribute* ToWeights = Mesh->Attributes()->GetWeightLayer(WeightLayerIndex);
				for (const std::pair<int, int>& MapVID : VertexMap.GetForwardMap())
				{
					float Weight;
					FromWeights->GetValue(MapVID.first, &Weight);
					ToWeights->SetValue(MapVID.second, &Weight);
				}
			}

			/*
			int NumPolygroupLayers = std::min(Mesh->Attributes()->NumPolygroupLayers(), AppendMesh->Attributes()->NumPolygroupLayers());
			for (int PolygroupLayerIndex = 0; PolygroupLayerIndex < NumPolygroupLayers; PolygroupLayerIndex++)
			{
				// TODO: remap groups? this will be somewhat expensive...
				const FDynamicMeshPolygroupAttribute* FromPolygroups = AppendMesh->Attributes()->GetPolygroupLayer(PolygroupLayerIndex);
				FDynamicMeshPolygroupAttribute* ToPolygroups = Mesh->Attributes()->GetPolygroupLayer(PolygroupLayerIndex);
				for (const std::pair<int, int>& MapTID : TriangleMap.GetForwardMap())
				{
					ToPolygroups->SetValue(MapTID.second, FromPolygroups->GetValue(MapTID.first));
				}
			}

			if (AppendMesh->Attributes()->HasBones() && Mesh->Attributes()->HasBones())
			{
				const bool bSameSkeletons = AppendMesh->Attributes()->GetBoneNames()->IsSameAs(*Mesh->Attributes()->GetBoneNames());

				if (!bSameSkeletons)
				{
					Mesh->Attributes()->AppendBonesUnique(*AppendMesh->Attributes());
				}

				for (const std::pair<std::string, TUniquePtr<FDynamicMeshVertexSkinWeightsAttribute>>& AttribPair : AppendMesh->Attributes()->GetSkinWeightsAttributes())
				{
					FDynamicMeshVertexSkinWeightsAttribute* ToAttrib = Mesh->Attributes()->GetSkinWeightsAttribute(AttribPair.first);
					if (ToAttrib)
					{
						if (bSameSkeletons)
						{
							// If the skeletons are the same then we do not need to re-index and we simply copy
							ToAttrib->CopyThroughMapping(AttribPair.second.Get(), IndexMapsOut);
						}
						else
						{
							// Make a copy of the append mesh skinning weights and reindex them with respect to the new skeleton
							FDynamicMeshVertexSkinWeightsAttribute CopyAppendMeshAttrib;
							CopyAppendMeshAttrib.Copy(*AttribPair.second.Get());
							CopyAppendMeshAttrib.ReindexBoneIndicesToSkeleton(AppendMesh->Attributes()->GetBoneNames()->GetAttribValues(),
								Mesh->Attributes()->GetBoneNames()->GetAttribValues());

							// Now copy re-indexed weights to the mesh we are appending to
							ToAttrib->CopyThroughMapping(&CopyAppendMeshAttrib, IndexMapsOut);
						}
					}
				}
			}
			else
			{
				for (const std::pair<std::string, TUniquePtr<FDynamicMeshVertexSkinWeightsAttribute>>& AttribPair : AppendMesh->Attributes()->GetSkinWeightsAttributes())
				{
					FDynamicMeshVertexSkinWeightsAttribute* ToAttrib = Mesh->Attributes()->GetSkinWeightsAttribute(AttribPair.first);
					if (ToAttrib)
					{
						ToAttrib->CopyThroughMapping(AttribPair.second.Get(), IndexMapsOut);
					}
				}
			}

			for (const std::pair<std::string, TUniquePtr<FDynamicMeshAttributeBase>>& AttribPair : AppendMesh->Attributes()->GetAttachedAttributes())
			{
				if (Mesh->Attributes()->HasAttachedAttribute(AttribPair.first))
				{
					FDynamicMeshAttributeBase* ToAttrib = Mesh->Attributes()->GetAttachedAttribute(AttribPair.first);
					ToAttrib->CopyThroughMapping(AttribPair.second.Get(), IndexMapsOut);
				}
			}
			*/
		}
	}


	void FDynamicMeshEditor::AppendNormals(const DynamicMesh* AppendMesh,
		const FDynamicMeshNormalOverlay* FromNormals, FDynamicMeshNormalOverlay* ToNormals,
		const FIndexMapi& VertexMap, const FIndexMapi& TriangleMap,
		std::function<Vector3(int, const Vector3&)> NormalTransform,
		FIndexMapi& NormalMapOut)
	{
		// copy over normals
		for (int ElemID = 0; ElemID < FromNormals->ElementCount(); ++ElemID)
		{
			if (!FromNormals->IsElement(ElemID))
			{
				continue;
			}

			int ParentVertID = FromNormals->GetParentVertex(ElemID);
			Vector3 Normal = FromNormals->GetElement(ElemID);
			if (NormalTransform != nullptr)
			{
				Normal = (Vector3)NormalTransform(ParentVertID, (Vector3)Normal);
			}
			int NewElemID = ToNormals->AppendElement(Normal);
			NormalMapOut.Add(ElemID, NewElemID);
		}

		// now set new triangles
		for (const std::pair<int, int>& MapTID : TriangleMap.GetForwardMap())
		{
			if (FromNormals->IsSetTriangle(MapTID.first))
			{
				Index3 ElemTri = FromNormals->GetTriangle(MapTID.first);
				for (int j = 0; j < 3; ++j)
				{
					ElemTri[j] = FromNormals->IsElement(ElemTri[j]) ? NormalMapOut.GetTo(ElemTri[j]) : -1;
				}
				ToNormals->SetTriangle(MapTID.second, ElemTri);
			}
		}
	}


	void FDynamicMeshEditor::AppendUVs(const DynamicMesh* AppendMesh,
		const FDynamicMeshUVOverlay* FromUVs, FDynamicMeshUVOverlay* ToUVs,
		const FIndexMapi& VertexMap, const FIndexMapi& TriangleMap,
		FIndexMapi& UVMapOut)
	{
		// copy over uv elements		
		for (int ElemID = 0; ElemID < FromUVs->ElementCount(); ++ElemID)
		{
			if (!FromUVs->IsElement(ElemID))
			{
				continue;
			}

			Vector2 UV = FromUVs->GetElement(ElemID);
			int NewElemID = ToUVs->AppendElement(UV);
			UVMapOut.Add(ElemID, NewElemID);
		}

		// now set new triangles
		for (const std::pair<int, int>& MapTID : TriangleMap.GetForwardMap())
		{
			if (FromUVs->IsSetTriangle(MapTID.first))
			{
				Index3 ElemTri = FromUVs->GetTriangle(MapTID.first);
				for (int j = 0; j < 3; ++j)
				{
					ElemTri[j] = FromUVs->IsElement(ElemTri[j]) ? UVMapOut.GetTo(ElemTri[j]) : -1;
				}
				ToUVs->SetTriangle(MapTID.second, ElemTri);
			}
		}
	}


	void FDynamicMeshEditor::AppendColors(const DynamicMesh* AppendMesh,
		const FDynamicMeshColorOverlay* FromOverlay, FDynamicMeshColorOverlay* ToOverlay,
		const FIndexMapi& VertexMap, const FIndexMapi& TriangleMap,
		FIndexMapi& MapOut)
	{
		// copy over color elements
		for (int ElemID = 0; ElemID < FromOverlay->ElementCount(); ++ElemID)
		{
			if (!FromOverlay->IsElement(ElemID))
			{
				continue;
			}
			int NewElemID = ToOverlay->AppendElement(FromOverlay->GetElement(ElemID));
			MapOut.Add(ElemID, NewElemID);
		}

		// now set new triangles
		for (const std::pair<int, int>& MapTID : TriangleMap.GetForwardMap())
		{
			if (FromOverlay->IsSetTriangle(MapTID.first))
			{
				Index3 ElemTri = FromOverlay->GetTriangle(MapTID.first);
				for (int j = 0; j < 3; ++j)
				{
					ElemTri[j] = FromOverlay->IsElement(ElemTri[j]) ? MapOut.GetTo(ElemTri[j]) : -1;
				}
				ToOverlay->SetTriangle(MapTID.second, ElemTri);
			}
		}
	}

	static int AppendTriangleUVAttribute(const DynamicMesh* FromMesh, int FromElementID, DynamicMesh* ToMesh, int UVLayerIndex, FMeshIndexMappings& IndexMaps)
	{
		int NewElementID = IndexMaps.GetNewUV(UVLayerIndex, FromElementID);
		if (NewElementID == IndexMaps.InvalidID())
		{
			const FDynamicMeshUVOverlay* FromUVOverlay = FromMesh->Attributes()->GetUVLayer(UVLayerIndex);
			FDynamicMeshUVOverlay* ToUVOverlay = ToMesh->Attributes()->GetUVLayer(UVLayerIndex);
			NewElementID = ToUVOverlay->AppendElement(FromUVOverlay->GetElement(FromElementID));
			IndexMaps.SetUV(UVLayerIndex, FromElementID, NewElementID);
		}
		return NewElementID;
	}

	static int AppendTriangleNormalAttribute(const DynamicMesh* FromMesh, int FromElementID, DynamicMesh* ToMesh, int NormalLayerIndex, FMeshIndexMappings& IndexMaps)
	{
		int NewElementID = IndexMaps.GetNewNormal(NormalLayerIndex, FromElementID);
		if (NewElementID == IndexMaps.InvalidID())
		{
			const FDynamicMeshNormalOverlay* FromNormalOverlay = FromMesh->Attributes()->GetNormalLayer(NormalLayerIndex);
			FDynamicMeshNormalOverlay* ToNormalOverlay = ToMesh->Attributes()->GetNormalLayer(NormalLayerIndex);
			NewElementID = ToNormalOverlay->AppendElement(FromNormalOverlay->GetElement(FromElementID));
			IndexMaps.SetNormal(NormalLayerIndex, FromElementID, NewElementID);
		}
		return NewElementID;
	}

	static int AppendTriangleColorAttribute(const DynamicMesh* FromMesh, int FromElementID, DynamicMesh* ToMesh, FMeshIndexMappings& IndexMaps)
	{
		int NewElementID = IndexMaps.GetNewColor(FromElementID);
		if (NewElementID == IndexMaps.InvalidID())
		{
			const FDynamicMeshColorOverlay* FromOverlay = FromMesh->Attributes()->PrimaryColors();
			FDynamicMeshColorOverlay* ToOverlay = ToMesh->Attributes()->PrimaryColors();
			NewElementID = ToOverlay->AppendElement(FromOverlay->GetElement(FromElementID));
			IndexMaps.SetColor(FromElementID, NewElementID);
		}
		return NewElementID;
	}

	static void AppendTriangleAttributes(const DynamicMesh* FromMesh, int FromTriangleID, DynamicMesh* ToMesh, int ToTriangleID, FMeshIndexMappings& IndexMaps, FDynamicMeshEditResult& ResultOut)
	{
		if (FromMesh->HasAttributes() == false || ToMesh->HasAttributes() == false)
		{
			return;
		}

		for (int UVLayerIndex = 0; UVLayerIndex < std::min(FromMesh->Attributes()->NumUVLayers(), ToMesh->Attributes()->NumUVLayers()); UVLayerIndex++)
		{
			const FDynamicMeshUVOverlay* FromUVOverlay = FromMesh->Attributes()->GetUVLayer(UVLayerIndex);
			FDynamicMeshUVOverlay* ToUVOverlay = ToMesh->Attributes()->GetUVLayer(UVLayerIndex);
			if (FromUVOverlay->IsSetTriangle(FromTriangleID))
			{
				Index3 FromElemTri = FromUVOverlay->GetTriangle(FromTriangleID);
				Index3 ToElemTri = ToUVOverlay->GetTriangle(ToTriangleID);
				for (int j = 0; j < 3; ++j)
				{
					assert(FromElemTri[j] != -1);
					int NewElemID = AppendTriangleUVAttribute(FromMesh, FromElemTri[j], ToMesh, UVLayerIndex, IndexMaps);
					ToElemTri[j] = NewElemID;
				}
				ToUVOverlay->SetTriangle(ToTriangleID, ToElemTri);
			}
		}

		for (int NormalLayerIndex = 0; NormalLayerIndex < std::min(FromMesh->Attributes()->NumNormalLayers(), ToMesh->Attributes()->NumNormalLayers()); NormalLayerIndex++)
		{
			const FDynamicMeshNormalOverlay* FromNormalOverlay = FromMesh->Attributes()->GetNormalLayer(NormalLayerIndex);
			FDynamicMeshNormalOverlay* ToNormalOverlay = ToMesh->Attributes()->GetNormalLayer(NormalLayerIndex);
			if (FromNormalOverlay->IsSetTriangle(FromTriangleID))
			{
				Index3 FromElemTri = FromNormalOverlay->GetTriangle(FromTriangleID);
				Index3 ToElemTri = ToNormalOverlay->GetTriangle(ToTriangleID);
				for (int j = 0; j < 3; ++j)
				{
					assert(FromElemTri[j] != -1);
					int NewElemID = AppendTriangleNormalAttribute(FromMesh, FromElemTri[j], ToMesh, NormalLayerIndex, IndexMaps);
					ToElemTri[j] = NewElemID;
				}
				ToNormalOverlay->SetTriangle(ToTriangleID, ToElemTri);
			}
		}

		if (FromMesh->Attributes()->HasPrimaryColors() && ToMesh->Attributes()->HasPrimaryColors())
		{
			const FDynamicMeshColorOverlay* FromOverlay = FromMesh->Attributes()->PrimaryColors();
			FDynamicMeshColorOverlay* ToOverlay = ToMesh->Attributes()->PrimaryColors();
			if (FromOverlay->IsSetTriangle(FromTriangleID))
			{
				Index3 FromElemTri = FromOverlay->GetTriangle(FromTriangleID);
				Index3 ToElemTri = ToOverlay->GetTriangle(ToTriangleID);
				for (int j = 0; j < 3; ++j)
				{
					assert(FromElemTri[j] != -1);
					int NewElemID = AppendTriangleColorAttribute(FromMesh, FromElemTri[j], ToMesh, IndexMaps);
					ToElemTri[j] = NewElemID;
				}
				ToOverlay->SetTriangle(ToTriangleID, ToElemTri);
			}
		}

		if (FromMesh->Attributes()->HasMaterialID() && ToMesh->Attributes()->HasMaterialID())
		{
			const FDynamicMeshMaterialAttribute* FromMaterialIDs = FromMesh->Attributes()->GetMaterialID();
			FDynamicMeshMaterialAttribute* ToMaterialIDs = ToMesh->Attributes()->GetMaterialID();
			ToMaterialIDs->SetValue(ToTriangleID, FromMaterialIDs->GetValue(FromTriangleID));
		}

		//int NumPolygroupLayers = std::min(FromMesh->Attributes()->NumPolygroupLayers(), ToMesh->Attributes()->NumPolygroupLayers());
		//for (int PolygroupLayerIndex = 0; PolygroupLayerIndex < NumPolygroupLayers; PolygroupLayerIndex++)
		//{
		//	// TODO: remap groups? this will be somewhat expensive...
		//	const FDynamicMeshPolygroupAttribute* FromPolygroups = FromMesh->Attributes()->GetPolygroupLayer(PolygroupLayerIndex);
		//	FDynamicMeshPolygroupAttribute* ToPolygroups = ToMesh->Attributes()->GetPolygroupLayer(PolygroupLayerIndex);
		//	ToPolygroups->SetValue(ToTriangleID, FromPolygroups->GetValue(FromTriangleID));
		//}
	}

	static void AppendVertexAttributes(const DynamicMesh* FromMesh, DynamicMesh* ToMesh, FMeshIndexMappings& IndexMaps)
	{
		if (FromMesh->HasAttributes() == false || ToMesh->HasAttributes() == false)
		{
			return;
		}

		int NumWeightLayers = std::min(FromMesh->Attributes()->NumWeightLayers(), ToMesh->Attributes()->NumWeightLayers());
		for (int WeightLayerIndex = 0; WeightLayerIndex < NumWeightLayers; WeightLayerIndex++)
		{
			const FDynamicMeshWeightAttribute* FromWeights = FromMesh->Attributes()->GetWeightLayer(WeightLayerIndex);
			FDynamicMeshWeightAttribute* ToWeights = ToMesh->Attributes()->GetWeightLayer(WeightLayerIndex);
			for (const std::pair<int, int>& MapVID : IndexMaps.GetVertexMap().GetForwardMap())
			{
				float Weight;
				FromWeights->GetValue(MapVID.first, &Weight);
				ToWeights->SetValue(MapVID.second, &Weight);
			}
		}

		/*
		// Copy skin weight and generic attributes after full IndexMaps have been created. 	
		for (const std::pair<std::string, TUniquePtr<FDynamicMeshVertexSkinWeightsAttribute>>& AttribPair : FromMesh->Attributes()->GetSkinWeightsAttributes())
		{
			FDynamicMeshVertexSkinWeightsAttribute* ToAttrib = ToMesh->Attributes()->GetSkinWeightsAttribute(AttribPair.first);
			if (ToAttrib)
			{
				ToAttrib->CopyThroughMapping(AttribPair.second.Get(), IndexMaps);
			}
		}

		for (const std::pair<std::string, TUniquePtr<FDynamicMeshAttributeBase>>& AttribPair : FromMesh->Attributes()->GetAttachedAttributes())
		{
			if (ToMesh->Attributes()->HasAttachedAttribute(AttribPair.first))
			{
				FDynamicMeshAttributeBase* ToAttrib = ToMesh->Attributes()->GetAttachedAttribute(AttribPair.first);
				ToAttrib->CopyThroughMapping(AttribPair.second.Get(), IndexMaps);
			}
		}
		*/
	}

	void FDynamicMeshEditor::AppendTriangles(const DynamicMesh* SourceMesh, const std::vector<int>& SourceTriangles, FMeshIndexMappings& IndexMaps, FDynamicMeshEditResult& ResultOut, bool bComputeTriangleMap)
	{
		ResultOut.Reset();
		IndexMaps.Initialize(Mesh);

		int DefaultGroupID = -1;
		for (int SourceTriangleID : SourceTriangles)
		{
			assert(SourceMesh->IsTriangle(SourceTriangleID));
			if (SourceMesh->IsTriangle(SourceTriangleID) == false)
			{
				continue;	// ignore missing triangles
			}

			Index3 Tri = SourceMesh->GetTriangle(SourceTriangleID);

			// FindOrCreateDuplicateGroup
			int NewGroupID = -1;
			if (Mesh->HasTriangleGroups())
			{
				if (SourceMesh->HasTriangleGroups())
				{
					int SourceGroupID = SourceMesh->GetTriangleGroup(SourceTriangleID);
					if (SourceGroupID >= 0)
					{
						NewGroupID = IndexMaps.GetNewGroup(SourceGroupID);
						if (NewGroupID == IndexMaps.InvalidID())
						{
							NewGroupID = Mesh->AllocateTriangleGroup();
							IndexMaps.SetGroup(SourceGroupID, NewGroupID);
							ResultOut.NewGroups.push_back(NewGroupID);
						}
					}
				}
				else
				{
					// If the source mesh does not have triangle groups, but the destination
					// mesh does, create a default group for all triangles.
					if (DefaultGroupID == -1)
					{
						DefaultGroupID = Mesh->AllocateTriangleGroup();
						ResultOut.NewGroups.push_back(DefaultGroupID);
					}
					NewGroupID = DefaultGroupID;
				}
			}

			// FindOrCreateDuplicateVertex
			Index3 NewTri;
			for (int j = 0; j < 3; ++j)
			{
				int SourceVertexID = Tri[j];
				int NewVertexID = IndexMaps.GetNewVertex(SourceVertexID);
				if (NewVertexID == IndexMaps.InvalidID())
				{
					NewVertexID = Mesh->AppendVertex(*SourceMesh, SourceVertexID);
					IndexMaps.SetVertex(SourceVertexID, NewVertexID);
					ResultOut.NewVertices.push_back(NewVertexID);
				}
				NewTri[j] = NewVertexID;
			}

			int NewTriangleID = Mesh->AppendTriangle(NewTri, NewGroupID);
			if (bComputeTriangleMap)
			{
				IndexMaps.SetTriangle(SourceTriangleID, NewTriangleID);
			}
			ResultOut.NewTriangles.push_back(NewTriangleID);

			AppendTriangleAttributes(SourceMesh, SourceTriangleID, Mesh, NewTriangleID, IndexMaps, ResultOut);

			//Mesh->CheckValidity(true);
		}

		AppendVertexAttributes(SourceMesh, Mesh, IndexMaps);

	}

	void FDynamicMeshEditResult::GetAllTriangles(std::vector<int>& TrianglesOut) const
	{
		TrianglesOut.insert(TrianglesOut.end(), NewTriangles.begin(), NewTriangles.end());
		for (const Index2& quad : NewQuads)
		{
			TrianglesOut.push_back(quad.a);
			TrianglesOut.push_back(quad.b);
		}
		for (const std::vector<int>& polygon : NewPolygons)
		{
			TrianglesOut.insert(TrianglesOut.end(), polygon.begin(), polygon.end());
		}
	}

	bool FDynamicMeshEditor::RemoveIsolatedVertices()
	{
		bool bSuccess = true;
		const int VertexCount = Mesh->GetVertexCount();
		for (int VertexID = 0; VertexID < VertexCount; ++VertexID)
		{
			if (Mesh->IsVertexFast(VertexID) && !Mesh->IsReferencedVertex(VertexID))
			{
				constexpr bool bPreserveManifold = false;
				bSuccess = (Mesh->RemoveVertex(VertexID, bPreserveManifold) == EMeshResult::Ok) && bSuccess;
			}
		}
		return bSuccess;
	}

	namespace DynamicMeshEditorLocal
	{
		template<typename TGetQuadVertices>
		bool StitchLoopsInternal(FDynamicMeshEditor& Editor, int NumQuads, TGetQuadVertices&& GetQuadVertices, FDynamicMeshEditResult& ResultOut)
		{
			ResultOut.NewQuads.reserve(ResultOut.NewQuads.size() + NumQuads);
			ResultOut.NewGroups.reserve(ResultOut.NewGroups.size() + NumQuads);

			for (int i = 0; i < NumQuads; ++i)
			{
				int a, b, c, d;
				GetQuadVertices(i, a, b, c, d);

				const int NewGroupID = Editor.Mesh->AllocateTriangleGroup();
				ResultOut.NewGroups.push_back(NewGroupID);

				const int tid1 = Editor.Mesh->AppendTriangle(Index3(b, a, d), NewGroupID);
				const int tid2 = Editor.Mesh->AppendTriangle(Index3(a, c, d), NewGroupID);
				ResultOut.NewQuads.push_back(Index2(tid1, tid2));

				if (tid1 < 0 || tid2 < 0)
				{
					std::vector<int> Triangles;
					ResultOut.GetAllTriangles(Triangles);
					Editor.RemoveTriangles(Triangles, false);
					return false;
				}
			}

			return true;
		}

		static float Distance(const Vector3& A, const Vector3& B)
		{
			return (A - B).Length();
		}

		static float TriangleArea(const DynamicMesh& Mesh, int TriangleID)
		{
			Vector3 a, b, c;
			Mesh.GetTriVertices(TriangleID, a, b, c);
			return 0.5f * (b - a).Cross(c - a).Length();
		}

		static double SignedTriangleVolume(const DynamicMesh& Mesh, int TriangleID)
		{
			Vector3 a, b, c;
			Mesh.GetTriVertices(TriangleID, a, b, c);
			return (double)a.Dot(b.Cross(c)) / 6.0;
		}

		static FDynamicMeshNormalOverlay* EnsureNormalOverlay(DynamicMesh* Mesh)
		{
			Mesh->EnableAttributes();
			if (Mesh->Attributes()->NumNormalLayers() == 0)
			{
				Mesh->Attributes()->SetNumNormalLayers(1);
			}
			return Mesh->Attributes()->PrimaryNormals();
		}

		static FDynamicMeshUVOverlay* EnsureUVOverlay(DynamicMesh* Mesh, int UVLayerIndex)
		{
			Mesh->EnableAttributes();
			if (Mesh->Attributes()->NumUVLayers() <= UVLayerIndex)
			{
				Mesh->Attributes()->SetNumUVLayers(UVLayerIndex + 1);
			}
			return Mesh->Attributes()->GetUVLayer(UVLayerIndex);
		}

		static Vector3 AverageSafe(const Vector3& A, const Vector3& B)
		{
			Vector3 Result = A + B;
			if (Result.SquareLength() <= 1e-12f)
			{
				Result = A.SquareLength() > B.SquareLength() ? A : B;
			}
			return Result.SafeUnit();
		}
	}

	bool FDynamicMeshEditor::StitchVertexLoopsMinimal(const std::vector<int>& Loop1, const std::vector<int>& Loop2, FDynamicMeshEditResult& ResultOut)
	{
		const int N = (int)Loop1.size();
		if (N != (int)Loop2.size())
		{
			return false;
		}

		return DynamicMeshEditorLocal::StitchLoopsInternal(*this, N,
			[&Loop1, &Loop2, N](int Index, int& VertA, int& VertB, int& VertC, int& VertD)
			{
				VertA = Loop1[Index];
				VertB = Loop1[(Index + 1) % N];
				VertC = Loop2[Index];
				VertD = Loop2[(Index + 1) % N];
			},
			ResultOut);
	}

	bool FDynamicMeshEditor::StitchVertexLoopToTriVidPairSequence(
		const std::vector<std::pair<int, std::pair<int8_t, int8_t>>>& TriVidPairs,
		const std::vector<int>& VertexLoop,
		FDynamicMeshEditResult& ResultOut)
	{
		const int N = (int)TriVidPairs.size();
		if (N != (int)VertexLoop.size())
		{
			return false;
		}

		return DynamicMeshEditorLocal::StitchLoopsInternal(*this, N,
			[this, &TriVidPairs, &VertexLoop, N](int Index, int& VertA, int& VertB, int& VertC, int& VertD)
			{
				const Index3 TriVids = Mesh->GetTriangle(TriVidPairs[Index].first);
				VertA = TriVids[TriVidPairs[Index].second.first];
				VertB = TriVids[TriVidPairs[Index].second.second];
				VertC = VertexLoop[Index];
				VertD = VertexLoop[(Index + 1) % N];
			},
			ResultOut);
	}

	bool FDynamicMeshEditor::ConvertLoopToTriVidPairSequence(
		const DynamicMesh& Mesh,
		const std::vector<int>& VidLoop,
		const std::vector<int>& EdgeLoop,
		std::vector<std::pair<int, std::pair<int8_t, int8_t>>>& TriVertPairsOut)
	{
		if (EdgeLoop.size() != VidLoop.size())
		{
			return false;
		}

		for (int QuadIndex = 0; QuadIndex < (int)EdgeLoop.size(); ++QuadIndex)
		{
			const int Tid = Mesh.GetEdgeT(EdgeLoop[QuadIndex]).a;
			const int FirstVid = VidLoop[QuadIndex];
			const int SecondVid = VidLoop[(QuadIndex + 1) % VidLoop.size()];
			const Index3 TriVids = Mesh.GetTriangle(Tid);
			const int SubIdx1 = TriVids.IndexOf(FirstVid);
			const int SubIdx2 = TriVids.IndexOf(SecondVid);
			if (SubIdx1 < 0 || SubIdx2 < 0)
			{
				return false;
			}
			TriVertPairsOut.emplace_back(Tid, std::make_pair((int8_t)SubIdx1, (int8_t)SubIdx2));
		}

		return true;
	}

	bool FDynamicMeshEditor::WeldVertexLoops(const std::vector<int>& Loop1, const std::vector<int>& Loop2)
	{
		const int N = (int)Loop1.size();
		if (N != (int)Loop2.size())
		{
			return false;
		}

		std::vector<int> Edges1(N), Edges2(N);
		for (int i = 0; i < N; ++i)
		{
			Edges1[i] = Mesh->FindEdge(Loop1[i], Loop1[(i + 1) % N]);
			if (Edges1[i] == InvalidID)
			{
				return false;
			}

			Edges2[i] = Mesh->FindEdge(Loop2[i], Loop2[(i + 1) % N]);
			if (Edges2[i] == InvalidID)
			{
				return false;
			}
		}

		int FailureCount = 0;
		std::vector<int> SkipEdges;
		for (int i = 0; i < N; ++i)
		{
			const int Edge1 = Edges1[i];
			const int Edge2 = Edges2[i];
			if (std::find(SkipEdges.begin(), SkipEdges.end(), Edge2) != SkipEdges.end())
			{
				continue;
			}

			FMergeEdgesInfo MergeInfo;
			const EMeshResult Result = Mesh->MergeEdges(Edge1, Edge2, MergeInfo);
			if (Result != EMeshResult::Ok)
			{
				FailureCount++;
				continue;
			}

			if (MergeInfo.ExtraRemovedEdges.a != InvalidID)
			{
				SkipEdges.push_back(MergeInfo.ExtraRemovedEdges.a);
			}
			if (MergeInfo.ExtraRemovedEdges.b != InvalidID)
			{
				SkipEdges.push_back(MergeInfo.ExtraRemovedEdges.b);
			}
			SkipEdges.insert(SkipEdges.end(), MergeInfo.BowtiesRemovedEdges.begin(), MergeInfo.BowtiesRemovedEdges.end());
		}

		return FailureCount == 0;
	}

	bool FDynamicMeshEditor::StitchSparselyCorrespondedVertexLoops(
		const std::vector<int>& VertexIDs1,
		const std::vector<int>& MatchedIndices1,
		const std::vector<int>& VertexIDs2,
		const std::vector<int>& MatchedIndices2,
		FDynamicMeshEditResult& ResultOut,
		bool bReverseOrientation)
	{
		const int CorrespondN = (int)MatchedIndices1.size();
		if (CorrespondN != (int)MatchedIndices2.size() || CorrespondN < 2)
		{
			return false;
		}

		auto GetWrappedSpanLen = [](const DynamicMesh* Mesh, const std::vector<int>& VertexIDs, int StartInd, int EndInd)
		{
			float LenTotal = 0.0f;
			Vector3 V = Mesh->GetVertex(VertexIDs[StartInd]);
			for (int Ind = StartInd, IndNext; Ind != EndInd;)
			{
				IndNext = (Ind + 1) % (int)VertexIDs.size();
				Vector3 VNext = Mesh->GetVertex(VertexIDs[IndNext]);
				LenTotal += DynamicMeshEditorLocal::Distance(V, VNext);
				Ind = IndNext;
				V = VNext;
			}
			return LenTotal;
		};

		ResultOut.NewGroups.reserve(ResultOut.NewGroups.size() + CorrespondN);
		constexpr float Epsilon = 1e-6f;
		for (int i = 0; i < CorrespondN; ++i)
		{
			int Starts[2]{ MatchedIndices1[i], MatchedIndices2[i] };
			int Ends[2]{ MatchedIndices1[(i + 1) % CorrespondN], MatchedIndices2[(i + 1) % CorrespondN] };
			float LenTotal[2]{
				GetWrappedSpanLen(Mesh, VertexIDs1, Starts[0], Ends[0]) + Epsilon,
				GetWrappedSpanLen(Mesh, VertexIDs2, Starts[1], Ends[1]) + Epsilon
			};
			float LenAlong[2]{ Epsilon, Epsilon };

			const int NewGroupID = Mesh->AllocateTriangleGroup();
			ResultOut.NewGroups.push_back(NewGroupID);

			int Walks[2]{ Starts[0], Starts[1] };
			Vector3 Vertex[2]{ Mesh->GetVertex(VertexIDs1[Starts[0]]), Mesh->GetVertex(VertexIDs2[Starts[1]]) };
			while (Walks[0] != Ends[0] || Walks[1] != Ends[1])
			{
				const float PctAlong[2]{ LenAlong[0] / LenTotal[0], LenAlong[1] / LenTotal[1] };
				const bool bAdvanceSecond = (Walks[0] == Ends[0] || (Walks[1] != Ends[1] && PctAlong[0] > PctAlong[1]));
				Index3 Tri(VertexIDs1[Walks[0]], VertexIDs2[Walks[1]], -1);
				if (!bAdvanceSecond)
				{
					Walks[0] = (Walks[0] + 1) % (int)VertexIDs1.size();
					Tri.c = VertexIDs1[Walks[0]];
					const Vector3 NextV = Mesh->GetVertex(Tri.c);
					LenAlong[0] += DynamicMeshEditorLocal::Distance(NextV, Vertex[0]);
					Vertex[0] = NextV;
				}
				else
				{
					Walks[1] = (Walks[1] + 1) % (int)VertexIDs2.size();
					Tri.c = VertexIDs2[Walks[1]];
					const Vector3 NextV = Mesh->GetVertex(Tri.c);
					LenAlong[1] += DynamicMeshEditorLocal::Distance(NextV, Vertex[1]);
					Vertex[1] = NextV;
				}
				if (bReverseOrientation)
				{
					std::swap(Tri.b, Tri.c);
				}

				const int Tid = Mesh->AppendTriangle(Tri, NewGroupID);
				if (Tid < 0)
				{
					RemoveTriangles(ResultOut.NewTriangles, false);
					return false;
				}
				ResultOut.NewTriangles.push_back(Tid);
			}
		}

		return true;
	}

	bool FDynamicMeshEditor::AddTriangleFan_OrderedVertexLoop(int CenterVertex, const std::vector<int>& VertexLoop, int GroupID, FDynamicMeshEditResult& ResultOut)
	{
		if (GroupID == -1)
		{
			GroupID = Mesh->AllocateTriangleGroup();
			ResultOut.NewGroups.push_back(GroupID);
		}

		const int N = (int)VertexLoop.size();
		ResultOut.NewTriangles.reserve(ResultOut.NewTriangles.size() + N);
		for (int i = 0; i < N; ++i)
		{
			const int A = VertexLoop[i];
			const int B = VertexLoop[(i + 1) % N];
			const int NewTID = Mesh->AppendTriangle(Index3(CenterVertex, B, A), GroupID);
			if (NewTID < 0)
			{
				RemoveTriangles(ResultOut.NewTriangles, false);
				return false;
			}
			ResultOut.NewTriangles.push_back(NewTID);
		}

		return true;
	}

	bool FDynamicMeshEditor::RemoveTriangles(const std::vector<int>& Triangles, bool bRemoveIsolatedVerts)
	{
		return RemoveTriangles(Triangles, bRemoveIsolatedVerts, std::function<void(int)>());
	}

	bool FDynamicMeshEditor::RemoveTriangles(const std::vector<int>& Triangles, bool bRemoveIsolatedVerts, const std::function<void(int)>& OnRemoveTriFunc)
	{
		bool bAllOK = true;
		for (int TriangleID : Triangles)
		{
			if (!Mesh->IsTriangle(TriangleID))
			{
				continue;
			}
			if (OnRemoveTriFunc)
			{
				OnRemoveTriFunc(TriangleID);
			}
			bAllOK = (Mesh->RemoveTriangle(TriangleID, bRemoveIsolatedVerts, false) == EMeshResult::Ok) && bAllOK;
		}
		return bAllOK;
	}

	int FDynamicMeshEditor::RemoveSmallComponents(double MinVolume, double MinArea, int MinTriangleCount)
	{
		std::vector<char> Visited(Mesh->GetTriangleCount(), 0);
		int Removed = 0;

		for (int SeedTID = 0; SeedTID < Mesh->GetTriangleCount(); ++SeedTID)
		{
			if (Visited[SeedTID] || !Mesh->IsTriangleFast(SeedTID))
			{
				continue;
			}

			std::vector<int> Component;
			std::vector<int> Queue;
			Queue.push_back(SeedTID);
			Visited[SeedTID] = 1;
			while (!Queue.empty())
			{
				const int TID = Queue.back();
				Queue.pop_back();
				Component.push_back(TID);
				const Index3 TriEdges = Mesh->GetTriEdges(TID);
				for (int j = 0; j < 3; ++j)
				{
					const Index2 EdgeTris = Mesh->GetEdgeT(TriEdges[j]);
					for (int k = 0; k < 2; ++k)
					{
						const int NbrTID = EdgeTris[k];
						if (NbrTID >= 0 && !Visited[NbrTID] && Mesh->IsTriangleFast(NbrTID))
						{
							Visited[NbrTID] = 1;
							Queue.push_back(NbrTID);
						}
					}
				}
			}

			bool bRemove = (MinTriangleCount > 0 && (int)Component.size() < MinTriangleCount);
			if (!bRemove && (MinVolume > 0.0 || MinArea > 0.0))
			{
				double Volume = 0.0;
				double Area = 0.0;
				for (int TID : Component)
				{
					Area += DynamicMeshEditorLocal::TriangleArea(*Mesh, TID);
					Volume += DynamicMeshEditorLocal::SignedTriangleVolume(*Mesh, TID);
				}
				bRemove = (MinVolume > 0.0 && std::fabs(Volume) < MinVolume) || (MinArea > 0.0 && Area < MinArea);
			}

			if (bRemove)
			{
				RemoveTriangles(Component, true);
				Removed++;
			}
		}

		return Removed;
	}

	void FDynamicMeshEditor::DuplicateTriangles(const std::vector<int>& Triangles, FMeshIndexMappings& IndexMaps, FDynamicMeshEditResult& ResultOut)
	{
		ResultOut.Reset();
		IndexMaps.Initialize(Mesh);

		for (int TriangleID : Triangles)
		{
			if (!Mesh->IsTriangle(TriangleID))
			{
				continue;
			}

			const Index3 Tri = Mesh->GetTriangle(TriangleID);
			const int NewGroupID = Mesh->HasTriangleGroups() ? FindOrCreateDuplicateGroup(TriangleID, IndexMaps, ResultOut) : -1;
			Index3 NewTri;
			for (int j = 0; j < 3; ++j)
			{
				NewTri[j] = FindOrCreateDuplicateVertex(Tri[j], IndexMaps, ResultOut);
			}

			const int NewTriangleID = Mesh->AppendTriangle(NewTri, NewGroupID);
			IndexMaps.SetTriangle(TriangleID, NewTriangleID);
			ResultOut.NewTriangles.push_back(NewTriangleID);
			CopyAttributes(TriangleID, NewTriangleID, IndexMaps, ResultOut);
		}
	}

	Vector3 FDynamicMeshEditor::ComputeAndSetQuadNormal(const Index2& QuadTris, bool bIsPlanar)
	{
		Vector3 Normal = Mesh->IsTriangle(QuadTris.a) ? Mesh->GetTriNormal(QuadTris.a) : Vector3::UnitZ();
		if (!bIsPlanar && Mesh->IsTriangle(QuadTris.b))
		{
			Normal = DynamicMeshEditorLocal::AverageSafe(Normal, Mesh->GetTriNormal(QuadTris.b));
		}
		SetQuadNormals(QuadTris, Normal);
		return Normal;
	}

	void FDynamicMeshEditor::SetQuadNormals(const Index2& QuadTris, const Vector3& Normal)
	{
		FDynamicMeshNormalOverlay* Normals = DynamicMeshEditorLocal::EnsureNormalOverlay(Mesh);
		std::map<int, int> VertToElement;

		auto SetTriangleNormal = [&](int TriangleID)
		{
			if (!Mesh->IsTriangle(TriangleID))
			{
				return;
			}
			if (Normals->IsSetTriangle(TriangleID))
			{
				Normals->UnsetTriangle(TriangleID);
			}
			const Index3 Tri = Mesh->GetTriangle(TriangleID);
			Index3 ElemTri;
			for (int j = 0; j < 3; ++j)
			{
				auto Found = VertToElement.find(Tri[j]);
				if (Found == VertToElement.end())
				{
					ElemTri[j] = Normals->AppendElement(Normal);
					Normals->SetParentVertex(ElemTri[j], Tri[j]);
					VertToElement.emplace(Tri[j], ElemTri[j]);
				}
				else
				{
					ElemTri[j] = Found->second;
				}
			}
			Normals->SetTriangle(TriangleID, ElemTri);
		};

		SetTriangleNormal(QuadTris.a);
		SetTriangleNormal(QuadTris.b);
	}

	void FDynamicMeshEditor::SetTriangleNormals(const std::vector<int>& Triangles, const Vector3& Normal)
	{
		FDynamicMeshNormalOverlay* Normals = DynamicMeshEditorLocal::EnsureNormalOverlay(Mesh);
		std::map<int, int> VertToElement;
		for (int TriangleID : Triangles)
		{
			if (!Mesh->IsTriangle(TriangleID))
			{
				continue;
			}
			if (Normals->IsSetTriangle(TriangleID))
			{
				Normals->UnsetTriangle(TriangleID);
			}
			const Index3 Tri = Mesh->GetTriangle(TriangleID);
			Index3 ElemTri;
			for (int j = 0; j < 3; ++j)
			{
				auto Found = VertToElement.find(Tri[j]);
				if (Found == VertToElement.end())
				{
					ElemTri[j] = Normals->AppendElement(Normal);
					Normals->SetParentVertex(ElemTri[j], Tri[j]);
					VertToElement.emplace(Tri[j], ElemTri[j]);
				}
				else
				{
					ElemTri[j] = Found->second;
				}
			}
			Normals->SetTriangle(TriangleID, ElemTri);
		}
	}

	void FDynamicMeshEditor::SetTriangleNormals(const std::vector<int>& Triangles)
	{
		std::map<int, Vector3> VertexNormals;
		for (int TriangleID : Triangles)
		{
			if (!Mesh->IsTriangle(TriangleID))
			{
				continue;
			}
			Vector3 a, b, c;
			Mesh->GetTriVertices(TriangleID, a, b, c);
			const Vector3 WeightedNormal = (b - a).Cross(c - a);
			const Index3 Tri = Mesh->GetTriangle(TriangleID);
			for (int j = 0; j < 3; ++j)
			{
				auto Found = VertexNormals.find(Tri[j]);
				if (Found == VertexNormals.end())
				{
					VertexNormals.emplace(Tri[j], WeightedNormal);
				}
				else
				{
					Found->second += WeightedNormal;
				}
			}
		}

		FDynamicMeshNormalOverlay* Normals = DynamicMeshEditorLocal::EnsureNormalOverlay(Mesh);
		std::map<int, int> VertToElement;
		for (int TriangleID : Triangles)
		{
			if (!Mesh->IsTriangle(TriangleID))
			{
				continue;
			}
			if (Normals->IsSetTriangle(TriangleID))
			{
				Normals->UnsetTriangle(TriangleID);
			}
			const Index3 Tri = Mesh->GetTriangle(TriangleID);
			Index3 ElemTri;
			for (int j = 0; j < 3; ++j)
			{
				auto Found = VertToElement.find(Tri[j]);
				if (Found == VertToElement.end())
				{
					const Vector3 Normal = VertexNormals[Tri[j]].SafeUnit();
					ElemTri[j] = Normals->AppendElement(Normal);
					Normals->SetParentVertex(ElemTri[j], Tri[j]);
					VertToElement.emplace(Tri[j], ElemTri[j]);
				}
				else
				{
					ElemTri[j] = Found->second;
				}
			}
			Normals->SetTriangle(TriangleID, ElemTri);
		}
	}

	void FDynamicMeshEditor::SetTubeNormals(
		const std::vector<int>& Triangles,
		const std::vector<int>& VertexIDs1,
		const std::vector<int>& MatchedIndices1,
		const std::vector<int>& VertexIDs2,
		const std::vector<int>& MatchedIndices2)
	{
		const int NumMatched = (int)MatchedIndices1.size();
		if (NumMatched == 0 || NumMatched != (int)MatchedIndices2.size())
		{
			return;
		}

		std::vector<Vector3> MatchedEdgeNormals[2];
		MatchedEdgeNormals[0].resize(NumMatched);
		MatchedEdgeNormals[1].resize(NumMatched);
		for (int Idx = 0; Idx < NumMatched; ++Idx)
		{
			const int LastMatchedIdx = (Idx + NumMatched - 1) % NumMatched;
			const int M1[2]{ MatchedIndices1[LastMatchedIdx], MatchedIndices1[Idx] };
			const int M2[2]{ MatchedIndices2[LastMatchedIdx], MatchedIndices2[Idx] };
			const Vector3 Corners[4]{
				Mesh->GetVertex(VertexIDs1[M1[0]]),
				Mesh->GetVertex(VertexIDs1[M1[1]]),
				Mesh->GetVertex(VertexIDs2[M2[0]]),
				Mesh->GetVertex(VertexIDs2[M2[1]])
			};
			const Vector3 Edges[2]{ Corners[1] - Corners[0], Corners[3] - Corners[2] };
			const Vector3 Across = Corners[2] - Corners[0];
			MatchedEdgeNormals[0][Idx] = Edges[0].Cross(Across).SafeUnit();
			MatchedEdgeNormals[1][Idx] = Edges[1].Cross(Across).SafeUnit();
		}

		std::vector<Vector3> MatchedVertNormals[2];
		MatchedVertNormals[0].resize(NumMatched);
		MatchedVertNormals[1].resize(NumMatched);
		for (int Idx = 0; Idx < NumMatched; ++Idx)
		{
			const int Next = (Idx + 1) % NumMatched;
			MatchedVertNormals[0][Idx] = DynamicMeshEditorLocal::AverageSafe(MatchedEdgeNormals[0][Idx], MatchedEdgeNormals[0][Next]);
			MatchedVertNormals[1][Idx] = DynamicMeshEditorLocal::AverageSafe(MatchedEdgeNormals[1][Idx], MatchedEdgeNormals[1][Next]);
		}

		std::vector<Vector3> VertNormals[2];
		VertNormals[0].resize(VertexIDs1.size());
		VertNormals[1].resize(VertexIDs2.size());
		for (int Side = 0; Side < 2; Side++)
		{
			const std::vector<int>& MatchedIndices = Side == 0 ? MatchedIndices1 : MatchedIndices2;
			const std::vector<int>& VertexIDs = Side == 0 ? VertexIDs1 : VertexIDs2;
			const int NumVertices = (int)VertNormals[Side].size();
			for (int Idx = 0; Idx < NumMatched; Idx++)
			{
				const int LastMatchedIdx = (Idx + NumMatched - 1) % NumMatched;
				const int Start = MatchedIndices[LastMatchedIdx];
				const int End = MatchedIndices[Idx];
				VertNormals[Side][End] = MatchedVertNormals[Side][Idx];
				if (Start == End)
				{
					continue;
				}

				VertNormals[Side][Start] = MatchedVertNormals[Side][LastMatchedIdx];
				const Vector3 StartPos = Mesh->GetVertex(VertexIDs[Start]);
				const Vector3 Along = Mesh->GetVertex(VertexIDs[End]) - StartPos;
				const float SepSq = Along.SquareLength();
				for (int InsideIdx = (Start + 1) % NumVertices; InsideIdx != End; InsideIdx = (InsideIdx + 1) % NumVertices)
				{
					if (SepSq <= 1e-12f)
					{
						VertNormals[Side][InsideIdx] = VertNormals[Side][End];
					}
					else
					{
						const float InterpT = (Mesh->GetVertex(VertexIDs[InsideIdx]) - StartPos).Dot(Along) / SepSq;
						VertNormals[Side][InsideIdx] = Maths::LinearInterp(MatchedVertNormals[Side][LastMatchedIdx], MatchedVertNormals[Side][Idx], InterpT).SafeUnit();
					}
				}
			}
		}

		FDynamicMeshNormalOverlay* Normals = DynamicMeshEditorLocal::EnsureNormalOverlay(Mesh);
		std::map<int, int> VertToElID;
		for (int Side = 0; Side < 2; Side++)
		{
			const std::vector<int>& VertexIDs = Side == 0 ? VertexIDs1 : VertexIDs2;
			for (int Idx = 0; Idx < (int)VertexIDs.size(); Idx++)
			{
				const int VID = VertexIDs[Idx];
				const int ElemID = Normals->AppendElement(VertNormals[Side][Idx]);
				Normals->SetParentVertex(ElemID, VID);
				VertToElID[VID] = ElemID;
			}
		}

		for (int TID : Triangles)
		{
			if (!Mesh->IsTriangle(TID))
			{
				continue;
			}
			if (Normals->IsSetTriangle(TID))
			{
				Normals->UnsetTriangle(TID);
			}
			const Index3 Tri = Mesh->GetTriangle(TID);
			Normals->SetTriangle(TID, Index3(VertToElID[Tri.a], VertToElID[Tri.b], VertToElID[Tri.c]));
		}
	}

	void FDynamicMeshEditor::SetGeneralTubeUVs(
		const std::vector<int>& Triangles,
		const std::vector<int>& VertexIDs1,
		const std::vector<int>& MatchedIndices1,
		const std::vector<int>& VertexIDs2,
		const std::vector<int>& MatchedIndices2,
		const std::vector<float>& UValues,
		const Vector3& VDir,
		float UVScaleFactor,
		const Vector2& UVTranslation,
		int UVLayerIndex)
	{
		const int NumMatched = (int)MatchedIndices1.size();
		if (NumMatched == 0 || NumMatched != (int)MatchedIndices2.size() || UValues.size() < (size_t)NumMatched)
		{
			return;
		}

		FDynamicMeshUVOverlay* UVs = DynamicMeshEditorLocal::EnsureUVOverlay(Mesh, UVLayerIndex);
		const Vector3 RefPos = Mesh->GetVertex(VertexIDs1[0]);
		auto GetUV = [this, &VDir, UVScaleFactor, &UVTranslation, &RefPos](int MeshIdx, float UStart, float UEnd, float Param)
		{
			const float U = (Mesh->GetVertex(MeshIdx) - RefPos).Dot(VDir);
			const float V = Maths::LinearInterp(UStart, UEnd, Param);
			return Vector2(U, V) * UVScaleFactor + UVTranslation;
		};

		std::vector<Vector2> VertUVs[2];
		VertUVs[0].resize(VertexIDs1.size() + 1);
		VertUVs[1].resize(VertexIDs2.size() + 1);

		std::map<int, int> VertToElID;
		int DuplicateMappingForLastVert[2]{ -1, -1 };
		for (int Side = 0; Side < 2; Side++)
		{
			const std::vector<int>& MatchedIndices = Side == 0 ? MatchedIndices1 : MatchedIndices2;
			const std::vector<int>& VertexIDs = Side == 0 ? VertexIDs1 : VertexIDs2;
			const int NumVertices = (int)VertexIDs.size();
			for (int Idx = 0; Idx < NumMatched; Idx++)
			{
				const int NextIdx = Idx + 1;
				const int NextIdxLooped = NextIdx % NumMatched;
				const bool bOnLast = NextIdx == NumMatched;
				const int Start = MatchedIndices[Idx];
				const int End = MatchedIndices[NextIdxLooped];
				const int EndUnlooped = bOnLast ? NumVertices : End;

				VertUVs[Side][EndUnlooped] = GetUV(VertexIDs[End], UValues[Idx], UValues[NextIdxLooped], 1.0f);
				if (Start == End)
				{
					continue;
				}

				VertUVs[Side][Start] = GetUV(VertexIDs[Start], UValues[Idx], UValues[NextIdxLooped], 0.0f);
				const Vector3 StartPos = Mesh->GetVertex(VertexIDs[Start]);
				const Vector3 Along = Mesh->GetVertex(VertexIDs[End]) - StartPos;
				const float SepSq = Along.SquareLength();
				for (int InsideIdx = (Start + 1) % NumVertices; InsideIdx != End; InsideIdx = (InsideIdx + 1) % NumVertices)
				{
					if (SepSq <= 1e-12f)
					{
						VertUVs[Side][InsideIdx] = VertUVs[Side][EndUnlooped];
					}
					else
					{
						const float InterpT = (Mesh->GetVertex(VertexIDs[InsideIdx]) - StartPos).Dot(Along) / SepSq;
						VertUVs[Side][InsideIdx] = GetUV(VertexIDs[InsideIdx], UValues[Idx], UValues[NextIdxLooped], InterpT);
					}
				}
			}

			for (int Idx = 0; Idx < NumVertices; Idx++)
			{
				const int VID = VertexIDs[Idx];
				const int ElemID = UVs->AppendElement(VertUVs[Side][Idx]);
				UVs->SetParentVertex(ElemID, VID);
				VertToElID[VID] = ElemID;
			}
			DuplicateMappingForLastVert[Side] = UVs->AppendElement(VertUVs[Side].back());
		}

		bool bPastInitialVertices[2]{ false, false };
		const int FirstVID[2]{ VertexIDs1[0], VertexIDs2[0] };
		for (int TID : Triangles)
		{
			if (!Mesh->IsTriangle(TID))
			{
				continue;
			}
			if (UVs->IsSetTriangle(TID))
			{
				UVs->UnsetTriangle(TID);
			}
			const Index3 Tri = Mesh->GetTriangle(TID);
			Index3 ElTri(VertToElID[Tri.a], VertToElID[Tri.b], VertToElID[Tri.c]);
			for (int Side = 0; Side < 2; Side++)
			{
				const int FirstVIDSubIdx = Tri.IndexOf(FirstVID[Side]);
				bPastInitialVertices[Side] = bPastInitialVertices[Side] || FirstVIDSubIdx == -1;
				if (bPastInitialVertices[Side] && FirstVIDSubIdx >= 0)
				{
					ElTri[FirstVIDSubIdx] = DuplicateMappingForLastVert[Side];
				}
			}
			UVs->SetTriangle(TID, ElTri);
		}
	}

	void FDynamicMeshEditor::SetTriangleUVsFromProjection(
		const std::vector<int>& Triangles,
		const Maths::Frame3& ProjectionFrame,
		float UVScaleFactor,
		const Vector2& UVTranslation,
		bool bShiftToOrigin,
		int UVLayerIndex)
	{
		SetTriangleUVsFromProjection(Triangles, ProjectionFrame, Vector2(UVScaleFactor, UVScaleFactor), UVTranslation, UVLayerIndex, bShiftToOrigin, false);
	}

	void FDynamicMeshEditor::SetTriangleUVsFromProjection(
		const std::vector<int>& Triangles,
		const Maths::Frame3& ProjectionFrame,
		const Vector2& UVScale,
		const Vector2& UVTranslation,
		int UVLayerIndex,
		bool bShiftToOrigin,
		bool bNormalizeBeforeScaling)
	{
		if (Triangles.empty())
		{
			return;
		}

		FDynamicMeshUVOverlay* UVs = DynamicMeshEditorLocal::EnsureUVOverlay(Mesh, UVLayerIndex);
		std::map<int, int> BaseToOverlayVIDMap;
		std::vector<int> AllUVIndices;
		Box2 UVBounds = Box2::Empty();

		for (int TID : Triangles)
		{
			if (!Mesh->IsTriangle(TID))
			{
				continue;
			}
			if (UVs->IsSetTriangle(TID))
			{
				UVs->UnsetTriangle(TID);
			}

			const Index3 BaseTri = Mesh->GetTriangle(TID);
			Index3 ElemTri;
			for (int j = 0; j < 3; ++j)
			{
				auto Found = BaseToOverlayVIDMap.find(BaseTri[j]);
				if (Found == BaseToOverlayVIDMap.end())
				{
					const Vector2 UV = ProjectionFrame.ProjectXY(Mesh->GetVertex(BaseTri[j]));
					UVBounds.Encapsulate(UV);
					ElemTri[j] = UVs->AppendElement(UV);
					UVs->SetParentVertex(ElemTri[j], BaseTri[j]);
					AllUVIndices.push_back(ElemTri[j]);
					BaseToOverlayVIDMap.emplace(BaseTri[j], ElemTri[j]);
				}
				else
				{
					ElemTri[j] = Found->second;
				}
			}
			UVs->SetTriangle(TID, ElemTri);
		}

		Vector2 UvScaleToUse = UVScale;
		if (bNormalizeBeforeScaling)
		{
			const float Width = UVBounds.GetLengthX();
			const float Height = UVBounds.GetLengthY();
			UvScaleToUse = Vector2(
				Width > 1e-12f ? UVScale.x / Width : UVScale.x,
				Height > 1e-12f ? UVScale.y / Height : UVScale.y);
		}

		for (int UVID : AllUVIndices)
		{
			Vector2 UV = UVs->GetElement(UVID);
			Vector2 TransformedUV = bShiftToOrigin ? ((UV - UVBounds.Min) * UvScaleToUse) : (UV * UvScaleToUse);
			TransformedUV += UVTranslation;
			UVs->SetElement(UVID, TransformedUV);
		}
	}

	void FDynamicMeshEditor::SetQuadUVsFromProjection(const Index2& QuadTris, const Maths::Frame3& ProjectionFrame, float UVScaleFactor, const Vector2& UVTranslation, int UVLayerIndex)
	{
		std::vector<int> Triangles;
		if (Mesh->IsTriangle(QuadTris.a))
		{
			Triangles.push_back(QuadTris.a);
		}
		if (Mesh->IsTriangle(QuadTris.b))
		{
			Triangles.push_back(QuadTris.b);
		}
		SetTriangleUVsFromProjection(Triangles, ProjectionFrame, UVScaleFactor, UVTranslation, true, UVLayerIndex);
	}

	void FDynamicMeshEditor::RescaleAttributeUVs(float UVScale, bool bWorldSpace, int UVLayerIndex, const Transform* ToWorld)
	{
		if (!Mesh->HasAttributes() || Mesh->Attributes()->NumUVLayers() <= UVLayerIndex)
		{
			return;
		}

		FDynamicMeshUVOverlay* UVs = Mesh->Attributes()->GetUVLayer(UVLayerIndex);
		if (bWorldSpace)
		{
			float TotalEdgeUVLen = 0.0f;
			double TotalEdgeLen = 0.0;
			for (int TID = 0; TID < Mesh->GetTriangleCount(); ++TID)
			{
				if (!Mesh->IsTriangleFast(TID) || !UVs->IsSetTriangle(TID))
				{
					continue;
				}

				Vector2 TriUVs[3];
				Vector3 TriVs[3];
				UVs->GetTriElements(TID, TriUVs[0], TriUVs[1], TriUVs[2]);
				Mesh->GetTriVertices(TID, TriVs[0], TriVs[1], TriVs[2]);
				if (ToWorld != nullptr)
				{
					for (int i = 0; i < 3; i++)
					{
						TriVs[i] = ToWorld->LocalToWorld(TriVs[i]);
					}
				}
				for (int j = 2, i = 0; i < 3; j = i++)
				{
					TotalEdgeUVLen += (TriUVs[j] - TriUVs[i]).Length();
					TotalEdgeLen += DynamicMeshEditorLocal::Distance(TriVs[j], TriVs[i]);
				}
			}
			if (TotalEdgeUVLen > 1e-6f)
			{
				UVScale *= (float)(TotalEdgeLen / TotalEdgeUVLen);
			}
		}

		for (int UVID = 0; UVID < UVs->MaxElementID(); ++UVID)
		{
			if (UVs->IsElement(UVID))
			{
				UVs->SetElement(UVID, UVs->GetElement(UVID) * UVScale);
			}
		}
	}

	int FDynamicMeshEditor::FindOrCreateDuplicateUV(int ElementID, int UVLayerIndex, FMeshIndexMappings& IndexMaps)
	{
		int NewElementID = IndexMaps.GetNewUV(UVLayerIndex, ElementID);
		if (NewElementID == IndexMaps.InvalidID())
		{
			FDynamicMeshUVOverlay* UVOverlay = Mesh->Attributes()->GetUVLayer(UVLayerIndex);
			NewElementID = UVOverlay->AppendElement(UVOverlay->GetElement(ElementID));
			UVOverlay->SetParentVertex(NewElementID, UVOverlay->GetParentVertex(ElementID));
			IndexMaps.SetUV(UVLayerIndex, ElementID, NewElementID);
		}
		return NewElementID;
	}

	int FDynamicMeshEditor::FindOrCreateDuplicateNormal(int ElementID, int NormalLayerIndex, FMeshIndexMappings& IndexMaps, FDynamicMeshEditResult* ResultOut)
	{
		int NewElementID = IndexMaps.GetNewNormal(NormalLayerIndex, ElementID);
		if (NewElementID == IndexMaps.InvalidID())
		{
			FDynamicMeshNormalOverlay* NormalOverlay = Mesh->Attributes()->GetNormalLayer(NormalLayerIndex);
			NewElementID = NormalOverlay->AppendElement(NormalOverlay->GetElement(ElementID));
			NormalOverlay->SetParentVertex(NewElementID, NormalOverlay->GetParentVertex(ElementID));
			IndexMaps.SetNormal(NormalLayerIndex, ElementID, NewElementID);
			if (ResultOut != nullptr)
			{
				if ((int)ResultOut->NewNormalOverlayElements.size() <= NormalLayerIndex)
				{
					ResultOut->NewNormalOverlayElements.resize((size_t)NormalLayerIndex + 1);
				}
				ResultOut->NewNormalOverlayElements[NormalLayerIndex].push_back(NewElementID);
			}
		}
		return NewElementID;
	}

	int FDynamicMeshEditor::FindOrCreateDuplicateColor(int ElementID, FMeshIndexMappings& IndexMaps, FDynamicMeshEditResult* ResultOut)
	{
		int NewElementID = IndexMaps.GetNewColor(ElementID);
		if (NewElementID == IndexMaps.InvalidID())
		{
			FDynamicMeshColorOverlay* ColorOverlay = Mesh->Attributes()->PrimaryColors();
			NewElementID = ColorOverlay->AppendElement(ColorOverlay->GetElement(ElementID));
			ColorOverlay->SetParentVertex(NewElementID, ColorOverlay->GetParentVertex(ElementID));
			IndexMaps.SetColor(ElementID, NewElementID);
			if (ResultOut != nullptr)
			{
				ResultOut->NewColorOverlayElements.push_back(NewElementID);
			}
		}
		return NewElementID;
	}

	int FDynamicMeshEditor::FindOrCreateDuplicateVertex(int VertexID, FMeshIndexMappings& IndexMaps, FDynamicMeshEditResult& ResultOut)
	{
		int NewVertexID = IndexMaps.GetNewVertex(VertexID);
		if (NewVertexID == IndexMaps.InvalidID())
		{
			NewVertexID = Mesh->AppendVertex(*Mesh, VertexID);
			IndexMaps.SetVertex(VertexID, NewVertexID);
			ResultOut.NewVertices.push_back(NewVertexID);

			if (Mesh->HasAttributes())
			{
				for (int WeightLayerIndex = 0; WeightLayerIndex < Mesh->Attributes()->NumWeightLayers(); ++WeightLayerIndex)
				{
					FDynamicMeshWeightAttribute* WeightAttr = Mesh->Attributes()->GetWeightLayer(WeightLayerIndex);
					float Val;
					WeightAttr->GetValue(VertexID, &Val);
					WeightAttr->SetNewValue(NewVertexID, &Val);
				}
			}
		}
		return NewVertexID;
	}

	int FDynamicMeshEditor::FindOrCreateDuplicateGroup(int TriangleID, FMeshIndexMappings& IndexMaps, FDynamicMeshEditResult& ResultOut)
	{
		const int GroupID = Mesh->GetTriangleGroup(TriangleID);
		if (GroupID < 0)
		{
			return -1;
		}

		int NewGroupID = IndexMaps.GetNewGroup(GroupID);
		if (NewGroupID == IndexMaps.InvalidID())
		{
			NewGroupID = Mesh->AllocateTriangleGroup();
			IndexMaps.SetGroup(GroupID, NewGroupID);
			ResultOut.NewGroups.push_back(NewGroupID);
		}
		return NewGroupID;
	}

	void FDynamicMeshEditor::CopyAttributes(int FromTriangleID, int ToTriangleID, FMeshIndexMappings& IndexMaps, FDynamicMeshEditResult& ResultOut)
	{
		if (!Mesh->HasAttributes())
		{
			return;
		}

		for (int UVLayerIndex = 0; UVLayerIndex < Mesh->Attributes()->NumUVLayers(); UVLayerIndex++)
		{
			FDynamicMeshUVOverlay* UVOverlay = Mesh->Attributes()->GetUVLayer(UVLayerIndex);
			if (UVOverlay->IsSetTriangle(FromTriangleID))
			{
				const Index3 FromElemTri = UVOverlay->GetTriangle(FromTriangleID);
				Index3 ToElemTri = UVOverlay->GetTriangle(ToTriangleID);
				for (int j = 0; j < 3; ++j)
				{
					ToElemTri[j] = FindOrCreateDuplicateUV(FromElemTri[j], UVLayerIndex, IndexMaps);
				}
				UVOverlay->SetTriangle(ToTriangleID, ToElemTri);
			}
		}

		if (ResultOut.NewNormalOverlayElements.size() < (size_t)Mesh->Attributes()->NumNormalLayers())
		{
			ResultOut.NewNormalOverlayElements.resize(Mesh->Attributes()->NumNormalLayers());
		}
		for (int NormalLayerIndex = 0; NormalLayerIndex < Mesh->Attributes()->NumNormalLayers(); NormalLayerIndex++)
		{
			FDynamicMeshNormalOverlay* NormalOverlay = Mesh->Attributes()->GetNormalLayer(NormalLayerIndex);
			if (NormalOverlay->IsSetTriangle(FromTriangleID))
			{
				const Index3 FromElemTri = NormalOverlay->GetTriangle(FromTriangleID);
				Index3 ToElemTri = NormalOverlay->GetTriangle(ToTriangleID);
				for (int j = 0; j < 3; ++j)
				{
					ToElemTri[j] = FindOrCreateDuplicateNormal(FromElemTri[j], NormalLayerIndex, IndexMaps, &ResultOut);
				}
				NormalOverlay->SetTriangle(ToTriangleID, ToElemTri);
			}
		}

		if (Mesh->Attributes()->HasPrimaryColors())
		{
			FDynamicMeshColorOverlay* ColorOverlay = Mesh->Attributes()->PrimaryColors();
			if (ColorOverlay->IsSetTriangle(FromTriangleID))
			{
				const Index3 FromElemTri = ColorOverlay->GetTriangle(FromTriangleID);
				Index3 ToElemTri = ColorOverlay->GetTriangle(ToTriangleID);
				for (int j = 0; j < 3; ++j)
				{
					ToElemTri[j] = FindOrCreateDuplicateColor(FromElemTri[j], IndexMaps, &ResultOut);
				}
				ColorOverlay->SetTriangle(ToTriangleID, ToElemTri);
			}
		}

		if (Mesh->Attributes()->HasMaterialID())
		{
			FDynamicMeshMaterialAttribute* MaterialIDs = Mesh->Attributes()->GetMaterialID();
			MaterialIDs->SetValue(ToTriangleID, MaterialIDs->GetValue(FromTriangleID));
		}
	}


}
