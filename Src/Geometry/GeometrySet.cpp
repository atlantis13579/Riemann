
#include <assert.h>
#include "GeometrySet.h"
#include "MeshSimplification.h"
#include "../Maths/Box1.h"
#include "../CollisionPrimitive/Triangle3d.h"

namespace Geometry
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

	GeometryData::GeometryData()
	{
	}

	GeometryData::~GeometryData()
	{
	}

	template<typename T>
	static void AddVertex(std::vector<T>& Vertices, int& NumVertices, const T& v)
	{
		const int OBJ_BATCH_SIZE = 4096;
		if (NumVertices >= Vertices.size())
		{
			Vertices.resize(Vertices.size() + OBJ_BATCH_SIZE * 3);
		}
		Vertices[NumVertices++] = v;
	}

	template<typename T>
	static void Insert(std::vector<T>& Vertices, size_t index, const T& v)
	{
		if (index >= Vertices.size())
		{
			Vertices.resize(index + 1);
		}
		Vertices[index] = v;
	}

	int GeometryData::AppendVertex(const VertexInfo& info)
	{
		size_t index = VertexPositions.size();

		VertexPositions.push_back(info.Position);

		if (info.bHaveColor)
		{
			bHaveVertexColor = true;
		}
		if (info.bHaveNormal)
		{
			bHaveVertexNormals = true;
		}
		if (info.bHaveUV)
		{
			bHaveVertexUVs = true;
		}

		if (HaveVertexColors())
		{
			Vector3 val = info.bHaveColor ? info.Color : Vector3::Zero();
			Insert(VertexColors, index, val);
		}

		if (HaveVertexNormals())
		{
			Vector3 val = info.bHaveNormal ? info.Normal : Vector3::UnitY();
			Insert(VertexNormals, index, val);
		}

		if (HaveVertexUVs())
		{
			Vector2 val = info.bHaveUV ? info.UV : Vector2::Zero();
			Insert(VertexUVs, index, val);
		}

		return (int)index;
	}

	int GeometryData::AppendTriangle(const Vector3i& tri)
	{
		int index = (int)Triangles.size();
		Triangles.push_back(tri);
		return index;
	}

	void GeometryData::SetColor(int idx, const Vector3& val)
	{
		bHaveVertexColor = true;
		Insert(VertexColors, idx, val);
	}

	void GeometryData::SetNormal(int idx, const Vector3& val)
	{
		bHaveVertexNormals = true;
		Insert(VertexNormals, idx, val);
	}

	void GeometryData::SetUV(int idx, const Vector2& val, int layer)
	{
		bHaveVertexUVs = true;
		Insert(VertexUVs, idx, val);
	}

	void GeometryData::ApplyTransform(const Transform& trans, bool bReverseOrientationIfNeeded)
	{
		for (size_t i = 0; i < VertexPositions.size(); ++i)
		{
			Vector3 &Position = VertexPositions[i];
			Position = trans.LocalToWorld(Position);
		}

		if (bHaveVertexNormals)
		{
			for (size_t i = 0; i < VertexNormals.size(); ++i)
			{
				Vector3 &Normal = VertexNormals[i];
				Normal = trans.LocalToWorldDirection(Normal);
			}
		}

		if (bReverseOrientationIfNeeded && trans.quat.ToRotationMatrix3().Determinant() < 0)
		{
			ReverseOrientation(false);
		}
	}

	void GeometryData::ApplyTransform(Transform3& trans, bool bReverseOrientationIfNeeded)
	{
		for (size_t i = 0; i < VertexPositions.size(); ++i)
		{
			Vector3& Position = VertexPositions[i];
			Position = trans.LocalToWorld(Position);
		}

		if (bHaveVertexNormals)
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

	void GeometryData::ReverseOrientation(bool bFlipNormals)
	{
		for (size_t i = 0; i < Triangles.size(); ++i)
		{
			Vector3i& tri = Triangles[i];
			std::swap(tri.y, tri.z);
		}

		if (bFlipNormals && bHaveVertexNormals)
		{
			for (size_t i = 0; i < VertexNormals.size(); ++i)
			{
				Vector3& Normal = VertexNormals[i];
				Normal = -Normal;
			}
		}
	}

	Vector3 GeometryData::GetTriangleCentroid(int idx) const
	{
		const Vector3i ti = Triangles[idx];
		const Vector3 centroid = (VertexPositions[ti.x] + VertexPositions[ti.y] + VertexPositions[ti.z]) / 3.0f;
		return centroid;
	}

	Box3 GeometryData::GetTriangleBounds(int idx) const
	{
		const Vector3i ti = Triangles[idx];
		Box3 box(VertexPositions[ti.x], VertexPositions[ti.y], VertexPositions[ti.z]);
		return box;
	}

	void GeometryData::GetTriangleVertices(int idx, Vector3& a, Vector3& b, Vector3& c) const
	{
		const Vector3i ti = Triangles[idx];
		a = VertexPositions[ti.x];
		b = VertexPositions[ti.y];
		c = VertexPositions[ti.z];
	}

	bool GeometryData::Simplify(float rate)
	{
		std::vector<Vector3> new_v;
		std::vector<int> new_i;

		if (!SimplifyMesh(GetVertexBuffer(), GetIndexBuffer(), GetNumVertices(), GetNumTriangles(), rate, new_v, new_i))
		{
			return false;
		}

		bHaveVertexColor = false;
		bHaveVertexNormals = false;
		bHaveVertexUVs = false;
		VertexColors.clear();
		VertexNormals.clear();
		VertexUVs.clear();

		VertexPositions = new_v;

		Triangles.resize(new_i.size() / 3);
		memcpy(&Triangles[0], &new_i[0], Triangles.size() * sizeof(Triangles[0]));

		return true;
	}

	void GeometryData::CalculateBounds()
	{
		Bounds = Box3(VertexPositions.data(), VertexPositions.size());
	}

	bool GeometryData::LoadObj(const char* name)
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
		int NumVertices = 0;
		int NumTriangles = 0;
		int NumNormals = 0;
		int NumColors = 0;
		int NumUVs = 0;
		
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
					AddVertex(VertexPositions, NumVertices, v);
					AddVertex(VertexColors, NumColors, c);
				}
				else
				{
					Vector3 v;
					sscanf(row + 1, "%f %f %f", &v.x, &v.y, &v.z);
					AddVertex(VertexPositions, NumVertices, v);
				}

			}
			else if (row[0] == 'v' && row[1] == 'n')
			{
				Vector3 v;
				sscanf(row + 2, "%f %f %f", &v.x, &v.y, &v.z);
				AddVertex(VertexNormals, NumNormals, v);
			}
			else if (row[0] == 'v' && row[1] == 't')
			{
				Vector2 v;
				sscanf(row + 2, "%f %f", &v.x, &v.y);
				AddVertex(VertexUVs, NumUVs, v);
			}
			else if (row[0] == 'f')
			{
				int face[32];
				int nv = ParseFace(NumVertices, row + 1, face, 32);
				for (int i = 2; i < nv; ++i)
				{
					const int a = face[0];
					const int b = face[i - 1];
					const int c = face[i];
					if (a < 0 || a >= NumVertices || b < 0 || b >= NumVertices || c < 0 || c >= NumVertices)
						continue;
					AddVertex(Triangles, NumTriangles, Vector3i(a, b, c));
				}
			}
		}

		VertexPositions.resize(NumVertices);
		VertexColors.resize(NumColors);
		VertexUVs.resize(NumUVs);
		VertexNormals.resize(NumNormals);
		Triangles.resize(NumTriangles);

		delete[] buf;

		CalculateBounds();
		ResourceName = name;
		return true;
	}

	bool GeometryData::ExportObj(const char* name)
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
			const Vector3i v = Triangles[i];
			fprintf(fp, "f %d %d %d\n", v.x + 1, v.y + 1, v.z + 1);
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
			for (size_t i = 0; i < VertexUVs.size(); ++i)
			{
				const Vector2 v = VertexUVs[i];
				fprintf(fp, "vt %.3f %.3f\n", v.x, v.y);
			}
		}

		fclose(fp);
		return true;
	}

	GeometryData* GeometryData::CreateFromObj(const char* name)
	{
		GeometryData *p = new GeometryData;
		if (!p->LoadObj(name))
		{
			delete p;
			return nullptr;
		}
		return p;
	}

	GeometrySet::GeometrySet()
	{

	}

	GeometrySet::~GeometrySet()
	{
		for (size_t i = 0; i < mMeshs.size(); ++i)
		{
			delete mMeshs[i];
		}
		mMeshs.clear();
	}

	bool GeometrySet::LoadObj(const char* name, const Transform& pose)
	{
		GeometryData *p = GeometryData::CreateFromObj(name);
		if (p == nullptr)
		{
			return false;
		}

		p->Pose = pose;

		mMeshs.push_back(p);
		
		if (mMeshs.size() == 1)
		{
			mBounds = p->Bounds;
		}
		else
		{
			mBounds.Encapsulate(p->Bounds);
		}

		return true;
	}

	void GeometrySet::CreateGeometryData(int NumMeshes)
	{
		mMeshs.resize(NumMeshes);
		for (size_t i = 0; i < mMeshs.size(); ++i)
		{
			mMeshs[i] = new GeometryData();
		}
	}

	GeometryAABBTree::GeometryAABBTree(GeometryData* data)
	{
		Mesh = data;
		Build();
	}

	GeometryAABBTree::IntersectionsQueryResult GeometryAABBTree::FindAllIntersections(const GeometryAABBTree& OtherTree, const Transform* TransformF) const
	{
		IntersectionsQueryResult result;
		FindIntersections(RootIndex, OtherTree, TransformF, OtherTree.RootIndex, 0, result);
		return result;
	}

	static void AddTriTriIntersectionResult(Riemann::Triangle3dTriangle3dIntersectionResult& Intr, int TID_A, int TID_B, GeometryAABBTree::IntersectionsQueryResult& Result)
	{
		if (Intr.Quantity == 1)
		{
			Result.Points.push_back(GeometryAABBTree::PointIntersection{ {TID_A, TID_B}, Intr.Points[0] });
		}
		else if (Intr.Quantity == 2)
		{
			Result.Segments.push_back(GeometryAABBTree::SegmentIntersection{ {TID_A, TID_B}, {Intr.Points[0], Intr.Points[1]} });
		}
		else if (Intr.Quantity > 2)
		{
			if (Intr.Type == Riemann::IntersectionType::MultiSegment)
			{
				Result.Segments.push_back(GeometryAABBTree::SegmentIntersection{ {TID_A, TID_B}, {Intr.Points[0], Intr.Points[1]} });
				Result.Segments.push_back(GeometryAABBTree::SegmentIntersection{ {TID_A, TID_B}, {Intr.Points[2], Intr.Points[3]} });
				if (Intr.Quantity > 4)
				{
					Result.Segments.push_back(GeometryAABBTree::SegmentIntersection{ {TID_A, TID_B}, {Intr.Points[4], Intr.Points[5]} });
				}
			}
			else
			{
				Result.Polygons.push_back(GeometryAABBTree::PolygonIntersection{ {TID_A, TID_B},
					{Intr.Points[0], Intr.Points[1], Intr.Points[2], Intr.Points[3], Intr.Points[4], Intr.Points[5]}, Intr.Quantity });
			}
		}
	}

	void GeometryAABBTree::FindIntersections(int iBox, const GeometryAABBTree& OtherTree, const Transform* TransformF, int oBox, int depth, IntersectionsQueryResult& result) const
	{
		int idx = BoxToIndex[iBox];
		int odx = OtherTree.BoxToIndex[oBox];

		if (idx < TrianglesEnd && odx < OtherTree.TrianglesEnd)
		{
			Riemann::Triangle3d Tri, otri;
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

					Riemann::Triangle3dTriangle3dIntersectionResult intr;

					if (CalculateIntersectionTriangle3dTriangle3d(otri, Tri, intr))
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

	Box3 GeometryAABBTree::GetAABB(int idx, const Transform* TransformF) const
	{
		Box3 box = AABB[idx];
		if (TransformF)
		{
			box = Box3::Transform(box, TransformF->pos, TransformF->quat);
		}
		return box;
	}

	void GeometryAABBTree::Build()
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

	void GeometryAABBTree::Build(std::vector<int>& Triangles, std::vector<Vector3>& Centers)
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

	int GeometryAABBTree::SplitTriSetMidpoint(std::vector<int>& Triangles, std::vector<Vector3>& Centers, int IStart, int ICount, int Depth, int MinTriCount, FBoxesSet& Tris, FBoxesSet& Nodes, Box3& Box)
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
