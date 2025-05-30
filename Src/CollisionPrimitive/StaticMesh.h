#pragma once

#include <assert.h>
#include <functional>
#include <string>
#include <vector>

#include "../Maths/Box3.h"
#include "../Maths/Vector3.h"
#include "../Geometry/MeshSimplification.h"
#include "PrimitiveType.h"

namespace Riemann
{
	#define TRIANGLE_BATCH			(4096)
	#define	INDICES_16_BIT			(0x01)

	class StaticMesh
	{
	public:
		uint32_t				NumVertices;
		uint32_t				NumTriangles;
		std::vector<Vector3>	mVertices;
		std::vector<uint16_t>	mIndices;
		std::vector<Vector3>	mNormals;
		std::vector<Vector3>	mUV3s;
		Box3					BoundingVolume;
		uint8_t					Flags;
		std::string				ResourceId;

		Vector3*				Vertices;
		uint16_t*				Indices;

		StaticMesh()
		{
			Clear();
		}

		static constexpr PrimitiveType	StaticType()
		{
			return PrimitiveType::TRIANGLE_MESH;
		}

		void Clear()
		{
			mVertices.clear();
			mIndices.clear();
			mNormals.clear();
			NumVertices = 0;
			NumTriangles = 0;
			Vertices = nullptr;
			Indices = nullptr;
		}

		void Compact()
		{
			mVertices.resize(NumVertices);
			if (0 < NumTriangles && NumTriangles <= 65535 && !Is16bitIndices())
			{
				uint16_t* pDst = GetIndices16();
				uint32_t* pSrc = GetIndices32();
				for (uint32_t i = 0; i < 3 * NumTriangles; ++i)
				{
					*pDst++ = (uint16_t)(*pSrc++);
				}
				Flags |= INDICES_16_BIT;
			}
			mIndices.resize(3 * NumTriangles * GetIndicesWidth());
			Indices = &mIndices[0];
		}

		Vector3* GetVertexBuffer()
		{
			return Vertices;
		}

		void* GetIndexBuffer()
		{
			return Indices;
		}

		const void* GetIndexBuffer() const
		{
			return Indices;
		}

		const uint16_t* GetIndices16() const			// (uint16_t*)&Indices[0]
		{
			return Indices;
		}

		uint16_t* GetIndices16()
		{
			return Indices;
		}

		const uint32_t* GetIndices32() const			// (uint32_t*)&Indices[0]
		{
			return (uint32_t*)Indices;
		}

		uint32_t* GetIndices32()
		{
			return (uint32_t*)Indices;
		}

		bool Is16bitIndices() const
		{
			return Flags & INDICES_16_BIT;
		}

		int GetIndicesWidth() const
		{
			return Is16bitIndices() ? 1 : 2;
		}

		void GetVertIndices(uint32_t triIndex, uint32_t& i0, uint32_t& i1, uint32_t& i2) const
		{
			if (Is16bitIndices())
			{
				const uint16_t* p = GetIndices16() + 3 * triIndex;
				i0 = p[0];
				i1 = p[1];
				i2 = p[2];
			}
			else
			{
				const uint32_t* p = GetIndices32() + 3 * triIndex;
				i0 = p[0];
				i1 = p[1];
				i2 = p[2];
			}
		}

		inline uint32_t GetVertexCount() const
		{
			return NumVertices;
		}

		inline uint32_t GetTriangleCount() const
		{
			return NumTriangles;
		}

		inline const Vector3& GetVertex(int i, int j) const
		{
			if (Is16bitIndices())
			{
				return Vertices[Indices[3 * i + j]];
			}
			else
			{
				const uint32_t* Indices32 = GetIndices32();
				return Vertices[Indices32[3 * i + j]];
			}
		}

		inline const Vector3& operator ()(int i, int j) const
		{
			return GetVertex(i, j);
		}

		void SetData(void* Verts, void* Tris, uint32_t Nv, uint32_t Nt, bool Is16bit, bool OwnMemory)
		{
			Clear();

			if (Is16bit)
				Flags |= (INDICES_16_BIT);
			else
				Flags &= (~INDICES_16_BIT);
			NumVertices = Nv;
			NumTriangles = Nt;

			if (OwnMemory)
			{
				mVertices.resize(Nv);
				Vertices = &mVertices[0];
				memcpy(&Vertices[0], Verts, sizeof(Vertices[0]) * Nv);

				mIndices.resize(Nt * 3 * GetIndicesWidth());
				Indices = &mIndices[0];
				memcpy(&Indices[0], Tris, mIndices.size() * sizeof(Indices[0]));
			}
			else
			{
				Vertices = (Vector3*)Verts;
				Indices = (uint16_t*)Tris;
			}
		}

		void AddVertex(const Vector3& v)
		{
			if (NumVertices >= mVertices.size())
			{
				mVertices.resize(mVertices.size() + TRIANGLE_BATCH * 3);
				Vertices = &mVertices[0];
			}
			Vertices[NumVertices++] = v;
		}

		void AddNormal(const Vector3& v)
		{
			mNormals.push_back(v);
		}

		void AddUV3(const Vector3& v)
		{
			mUV3s.push_back(v);
		}

		void AddTriangle(uint32_t a, uint32_t b, uint32_t c)
		{
			if ((NumTriangles * 3 + 2) * GetIndicesWidth() >= mIndices.size())
			{
				mIndices.resize((mIndices.size() + TRIANGLE_BATCH * 3) * GetIndicesWidth());
				Indices = &mIndices[0];
			}

			if (Is16bitIndices())
			{
				Indices[3 * NumTriangles] = a;
				Indices[3 * NumTriangles + 1] = b;
				Indices[3 * NumTriangles + 2] = c;
			}
			else
			{
				uint32_t* Indices32 = GetIndices32();
				Indices32[3 * NumTriangles] = a;
				Indices32[3 * NumTriangles + 1] = b;
				Indices32[3 * NumTriangles + 2] = c;
			}

			if (NumTriangles == 0)
			{
				BoundingVolume = Box3(Vertices[a], Vertices[a]);
			}
			BoundingVolume.Encapsulate(Vertices[a], Vertices[b], Vertices[c]);

			NumTriangles++;
		}

		void AddAABB(const Vector3& Bmin, const Vector3& Bmax)
		{
			int k = NumVertices;

			Vector3 v[8];
			Box3::GetVertices(Bmin, Bmax, v);
			for (int i = 0; i < 8; ++i)
			{
				AddVertex(v[i]);
			}

			AddTriangle(k + 0, k + 1, k + 2);
			AddTriangle(k + 1, k + 3, k + 2);
			AddTriangle(k + 4, k + 5, k + 6);
			AddTriangle(k + 5, k + 7, k + 6);
			AddTriangle(k + 0, k + 1, k + 4);
			AddTriangle(k + 5, k + 4, k + 1);
			AddTriangle(k + 1, k + 3, k + 5);
			AddTriangle(k + 7, k + 5, k + 3);
			AddTriangle(k + 2, k + 4, k + 0);
			AddTriangle(k + 6, k + 4, k + 2);
			AddTriangle(k + 3, k + 2, k + 6);
			AddTriangle(k + 6, k + 7, k + 3);
		}

		void Reorder(const std::vector<uint32_t>& IndicesPermute)
		{
			std::vector<uint16_t> newIndices(NumTriangles * 3 * GetIndicesWidth());
			if (Is16bitIndices())
			{
				uint16_t* pSrc = GetIndices16();
				uint16_t* pDst = &newIndices[0];
				for (uint32_t i = 0; i < NumTriangles; ++i)
				{
					pDst[3 * i + 0] = pSrc[3 * IndicesPermute[i] + 0];
					pDst[3 * i + 1] = pSrc[3 * IndicesPermute[i] + 1];
					pDst[3 * i + 2] = pSrc[3 * IndicesPermute[i] + 2];
				}
			}
			else
			{
				uint32_t* pSrc = GetIndices32();
				uint32_t* pDst = (uint32_t*)&newIndices[0];
				for (uint32_t i = 0; i < NumTriangles; ++i)
				{
					pDst[3 * i + 0] = pSrc[3 * IndicesPermute[i] + 0];
					pDst[3 * i + 1] = pSrc[3 * IndicesPermute[i] + 1];
					pDst[3 * i + 2] = pSrc[3 * IndicesPermute[i] + 2];
				}
			}
			mIndices = std::move(newIndices);
			Indices = &mIndices[0];
		}

		bool LoadObj(const char* name)
		{
			char* buf = 0;
			int error = 0;
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

			char* p = buf, * pEnd = buf + bufSize;
			while (p < pEnd)
			{
				char row[512];
				row[0] = '\0';
				p = ParseRow(p, pEnd, row, sizeof(row) / sizeof(char));
				if (row[0] == '#') continue;
				if (row[0] == 'v' && row[1] != 'n' && row[1] != 't')
				{
					Vector3 v;
					sscanf(row + 1, "%f %f %f", &v.x, &v.y, &v.z);
					AddVertex(v);
				}
				else if (row[0] == 'v' && row[1] == 'n')
				{
					Vector3 v;
					sscanf(row + 2, "%f %f %f", &v.x, &v.y, &v.z);
					AddNormal(v);
				}
				else if (row[0] == 'v' && row[1] == 't')
				{
					Vector3 v;
					sscanf(row + 2, "%f %f %f", &v.x, &v.y, &v.z);
					AddUV3(v);
				}
				else if (row[0] == 'f')
				{
					int v[32];
					int vt[32];
					int vn[32];
					int nv = ParseFace(row + 1, v, vt, vn, 32);
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
						if (a < 0 || a >= (int)NumVertices || b < 0 || b >= (int)NumVertices || c < 0 || c >= (int)NumVertices)
							continue;
						AddTriangle(a, b, c);
					}
				}
			}

			Compact();

			delete[] buf;
			ResourceId = name;
			return true;
		}

		bool ExportObj(const char* name)
		{
			FILE* fp = fopen(name, "w");
			if (!fp)
			{
				return false;
			}

			int nv = (int)GetVertexCount();
			int nt = (int)GetTriangleCount();

			fprintf(fp, "# %d vertices, %d faces\n", nv, nt);

			for (int i = 0; i < nv; ++i)
			{
				fprintf(fp, "v %.3f %.3f %.3f\n", Vertices[i].x, Vertices[i].y, Vertices[i].z);
			}

			for (int i = 0; i < nt; ++i)
			{
				int i0, i1, i2;
				if (Is16bitIndices())
				{
					i0 = Indices[3 * i + 0];
					i1 = Indices[3 * i + 1];
					i2 = Indices[3 * i + 2];
				}
				else
				{
					uint32_t* Indices32 = GetIndices32();
					i0 = Indices32[3 * i + 0];
					i1 = Indices32[3 * i + 1];
					i2 = Indices32[3 * i + 2];
				}

				fprintf(fp, "f %d %d %d\n", i0 + 1, i1 + 1, i2 + 1);
			}

			fclose(fp);
			return true;
		}

		bool ExportFlat(const char* name)
		{
			FILE* fp = fopen(name, "wb");
			if (!fp)
			{
				return false;
			}

			uint32_t Magic = 0xF24D8017;
			fwrite(&Magic, sizeof(Magic), 1, fp);
			fwrite(&Flags, sizeof(Flags), 1, fp);
			fwrite(&NumVertices, sizeof(NumVertices), 1, fp);
			fwrite(&NumTriangles, sizeof(NumTriangles), 1, fp);
			fwrite(&Vertices[0], sizeof(Vertices[0]), NumVertices, fp);
			fwrite(&Indices[0], sizeof(Indices[0]), NumTriangles * 3, fp);
			fclose(fp);
			return true;
		}

		bool LoadFlat(const char* name)
		{
			FILE* fp = fopen(name, "rb");
			if (!fp)
			{
				return false;
			}

			fseek(fp, 0, SEEK_END);
			size_t fileSize = (size_t)ftell(fp);
			fseek(fp, 0, SEEK_SET);

			if (fileSize < 12)
			{
				return false;
			}

			uint32_t Magic;
			fread(&Magic, sizeof(Magic), 1, fp);
			if (Magic != 0xF24D8017)
			{
				return false;
			}
			fread(&Flags, sizeof(Flags), 1, fp);
			fread(&NumVertices, sizeof(NumVertices), 1, fp);
			fread(&NumTriangles, sizeof(NumTriangles), 1, fp);
			mVertices.resize(NumVertices);
			mIndices.resize(NumTriangles * 3 * GetIndicesWidth());
			Vertices = &mVertices[0];
			Indices = &mIndices[0];
			fread(Vertices, sizeof(Vertices[0]), mVertices.size(), fp);
			fread(Indices, sizeof(Indices[0]), mIndices.size(), fp);
			fclose(fp);

			ResourceId = name;

			CalculateBoundingBox();
			return true;
		}

		int FilterTriangle(std::function<bool(const Vector3&, const Vector3&, const Vector3&)> filter_func)
		{
			int j = 0;
			if (Is16bitIndices())
			{
				for (uint32_t i = 0; i < NumTriangles; ++i)
				{
					if (!filter_func(Vertices[Indices[3 * i]], Vertices[Indices[3 * i + 1]], Vertices[Indices[3 * i + 2]]))
					{
						if (i != j)
						{
							Indices[3 * j + 0] = Indices[3 * i + 0];
							Indices[3 * j + 1] = Indices[3 * i + 1];
							Indices[3 * j + 2] = Indices[3 * i + 2];
						}
						++j;
					}
				}
			}
			else
			{
				uint32_t* Indices32 = GetIndices32();
				for (uint32_t i = 0; i < NumTriangles; ++i)
				{
					if (!filter_func(Vertices[Indices32[3 * i]], Vertices[Indices32[3 * i + 1]], Vertices[Indices32[3 * i + 2]]))
					{
						if (i != j)
						{
							Indices32[3 * j + 0] = Indices32[3 * i + 0];
							Indices32[3 * j + 1] = Indices32[3 * i + 1];
							Indices32[3 * j + 2] = Indices32[3 * i + 2];
						}
						++j;
					}
				}
			}

			int nFiltered = NumTriangles - j;
			NumTriangles = j;
			return nFiltered;
		}

		void CalculateWeightAverageNormals()
		{
			if (NumTriangles == 0)
			{
				return;
			}

			if (!mNormals.empty())
			{
				return;
			}

			std::vector<float> Weight;
			Weight.resize(NumVertices, 0.0f);
			mNormals.resize(NumVertices);
			memset(&mNormals[0], 0, sizeof(mNormals[0]) * mNormals.size());
			for (uint32_t i = 0; i < NumTriangles; ++i)
			{
				int i0, i1, i2;
				if (Is16bitIndices())
				{
					i0 = Indices[3 * i + 0];
					i1 = Indices[3 * i + 1];
					i2 = Indices[3 * i + 2];
				}
				else
				{
					uint32_t* Indices32 = GetIndices32();
					i0 = Indices32[3 * i + 0];
					i1 = Indices32[3 * i + 1];
					i2 = Indices32[3 * i + 2];
				}

				const Vector3& v0 = Vertices[i0];
				const Vector3& v1 = Vertices[i1];
				const Vector3& v2 = Vertices[i2];

				const Vector3 v20 = (v2 - v0).Unit();
				const Vector3 v10 = (v1 - v0).Unit();
				const Vector3 v21 = (v2 - v1).Unit();
				Vector3 Nor = v20.Cross(v10).Unit();
				
				const float w0 = acosf(v20.Dot(v10));
				const float w1 = acosf(v21.Dot(-v10));
				const float w2 = acosf(v21.Dot(v20));

				mNormals[i0] += w0 * Nor;
				Weight[i0] += w0;
				
				mNormals[i1] += w1 * Nor;
				Weight[i1] += w1;
				
				mNormals[i2] += w2 * Nor;
				Weight[i2] += w2;
			}

			for (size_t i = 0; i < mNormals.size(); ++i)
			{
				mNormals[i] *= 1.0f / Weight[i];
				mNormals[i].Normalize();
			}
		}

		bool Simplify(const SimplificationConfig& cfg)
		{
			std::vector<Vector3> new_v;
			std::vector<int> new_i;

			if (!SimplifyMesh(GetVertexBuffer(), (const void*)GetIndexBuffer(), GetVertexCount(), GetTriangleCount(), Is16bitIndices(), cfg, new_v, new_i))
			{
				return false;
			}

			Clear();
			SetData(new_v.data(), new_i.data(), (int)new_v.size(), (int)new_i.size() / 3, false, true);
			CalculateBoundingBox();
			Compact();
			return true;
		}

		void CalculateBoundingBox()
		{
			if (NumTriangles == 0)
			{
				return;
			}

			BoundingVolume = Box3(&Vertices[0], NumVertices);
		}

		int ParseFace(const char* row, int* v, int* vt, int* vn, int n)
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
				int* pv = (pos == 0) ? v : ((pos == 1) ? vt : vn);
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

	private:
		char* ParseRow(char* buf, char* bufEnd, char* row, int len)
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
	};
}
