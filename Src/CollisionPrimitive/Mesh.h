#pragma once

#include <stdio.h>
#include <functional>
#include <string>
#include <vector>

#include "../Maths/Box3.h"
#include "../Maths/Vector3.h"
#include "ShapeType.h"

#define TRIANGLE_BATCH			(4096)
#define	INDICES_16_BIT			(0x01)

namespace Riemann
{
	class Mesh
	{
	public:
		uint32_t				NumVertices;
		uint32_t				NumTriangles;
		std::vector<Vector3>	mVertices;
		std::vector<uint16_t>	mIndices;
		std::vector<Vector3>	mNormals;
		Box3					BoundingVolume;
		uint8_t					Flags;
		std::string				ResourceId;

		Vector3* Vertices;
		uint16_t* Indices;

		Mesh()
		{
			Release();
		}

		static constexpr ShapeType3d	StaticType()
		{
			return ShapeType3d::TRIANGLE_MESH;
		}

		void Release()
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
				uint32_t  t = pSrc[0];
				pDst[0] = (uint16_t)t;
				for (uint32_t i = 1; i < 3 * NumTriangles; ++i)
				{
					*pDst++ = *pSrc++;
				}
				Flags |= INDICES_16_BIT;
			}
			mIndices.resize(3 * NumTriangles * GetIndicesWidth());
			Indices = &mIndices[0];
		}

		void* GetVertexBuffer()
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

		inline uint32_t GetNumVertices() const
		{
			return NumVertices;
		}

		inline uint32_t GetNumTriangles() const
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
			Release();

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
			Vector3 v;
			while (p < pEnd)
			{
				char row[512];
				row[0] = '\0';
				p = ParseRow(p, pEnd, row, sizeof(row) / sizeof(char));
				if (row[0] == '#') continue;
				if (row[0] == 'v' && row[1] != 'n' && row[1] != 't')
				{
					sscanf(row + 1, "%f %f %f", &v.x, &v.y, &v.z);
					AddVertex(v);
				}
				else if (row[0] == 'f')
				{
					uint32_t face[32];
					int nv = ParseFace(row + 1, face, 32);
					for (int i = 2; i < nv; ++i)
					{
						const uint32_t a = face[0];
						const uint32_t b = face[i - 1];
						const uint32_t c = face[i];
						if (a < 0 || a >= NumVertices || b < 0 || b >= NumVertices || c < 0 || c >= NumVertices)
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

			int nv = (int)GetNumVertices();
			int nt = (int)GetNumTriangles();

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

			uint32_t Magic = 0xF34D9017;
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
			if (Magic != 0xF34D9017)
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

		bool Simplify(float rate);

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

		void CalculateNormals()
		{
			if (NumTriangles == 0)
			{
				return;
			}

			if (!mNormals.empty())
			{
				return;
			}

			std::vector<int> Count;
			Count.resize(NumVertices, 0);
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
				Vector3 Nor = (v1 - v0).Cross(v2 - v0);
				mNormals[i0] += Nor.Unit(); Count[i0]++;
				mNormals[i1] += Nor.Unit(); Count[i1]++;
				mNormals[i2] += Nor.Unit(); Count[i2]++;
			}

			for (size_t i = 0; i < mNormals.size(); ++i)
			{
				mNormals[i] *= 1.0f / Count[i];
				mNormals[i].Normalize();
			}
		}

		void CalculateBoundingBox()
		{
			if (NumTriangles == 0)
			{
				return;
			}

			BoundingVolume = Box3(&Vertices[0], NumVertices);
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

		int ParseFace(char* row, uint32_t* data, int n)
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
				uint32_t vi = atoi(s);
				data[j++] = vi < 0 ? vi + NumVertices : vi - 1;
				if (j >= n) return j;
			}
			return j;
		}
	};
}
