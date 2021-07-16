
#pragma once

#include <stdio.h>
#include <functional>
#include <string>
#include <vector>

#include "../Maths/Box3d.h"
#include "../Maths/Vector3d.h"

#define TRIANGLE_BATCH			(1024)

class TriangleMesh
{
public:
	unsigned int				m_NumVerties;
	unsigned int				m_NumTriangles;
	std::vector<Vector3d>		Verties;
	std::vector<unsigned int>	Indices;
	std::vector<Vector3d>		Normals;
	Box3d						BoundingBox;
	std::string					ResourceId;

	void* GetVertexBuffer()
	{
		return &Verties[0];
	}

	void* GetIndexBuffer()
	{
		return &Indices[0];
	}

	inline unsigned int GetNumVerties() const
	{
		return m_NumVerties;
	}

	inline unsigned int GetNumTriangles() const
	{
		return m_NumTriangles;
	}

	inline const Vector3d& operator ()(int i, int j) const
	{
		return Verties[Indices[3 * i + j]];
	}

	void AddVertex(const Vector3d& v)
	{
		if (m_NumVerties >= Verties.size())
		{
			Verties.resize(Verties.size() + TRIANGLE_BATCH * 3);
		}
		Verties[m_NumVerties++] = v;
	}

	void AddTriangle(unsigned int a, unsigned int b, unsigned int c)
	{
		if (m_NumTriangles * 3 >= Indices.size())
		{
			Indices.resize(Indices.size() + TRIANGLE_BATCH * 3);
		}
		Indices[3 * m_NumTriangles] = a;
		Indices[3 * m_NumTriangles + 1] = b;
		Indices[3 * m_NumTriangles + 2] = c;

		if (m_NumTriangles == 0)
		{
			BoundingBox = Box3d(Verties[a], Verties[a]);
		}
		BoundingBox.Grow(Verties[a]);
		BoundingBox.Grow(Verties[b]);
		BoundingBox.Grow(Verties[c]);

		m_NumTriangles++;
	}

	void AddAABB(const Vector3d& Bmin, const Vector3d& Bmax)
	{
		int k = m_NumVerties;

		Vector3d v[8];
		Box3d::GetVertices(Bmin, Bmax, v);
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

	bool LoadObj(const char* name)
	{
		char* buf = 0;
		FILE* fp = fopen(name, "rb");
		if (!fp)
		{
			return false;
		}
		_fseeki64(fp, 0, SEEK_END);
		unsigned long long bufSize = _ftelli64(fp);
		_fseeki64(fp, 0, SEEK_SET);
		buf = new char[bufSize];
		if (!buf)
		{
			fclose(fp);
			return false;
		}
		unsigned long long readLen = fread(buf, bufSize, 1, fp);
		fclose(fp);

		if (readLen != 1)
		{
			delete[] buf;
			return false;
		}

		char *p = buf, *pEnd = buf + bufSize;
		Vector3d v;
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
				unsigned int face[32];
				int nv = ParseFace(row + 1, face, 32);
				for (int i = 2; i < nv; ++i)
				{
					const unsigned int a = face[0];
					const unsigned int b = face[i - 1];
					const unsigned int c = face[i];
					if (a < 0 || a >= m_NumVerties || b < 0 || b >= m_NumVerties || c < 0 || c >= m_NumVerties)
						continue;
					AddTriangle(a, b, c);
				}
			}
		}

		delete[] buf;
		ResourceId = name;
		return true;
	}

	bool WriteFlat(const char* name)
	{
		FILE* fp = fopen(name, "wb");
		if (!fp)
		{
			return false;
		}

		unsigned int Magic = 0xF34D9017;
		fwrite(&Magic, sizeof(Magic), 1, fp);
		fwrite(&m_NumVerties, sizeof(m_NumVerties), 1, fp);
		fwrite(&m_NumTriangles, sizeof(m_NumTriangles), 1, fp);
		fwrite(&Verties[0], sizeof(Verties[0]), m_NumVerties, fp);
		fwrite(&Indices[0], sizeof(Indices[0]), m_NumTriangles * 3, fp);
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

		_fseeki64(fp, 0, SEEK_END);
		unsigned long long fileSize = _ftelli64(fp);
		_fseeki64(fp, 0, SEEK_SET);

		if (fileSize < 12)
		{
			return false;
		}

		unsigned int Magic;
		fread(&Magic, sizeof(Magic), 1, fp);
		if (Magic != 0xF34D9017)
		{
			return false;
		}

		fread(&m_NumVerties, sizeof(m_NumVerties), 1, fp);
		fread(&m_NumTriangles, sizeof(m_NumTriangles), 1, fp);
		Verties.resize(m_NumVerties);
		Indices.resize(m_NumTriangles * 3);
		fread(&Verties[0], sizeof(Verties[0]), m_NumVerties, fp);
		fread(&Indices[0], sizeof(Indices[0]), m_NumTriangles * 3, fp);
		fclose(fp);

		CalculateBoundingBox();
		return true;
	}

	int FilterTriangle(std::function<bool(const Vector3d&, const Vector3d&, const Vector3d&)> filter_func)
	{
		int j = 0;
		for (unsigned int i = 0; i < m_NumTriangles; ++i)
		{
			if (!filter_func(Verties[Indices[3*i]], Verties[Indices[3*i+1]], Verties[Indices[3*i+2]] ))
			{
				if (i != j)
				{
					Indices[3 * j + 0] = Indices[3 * i  +0];
					Indices[3 * j + 1] = Indices[3 * i + 1];
					Indices[3 * j + 2] = Indices[3 * i + 2];
				}
				++j;
			}
		}
		int nFiltered = m_NumTriangles - j;
		m_NumTriangles = j;
		return nFiltered;
	}

	void CalculateNormals()
	{
		if (m_NumTriangles == 0)
		{
			return;
		}

		if (!Normals.empty())
		{
			return;
		}

		std::vector<int> Count;
		Count.resize(m_NumVerties, 0);
		Normals.resize(m_NumVerties);
		memset(&Normals[0], 0, sizeof(Normals[0]) * Normals.size());
		for (unsigned int i = 0; i < m_NumTriangles; ++i)
		{
			const int i0 = Indices[3 * i + 0];
			const int i1 = Indices[3 * i + 1];
			const int i2 = Indices[3 * i + 2];
			const Vector3d& v0 = Verties[i0];
			const Vector3d& v1 = Verties[i1];
			const Vector3d& v2 = Verties[i2];
			Vector3d Nor = (v1 - v0).Cross(v2 - v0);
			Normals[i0] += Nor.Unit(), Count[i0]++;
			Normals[i1] += Nor.Unit(), Count[i1]++;
			Normals[i2] += Nor.Unit(), Count[i2]++;
		}
		
		for (unsigned int i = 0; i < m_NumVerties; ++i)
		{
			Normals[i] *= 1.0f / Count[i];
			Normals[i].Normalize();
		}
	}

	void CalculateBoundingBox()
	{
		if (m_NumTriangles == 0)
		{
			return;
		}

		BoundingBox = Box3d(&Verties[0], m_NumVerties);
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

	int ParseFace(char* row, unsigned int* data, int n)
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
			unsigned int vi = atoi(s);
			data[j++] = vi < 0 ? vi + m_NumVerties : vi - 1;
			if (j >= n) return j;
		}
		return j;
	}
};