
#include "GeometrySet.h"

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

	template<typename T>
	static void AddVertex(std::vector<T> &Vertices, int &NumVertices, const T& v)
	{
		const int OBJ_BATCH_SIZE = 4096;
		if (NumVertices >= Vertices.size())
		{
			Vertices.resize(Vertices.size() + OBJ_BATCH_SIZE * 3);
		}
		Vertices[NumVertices++] = v;
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
					AddVertex(Vertices, NumVertices, v);
					AddVertex(VertexColors, NumColors, c);
				}
				else
				{
					Vector3 v;
					sscanf(row + 1, "%f %f %f", &v.x, &v.y, &v.z);
					AddVertex(Vertices, NumVertices, v);
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

		Vertices.resize(NumVertices);

		delete[] buf;

		Bounds = Box3d(Vertices.data(), Vertices.size());
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

		fprintf(fp, "# %d vertices, %d faces\n", (int)Vertices.size(), (int)Triangles.size());

		fprintf(fp, "# List of geometric vertices, with (x,y,z[,w]) coordinates, w is optional and defaults to 1.0. \n");
		if (HasVertexColors())
		{
			for (size_t i = 0; i < Vertices.size(); ++i)
			{
				const Vector3 v = Vertices[i];
				const Vector3 c = VertexColors[i];
				fprintf(fp, "v %.3f %.3f %.3f %.3f %.3f %.3f\n", v.x, v.y, v.z, c.x, c.y, c.z);
			}
		}
		else
		{
			for (size_t i = 0; i < Vertices.size(); ++i)
			{
				const Vector3 v = Vertices[i];
				fprintf(fp, "v %.3f %.3f %.3f\n", v.x, v.y, v.z);
			}
		}

		fprintf(fp, "# Polygonal face element (see below) \n");
		for (size_t i = 0; i < Triangles.size(); ++i)
		{
			const Vector3i v = Triangles[i];
			fprintf(fp, "f %d %d %d\n", v.x + 1, v.y + 1, v.z + 1);
		}

		if (HasVertexNormals())
		{
			fprintf(fp, "# List of vertex normals in (x,y,z) form; normals might not be unit vectors. \n");
			for (size_t i = 0; i < VertexNormals.size(); ++i)
			{
				const Vector3 v = VertexNormals[i];
				fprintf(fp, "vn %.3f %.3f %.3f\n", v.x, v.y, v.z);
			}
		}

		if (HasVertexUVs())
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

}	// namespace Destruction
