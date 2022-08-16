#pragma once

#include <stdio.h>
#include <vector>

namespace libMat
{
#define MAX_DIMS 4

enum DataType
{
	TYPE_U8 = 0,
	TYPE_I8,
	TYPE_U16,
	TYPE_I16,
	TYPE_U32,
	TYPE_I32,
	TYPE_U64,
	TYPE_I64,
	TYPE_FP32,
	TYPE_FP64,
};

inline int ElementSize(int type)
{
	static const int sizeTable[] = { 1, 1, 2, 2, 4, 4, 8, 8, 4, 8 };
	return sizeTable[type];
}

inline bool _WriteMatToFile(const char* filename, const int type, const int dim, const int dims[MAX_DIMS], const void* buffer)
{
	FILE* fp = fopen(filename, "wb");
	if (fp)
	{
		int tt = (type << 16) | dim;
		fwrite(&tt, sizeof(int), 1, fp);
		fwrite(dims, sizeof(int), dim, fp);
		int nSize = 1;
		for (int i = 0; i < dim; ++i)
		{
			nSize *= dims[i];
		}
		int elementSize = ElementSize(type);
		fwrite(buffer, elementSize, nSize, fp);
		fclose(fp);
		return true;
	}
	return false;
}

inline bool _LoadMatFromFile(const char* filename, int& type, int dim, int dims[MAX_DIMS], std::vector<unsigned char>& buffer)
{
	FILE* fp = fopen(filename, "rb");
	if (fp)
	{
		int tt;
		fread(&tt, sizeof(int), 1, fp);
		type = (tt & 0xFFFF0000) >> 16;
		dim = (tt & 0x0000FFFF);

		fread(dims, sizeof(int), dim, fp);

		int nSize = 1;
		for (int i = 0; i < dim; ++i)
		{
			nSize *= dims[i];
		}
		int elementSize = ElementSize(type);
		buffer.resize(nSize * elementSize);
		fread(&buffer[0], elementSize, nSize, fp);
		fclose(fp);
		return true;
	}
	return false;
}

class Mat2
{
public:
	static bool WriteToFile(const char* filename, const int w, const int h, const float* buffer)
	{
		int dims[2] = { w, h };
		return _WriteMatToFile(filename, TYPE_FP32, 2, dims, (const void*)buffer);
	}

	static bool LoadFromFile(const char* filename, int &w, int &h, std::vector<unsigned char>& buffer)
	{
		int dims[MAX_DIMS] = { 0 };
		int type;
		bool succ = _LoadMatFromFile(filename, type, 2, dims, buffer);
		if (succ)
		{
			w = dims[0];
			h = dims[1];
		}
		return succ;
	}
};
}
