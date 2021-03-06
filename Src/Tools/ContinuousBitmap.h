#pragma once

#include <string.h>
#include <vector>
#include <limits>

template <typename T>
class ContinuousBitmap
{
public:
	ContinuousBitmap()
	{
		m_nX = m_nY = 0;
	}
	~ContinuousBitmap()
	{
		m_Rows.clear();
		m_CompactData.clear();
	}

	bool	QueryBitmapSpace(T nx, T ny) const
	{
		if (nx < 0 || nx >= m_nX || ny < 0 || ny >= m_nY)
		{
			return false;
		}
		T* pX = m_Rows[ny];
		while (pX)
		{
			if (pX[0] == std::numeric_limits<T>::max())
				break;
			T low = pX[0];
			T high = pX[1];
			if (low <= nx && nx <= high)
				return true;
			else if (nx < low)
				break;
			pX = pX + 2;
		}
		return false;
	}

	bool	QueryWorldSpace(float x, float y) const
	{
		x -= m_minX;
		y -= m_minY;
		if (x < 0 || x >= m_maxX - m_minX || y < 0 || y >= m_maxY - m_minY)
			return false;
		T nx = (T)(x * m_invX);
		T ny = (T)(y * m_invY);
		return QueryBitmapSpace(nx, ny);
	}

	void	SetTransform(int nX, int nY, float minX, float minY, float maxX, float maxY)
	{
		m_nX = nX;
		m_nY = nY;
		m_minX = minX;
		m_minY = minY;
		m_maxX = maxX;
		m_maxY = maxY;
		m_invX = 1.0f * m_nX / (maxX - minX);
		m_invY = 1.0f * m_nY / (maxY - minY);
	}

	template <typename TSrc>
	void	Build(TSrc* pArray, int nX, int nY, float minX, float minY, float maxX, float maxY)
	{
		if (!m_Rows.empty())
		{
			m_Rows.clear();
			m_CompactData.clear();
		}

		SetTransform(nX, nY, minX, minY, maxX, maxY);

		m_Rows.resize(m_nY);
		for (int i = 0; i < nY; ++i)
		{
			m_Rows[i] = (T*)m_CompactData.size();

			TSrc* p = pArray + i * nX;
			for (int j = 0; j < nX; ++j)
			{
				if (p[j] != 0)
				{
					m_CompactData.push_back(j);
					while (p[j] != 0 && j < nX)
					{
						++j;
					}
					m_CompactData.push_back(j - 1);
				}
			}
			m_CompactData.push_back(std::numeric_limits<T>::max());
		}

		for (int i = 0; i < nY; ++i)
		{
			m_Rows[i] = &m_CompactData[(uint64_t)m_Rows[i]];
		}
	}

	bool SerializeToFile(const char* filename)
	{
		FILE* fp = fopen(filename, "wb");
		if (!fp)
		{
			return false;
		}

		uint32_t Magic = 0x353BA50D;
		int nData = (int)m_CompactData.size();
		int nElementSize = sizeof(T);
		int nUnuse = 0;
		fwrite(&Magic, 4, 1, fp);
		fwrite(&m_nX, 4, 1, fp);
		fwrite(&m_nY, 4, 1, fp);
		fwrite(&m_minX, 4, 1, fp);
		fwrite(&m_minY, 4, 1, fp);
		fwrite(&m_maxX, 4, 1, fp);
		fwrite(&m_maxY, 4, 1, fp);
		fwrite(&m_invX, 4, 1, fp);
		fwrite(&m_invY, 4, 1, fp);
		fwrite(&nData, 4, 1, fp);
		fwrite(&nElementSize, 4, 1, fp);
		fwrite(&nUnuse, 4, 1, fp);
		fwrite(&m_CompactData[0], sizeof(T), m_CompactData.size(), fp);
		fclose(fp);
		return true;
	}

	bool SerializeFromFile(const char* filename)
	{
		if (!m_Rows.empty())
		{
			m_Rows.clear();
			m_CompactData.clear();
		}

		FILE* fp = fopen(filename, "rb");
		if (!fp)
		{
			return false;
		}

		fseek(fp, 0, SEEK_END);
		size_t fileSize = ftell(fp);
		fseek(fp, 0, SEEK_SET);

		if (fileSize < 48)
		{
			return false;
		}

		uint32_t Magic = 0;
		int nData = 0;
		int nElementSize = 0;
		int nUnuse = 0;
		fread(&Magic, 4, 1, fp);
		fread(&m_nX, 4, 1, fp);
		fread(&m_nY, 4, 1, fp);
		fread(&m_minX, 4, 1, fp);
		fread(&m_minY, 4, 1, fp);
		fread(&m_maxX, 4, 1, fp);
		fread(&m_maxY, 4, 1, fp);
		fread(&m_invX, 4, 1, fp);
		fread(&m_invY, 4, 1, fp);
		fread(&nData, 4, 1, fp);
		fread(&nElementSize, 4, 1, fp);
		fread(&nUnuse, 4, 1, fp);
		if (Magic != 0x353BA50D)
		{
			fclose(fp);
			return false;
		}
		if (nElementSize != sizeof(T))
		{
			fclose(fp);
			return false;
		}
		m_CompactData.resize(nData);
		fread(&m_CompactData[0], sizeof(T), m_CompactData.size(), fp);
		fclose(fp);

		m_Rows.resize(m_nY);
		memset(&m_Rows[0], 0, sizeof(T*) * m_Rows.size());
		
		T* p = &m_CompactData[0];
		for (int i = 0; i < m_nY; ++i)
		{
			m_Rows[i] = p;
			while (*p != std::numeric_limits<T>::max())
			{
				p = p + 2;
			}
			p = p + 1;
		}

		return true;
	}

private:
	int					m_nX, m_nY;
	float				m_minX, m_minY;
	float				m_maxX, m_maxY;
	float				m_invX, m_invY;
	std::vector<T>		m_CompactData;
	std::vector<T*>		m_Rows;
};
