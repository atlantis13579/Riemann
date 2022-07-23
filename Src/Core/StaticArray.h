
#pragma once

template<class T, int MAX_SIZE>
class StaticArray
{
public:
	StaticArray()
	{
		m_Size = 0;
	}

	~StaticArray()
	{
	}

	inline T& operator[](int i)
	{
		return m_Buf[i];
	}

	inline const T& operator[](int i) const
	{
		return m_Buf[i];
	}

	inline T* GetData()
	{
		return m_Buf;
	}

	inline const T* GetData() const
	{
		return m_Buf;
	}

	inline T* Add()
	{
		if (m_Size >= MAX_SIZE)
			return nullptr;
		return m_Buf[m_Size++];
	}

	inline void Push(T v)
	{
		if (m_Size < MAX_SIZE)
		{
			m_Buf[m_Size++] = v;
		}
	}

	inline T Pop()
	{
		return m_Size > 0 ? m_Buf[--m_Size] : T();
	}

	inline void Clear()
	{
		m_Size = 0;
	}

	inline void SetSize(int Size)
	{
		m_Size = Size;
	}

	inline int GetSize() const
	{
		return m_Size;
	}

	inline int GetMaxSize() const
	{
		return MAX_SIZE;
	}

private:
	int		m_Size;
	T		m_Buf[MAX_SIZE];
};