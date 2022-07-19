
#pragma once

template<class T, int MaxSize>
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

	inline void Push(T v)
	{
		m_Buf[m_Size++] = v;
	}

	inline T Pop()
	{
		if (m_Size > 0)
			return m_Buf[--m_Size];
		else
			return 0;
	}

	inline void SetEmpty()
	{
		m_Size = 0;
	}

	inline int Size() const
	{
		return m_Size;
	}

private:
	int		m_Size;
	T		m_Buf[MaxSize];
};