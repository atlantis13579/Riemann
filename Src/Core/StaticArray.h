#pragma once

template<class T, int Capacity>
class StaticArray
{
public:
	StaticArray()
	{
		size = 0;
	}

	~StaticArray()
	{
	}

	inline T& operator[](int i)
	{
		return data[i];
	}

	inline const T& operator[](int i) const
	{
		return data[i];
	}

	inline T* GetData()
	{
		return data;
	}

	inline const T* GetData() const
	{
		return data;
	}

	inline int* GetSizeData()
	{
		return &size;
	}

	inline T* Add()
	{
		if (size >= Capacity)
			return nullptr;
		return data[size++];
	}

	inline bool Push(const T& v)
	{
		if (size < Capacity)
		{
			data[size++] = v;
			return true;
		}
		return false;
	}

	inline T Pop()
	{
		return size > 0 ? data[--size] : T();
	}

	inline void Clear()
	{
		size = 0;
	}

	inline void SetSize(int s)
	{
		size = s;
	}

	inline int GetSize() const
	{
		return size;
	}

	inline int GetCapacity() const
	{
		return Capacity;
	}

private:
	int		size;
	T		data[Capacity];
};
