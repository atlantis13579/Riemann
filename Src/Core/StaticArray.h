#pragma once

namespace Riemann
{
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

		inline bool Add(const T& v)
		{
			if (size < Capacity)
			{
				data[size++] = v;
				return true;
			}
			return false;
		}

		template<typename ... Ts>
		inline void Emplace(const Ts &... args)
		{
			if (size < Capacity)
			{
				data[size++] = T(args ...);
				return true;
			}
			return false;
		}

		inline bool InsertAt(int idx, const T& v, bool preserve_order = true)
		{
			if (size >= Capacity)
			{
				return false;
			}

			if (preserve_order)
			{
				for (int i = size; i > idx; --i)
				{
					data[i] = data[i - 1];
				}
				data[idx] = v;
			}
			else
			{
				data[size] = data[idx];
				data[idx] = v;
			}

			size++;
		}

		inline bool RemoveAt(int idx, bool preserve_order = true)
		{
			if (idx < 0 || idx >= size)
			{
				return false;
			}
			if (preserve_order)
			{
				for (int i = idx; i < size - 1; ++i)
				{
					data[i] = data[i + 1];
				}
			}
			else
			{
				data[idx] = data[size - 1];
			}

			size--;
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
}