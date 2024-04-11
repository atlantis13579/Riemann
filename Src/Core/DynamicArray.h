#pragma once

#include <vector>

namespace Riemann
{
	template<class T>
	class DynamicArray : public std::vector<T>
	{
	public:
		DynamicArray()
		{
		}

		DynamicArray(int i) : std::vector<T>(i)
		{
		}

		DynamicArray(int i, const T& default_val) : std::vector<T>(i, default_val)
		{
		}

		DynamicArray(const std::initializer_list<T> &list) : std::vector<T>(list)
		{
		}

		~DynamicArray()
		{
		}

		inline void insert_at(int idx, const T& v, bool preserve_order = true)
		{
			if (preserve_order)
			{
				insert(this->begin() + idx, v);
			}
			else
			{
				resize(this->size() + 1);
				T *p = this->data();
				p[this->size() - 1] = p[idx];
				p[idx] = v;
			}
		}

		inline void remove_at(int idx, bool preserve_order = true)
		{
			if (preserve_order)
			{
				erase(this->begin() + idx);
			}
			else
			{
				T* p = this->data();
				p[idx] = p[this->size() - 1];
				resize(this->size() - 1);
			}
		}

		inline void append(const DynamicArray<T> &rhs)
		{
			insert(this->begin(), rhs.begin(), rhs.end());
		}
	};
}