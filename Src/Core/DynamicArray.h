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

		inline T* GetData()
		{
			return this->data();
		}

		inline const T* GetData() const
		{
			return  this->data();
		}

		inline T& GetAt(int i)
		{
			return this->at(i);
		}

		inline const T& GetAt(int i) const
		{
			return this->at(i);
		}

		inline void SetAt(int i, const T& v)
		{
			this->at(i) = v;
		}

		inline void Add(const T& v)
		{
			this->push_back(v);
		}

		template<typename ... Ts>
		inline void Emplace(const Ts &... args)
		{
			this->emplace_back(args...);
		}

		inline void InsertAt(int idx, const T& v, bool preserve_order = true)
		{
			if (preserve_order)
			{
				this->insert(this->begin() + idx, v);
			}
			else
			{
				this->resize(this->size() + 1);
				T *p = this->data();
				p[this->size() - 1] = p[idx];
				p[idx] = v;
			}
		}

		inline void RemoveAt(int idx, bool preserve_order = true)
		{
			if (preserve_order)
			{
				this->erase(this->begin() + idx);
			}
			else
			{
				T* p = this->data();
				p[idx] = p[this->size() - 1];
				this->resize(this->size() - 1);
			}
		}

		inline void Clear()
		{
			this->clear();
		}

		inline void Reserve(int s)
		{
			this->reserve(s);
		}

		inline void SetSize(int s)
		{
			this->resize(s);
		}

		inline int GetSize() const
		{
			return (int)this->size();
		}

		inline void Append(const DynamicArray<T> &rhs)
		{
			this->insert(this->begin(), rhs.begin(), rhs.end());
		}
	};
}