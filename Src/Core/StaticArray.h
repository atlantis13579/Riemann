#pragma once

namespace Riemann
{
	template<class T, int Capacity>
	class StaticArray
	{
	public:
		StaticArray()
		{
			m_size = 0;
		}

		~StaticArray()
		{
		}

		inline T& operator[](int i)
		{
			return m_data[i];
		}

		inline const T& operator[](int i) const
		{
			return m_data[i];
		}

		inline T* data()
		{
			return m_data;
		}

		inline const T* data() const
		{
			return m_data;
		}

		inline int* GetSizeData()
		{
			return &m_size;
		}

		inline T* add()
		{
			if (m_size >= Capacity)
				return nullptr;
			return m_data[m_size++];
		}

		inline bool add(const T& v)
		{
			if (m_size < Capacity)
			{
				m_data[m_size++] = v;
				return true;
			}
			return false;
		}

		template<typename ... Ts>
		inline void emplace(const Ts &... args)
		{
			if (m_size < Capacity)
			{
				m_data[m_size++] = T(args ...);
				return true;
			}
			return false;
		}

		inline bool insert_at(int idx, const T& v, bool preserve_order = true)
		{
			if (m_size >= Capacity)
			{
				return false;
			}

			if (preserve_order)
			{
				for (int i = m_size; i > idx; --i)
				{
					m_data[i] = m_data[i - 1];
				}
				m_data[idx] = v;
			}
			else
			{
				m_data[m_size] = m_data[idx];
				m_data[idx] = v;
			}

			m_size++;
		}

		inline bool remove_at(int idx, bool preserve_order = true)
		{
			if (idx < 0 || idx >= m_size)
			{
				return false;
			}
			if (preserve_order)
			{
				for (int i = idx; i < m_size - 1; ++i)
				{
					m_data[i] = m_data[i + 1];
				}
			}
			else
			{
				m_data[idx] = m_data[m_size - 1];
			}

			m_size--;
		}

		inline void clear()
		{
			m_size = 0;
		}

		inline void resize(int s)
		{
			m_size = s;
		}

		inline int size() const
		{
			return m_size;
		}

		inline int capacity() const
		{
			return Capacity;
		}

	private:
		int		m_size;
		T		m_data[Capacity];
	};
}