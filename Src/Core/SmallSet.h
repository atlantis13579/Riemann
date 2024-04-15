#pragma once

#include <set>

namespace Riemann
{
	template<typename T, int BufferSize = 16>
	class SmallSet
	{
	public:
		SmallSet()
		{
			m_size = 0;				// use buffer
		}

		void insert(const T& v)
		{
			if (m_size == BufferSize - 1)
			{
				for (int i = 0; i < m_size; ++i)
				{
					m_set.insert(m_buffer[i]);
				}
				m_size = -1;		// use set
			}

			if (m_size >= 0)
			{
				for (int i = 0; i < m_size; ++i)
				{
					if (m_buffer[i] == v)
					{
						return;
					}
				}
				m_buffer[m_size++] = v;
			}
			else
			{
				m_set.insert(v);
			}
		}

		void erase(const T& v)
		{
			if (m_size >= 0)
			{
				for (int i = 0; i < m_size; ++i)
				{
					if (m_buffer[i] == v)
					{
						m_buffer[i] = m_buffer[m_size - 1];
						m_size--;
						return;
					}
				}
			}
			else
			{
				m_set.erase(v);
			}
		}

		size_t count(const T& v) const
		{
			if (m_size >= 0)
			{
				for (int i = 0; i < m_size; ++i)
				{
					if (m_buffer[i] == v)
					{
						return 1;
					}
				}
				return 0;
			}
			else
			{
				return m_set.count(v);
			}
		}

		void clear()
		{
			m_size = 0;
			m_set.clear();
		}

		size_t size() const
		{
			return m_size >= 0 ? (size_t)m_size : m_set.size();
		}

		bool empty() const
		{
			return m_size >= 0 ? m_size == 0 : m_set.empty();
		}

		std::vector<T> to_vector() const
		{
			std::vector<T> ret;
			if (m_size >= 0)
			{
				for (int i = 0; i < m_size; ++i)
				{
					ret.push_back(m_buffer[i]);
				}
			}
			else
			{
				for (T v : m_set)
				{
					ret.push_back(v);
				}
			}
			return ret;
		}

	private:
		int			m_size;			//  = -1 use m_set 
		T			m_buffer[BufferSize];
		std::set<T>	m_set;
	};
}