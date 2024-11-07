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

		bool contains(const T& v) const
		{
			return count(v) > 0;
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

		class Iterator
		{
		public:
			inline const T& operator*() const
			{
				if (m_owner->m_size >= 0)
				{
					return m_owner->m_buffer[m_index];
				}
				else
				{
					return *m_set_it;
				}
			}

			inline const Iterator& operator++()
			{
				if (m_owner->m_size >= 0)
				{
					m_index++;
				}
				else
				{
					m_set_it++;
				}
				return *this;
			}

			inline bool operator != (const Iterator& rhs) const
			{
				if (m_owner->m_size >= 0)
				{
					return m_index != rhs.m_index;
				}
				else
				{
					return m_set_it != rhs.m_set_it;
				}
			}

		private:
			friend class SmallSet;

			Iterator(const SmallSet<T>* _owner, bool is_end) : m_owner(_owner)
			{
				if (is_end)
				{
					m_index = m_owner->m_size;
					m_set_it = m_owner->m_set.end();
				}
				else
				{
					m_index = 0;
					m_set_it = m_owner->m_set.begin();
				}
			}

			const SmallSet<T>*						m_owner;
			int										m_index;
			typename std::set<T>::const_iterator	m_set_it;
		};

		Iterator begin() const
		{
			return Iterator(this, false);
		}

		Iterator end() const
		{
			return Iterator(this, true);
		}

	private:
		int			m_size;			//  = -1 use m_set 
		T			m_buffer[BufferSize];
		std::set<T>	m_set;
	};
}