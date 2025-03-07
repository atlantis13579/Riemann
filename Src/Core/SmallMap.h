#pragma once

#include <map>

namespace Riemann
{
	template<typename K, typename V, int BufferSize = 16>
	class SmallMap
	{
	public:
		struct Pair
		{
			K key;
			V value;
		};

		SmallMap()
		{
			m_size = 0;				// use buffer
		}

		void insert(const K &k, const V& v)
		{
			if (m_size == BufferSize - 1)
			{
				for (int i = 0; i < m_size; ++i)
				{
					m_map[m_buffer[i].key] = m_buffer[i].value;
				}
				m_size = -1;		// use set
			}

			if (m_size >= 0)
			{
				for (int i = 0; i < m_size; ++i)
				{
					if (m_buffer[i] == k)
					{
						return;
					}
				}
				m_buffer[m_size].key = k;
				m_buffer[m_size].value = v;
				m_size++;
			}
			else
			{
				m_map[k] = v;
			}
		}

		void erase(const K& key)
		{
			if (m_size >= 0)
			{
				for (int i = 0; i < m_size; ++i)
				{
					if (m_buffer[i].key == key)
					{
						m_buffer[i] = m_buffer[m_size - 1];
						m_size--;
						return;
					}
				}
			}
			else
			{
				m_map.erase(key);
			}
		}

		const V& operator[](const K& k) const
		{
			if (m_size >= 0)
			{
				for (int i = 0; i < m_size; ++i)
				{
					if (m_buffer[i] == k)
					{
						return m_buffer.value;
					}
				}
				const V *ptr = nullptr;
				return ptr[0];
			}
			else
			{
				m_map[k];
			}
		}

		V& operator[](const K& k)
		{
			if (m_size >= 0)
			{
				for (int i = 0; i < m_size; ++i)
				{
					if (m_buffer[i].key == k)
					{
						return m_buffer[i].value;
					}
				}

				if (m_size == BufferSize - 1)
				{
					for (int i = 0; i < m_size; ++i)
					{
						m_map[m_buffer[i].key] = m_buffer[i].value;
					}
					m_size = -1;		// use set
					return m_map[k];
				}

				m_size++;
				m_buffer[m_size - 1].key = k;
				return m_buffer[m_size - 1].value;
			}
			else
			{
				return m_map[k];
			}
		}

		bool haskey(const K& k) const
		{
			return count(k) > 0;
		}

		size_t count(const K& k) const
		{
			if (m_size >= 0)
			{
				for (int i = 0; i < m_size; ++i)
				{
					if (m_buffer[i].key == k)
					{
						return 1;
					}
				}
				return 0;
			}
			else
			{
				return m_map.count(k);
			}
		}

		void clear()
		{
			m_size = 0;
			m_map.clear();
		}

		size_t size() const
		{
			return m_size >= 0 ? (size_t)m_size : m_map.size();
		}

		bool empty() const
		{
			return m_size >= 0 ? m_size == 0 : m_map.empty();
		}


		class Iterator
		{
		public:
			inline Pair operator*() const
			{
				if (m_owner->m_size >= 0)
				{
					return m_owner->m_buffer[m_index];
				}
				else
				{
					Pair p;
					p.key = m_map_it->first;
					p.value = m_map_it->second;
					return p;
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
					m_map_it++;
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
					return m_map_it != rhs.m_map_it;
				}
			}

		private:
			friend class SmallMap;

			Iterator(const SmallMap<K, V>* _owner, bool is_end) : m_owner(_owner)
			{
				if (is_end)
				{
					m_index = m_owner->m_size;
					m_map_it = m_owner->m_map.end();
				}
				else
				{
					m_index = 0;
					m_map_it = m_owner->m_map.begin();
				}
			}

			const SmallMap<K, V>*					m_owner;
			int										m_index;
			typename std::map<K, V>::const_iterator	m_map_it;
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
		int				m_size;			//  = -1 use m_map 
		Pair			m_buffer[BufferSize];
		std::map<K, V>	m_map;
	};
}