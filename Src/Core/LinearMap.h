#pragma once

namespace Riemann
{
	template<typename K, typename V, int BufferSize = 16>
	class LinearMap
	{
	public:
		struct Pair
		{
			K key;
			V value;
		};

		LinearMap()
		{
			m_size = 0;				// use buffer
		}

		bool insert(const K &k, const V& v)
		{
			if (m_size == BufferSize - 1)
			{
				return false;
			}

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

		bool erase(const K& key)
		{
			for (int i = 0; i < m_size; ++i)
			{
				if (m_buffer[i].key == key)
				{
					m_buffer[i] = m_buffer[m_size - 1];
					m_size--;
					return true;
				}
			}
			return false;
		}

		const V& operator[](const K& k) const
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

		V& operator[](const K& k)
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
				V* ptr = nullptr;
				return ptr[0];
			}

			m_size++;
			m_buffer[m_size - 1].key = k;
			return m_buffer[m_size - 1].value;
		}

		bool haskey(const K& k) const
		{
			return count(k) > 0;
		}

		size_t count(const K& k) const
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

		void clear()
		{
			m_size = 0;
		}

		size_t size() const
		{
			return (size_t)m_size;
		}

		bool empty() const
		{
			return m_size == 0;
		}


		class Iterator
		{
		public:
			inline Pair operator*() const
			{
				return m_owner->m_buffer[m_index];
			}

			inline const Iterator& operator++()
			{
				m_index++;
				return *this;
			}

			inline bool operator != (const Iterator& rhs) const
			{
				return m_index != rhs.m_index;
			}

		private:
			friend class LinearMap;

			Iterator(const LinearMap<K, V>* _owner, bool is_end) : m_owner(_owner)
			{
				if (is_end)
				{
					m_index = m_owner->m_size;
				}
				else
				{
					m_index = 0;
				}
			}

			const LinearMap<K, V>*					m_owner;
			int										m_index;
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
		int				m_size;
		Pair			m_buffer[BufferSize];
	};
}