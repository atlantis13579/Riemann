#pragma once

namespace Riemann
{
	template<typename T, int BufferSize = 16>
	class LinearSet
	{
	public:
		LinearSet()
		{
			m_size = 0;				// use buffer
		}

		bool insert(const T& v)
		{
			for (int i = 0; i < m_size; ++i)
			{
				if (m_buffer[i] == v)
				{
					return true;
				}
			}

			if (m_size == BufferSize - 1)
			{
				return false;
			}

			m_buffer[m_size++] = v;
			return true;
		}

		bool erase(const T& v)
		{
			for (int i = 0; i < m_size; ++i)
			{
				if (m_buffer[i] == v)
				{
					m_buffer[i] = m_buffer[m_size - 1];
					m_size--;
					return true;
				}
			}
			return false;
		}

		bool contains(const T& v) const
		{
			return count(v) > 0;
		}

		size_t count(const T& v) const
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

		std::vector<T> to_vector() const
		{
			std::vector<T> ret;
			for (int i = 0; i < m_size; ++i)
			{
				ret.push_back(m_buffer[i]);
			}
			return ret;
		}

		class Iterator
		{
		public:
			inline const T& operator*() const
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
			friend class LinearSet;

			Iterator(const LinearSet<T>* _owner, bool is_end) : m_owner(_owner)
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

			const LinearSet<T>*						m_owner;
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
		int			m_size;
		T			m_buffer[BufferSize];
	};
}