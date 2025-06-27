#pragma once

#include <string>
#include <vector>

namespace Riemann
{
	class BitSet
	{
	public:
		class ItemIterator
		{
		public:
			inline uint32_t operator*() const
			{
				return m_index;
			}

			inline const ItemIterator& operator++()
			{
				if (!goto_next())
				{
					goto_end();
					return *this;
				}
				return *this;
			}

			inline bool operator != (const ItemIterator& rhs) const
			{
				return m_index != rhs.m_index;
			}

		private:
			friend class BitSet;

			ItemIterator(const BitSet* _owner, bool is_end) : m_owner(_owner)
			{
				m_curr_i = 0;
				m_curr_j = 0;
				m_index = 0;

				if (is_end || !goto_next())
				{
					goto_end();
				}
			}

			bool goto_next()
			{
				int buf_size = (int)m_owner->m_data.size();
				for (; m_curr_i < buf_size; ++m_curr_i)
				{
					if (m_owner->m_data[m_curr_i] == 0)
						continue;
					int k = (m_curr_i < buf_size - 1) ? 63 : ((m_owner->m_size - 1) & 63);
					for (; m_curr_j <= k; ++m_curr_j)
					{
						if (m_owner->m_data[m_curr_i] & (1LL << m_curr_j))
						{
							m_index = (uint32_t)(m_curr_i * 64 + m_curr_j);

							if (m_curr_j == 63)
							{
								m_curr_i += 1;
								m_curr_j = 0;
							}
							else
							{
								m_curr_j += 1;
							}
							return true;
						}
					}

					m_curr_j = 0;
				}
				return false;
			}

			void goto_end()
			{
				m_index = (uint32_t)m_owner->m_size;
			}

			const BitSet *m_owner {nullptr};
			int			m_curr_i;
			int			m_curr_j;
			uint32_t	m_index;
		};

		class ValueProxy
		{
		public:
			explicit operator bool() const
			{
				return m_owner->get(m_index);
			}

			ValueProxy& operator=(const bool val)
			{
				m_owner->set(m_index, val);
				return *this;
			}

		private:
			friend class BitSet;

			ValueProxy(BitSet* _owner, size_t index) : m_owner(_owner), m_index(index)
			{
			}

			BitSet* m_owner{ nullptr };
			size_t m_index;
		};

		explicit BitSet(size_t _size = 0)
		{
			m_size = _size;
			m_data.resize((m_size + 63) / 64, 0);
		}

		BitSet(const BitSet& rhs)
		{
			m_size = rhs.m_size;
			m_data = rhs.m_data;
		}

		BitSet(const std::initializer_list<uint32_t>& list)
		{
			m_size = 0;
			for (uint32_t i : list)
			{
				if (i > m_size)
					m_size = i;
			};
			m_size = m_size + 1;
			m_data.resize((m_size + 63) / 64, 0);
			for (uint32_t i : list)
			{
				m_data[i >> 6] |= (1LL << (i & 63));
			}
		}

		BitSet(size_t _size, const std::vector<uint32_t>& list)
		{
			m_size = _size;
			m_data.resize((m_size + 63) / 64, 0);
			for (uint32_t i : list)
			{
				m_data[i >> 6] |= (1LL << (i & 63));
			}
		}

		std::string to_string() const
		{
			if (m_size == 0)
				return "";

			std::string str;
			bool first_item = true;
			for (ItemIterator it = begin(); it != end(); ++it)
			{
				if (first_item)
				{
					str += std::to_string(*it);
					first_item = false;
				}
				else
				{
					str += ", " + std::to_string(*it);
				}
			}

			return str;
		}

		std::vector<uint32_t> to_vector() const
		{
			if (m_size == 0)
				return {};
			std::vector<uint32_t> ret;
			for (size_t i = 0; i < m_data.size(); ++i)
			{
				if (m_data[i] == 0)
					continue;
				int k = (i < m_data.size() - 1) ? 63 : ((m_size - 1) & 63);
				for (int j = 0; j <= k; ++j)
				{
					if (m_data[i] & (1LL << j))
					{
						ret.push_back((uint32_t)(i * 64 + j));
					}
				}
			}
			return ret;
		}

		size_t size() const
		{
			return m_size;
		}

		bool	empty() const
		{
			return m_size == 0;
		}

		void resize(size_t new_size)
		{
			if (new_size == m_size)
			{
				return;
			}

			m_data.resize((new_size + 63) / 64);
			if (new_size < m_size)
			{
				m_size = new_size;
			}
			else
			{
				for (size_t i = (m_size + 63) / 64; i < m_data.size(); ++i)
				{
					m_data[i] = 0;
				}
				if (m_size > 0)
				{
					size_t k = (m_size + 63) / 64 - 1;
					for (int i = m_size & 63; i < 64; ++i)
					{
						m_data[k] &= ~(1LL << i);
					}
				}
				m_size = new_size;
			}
		}

		void clear()
		{
			for (size_t i = 0; i < m_data.size(); ++i)
			{
				m_data[i] = 0;
			}
		}

		bool operator[](size_t i) const
		{
			return m_data[i >> 6] & (1LL << (i & 63)) ? true : false;
		}

		ValueProxy operator[](size_t i)
		{
			return ValueProxy(this, i);
		}

		BitSet& operator=(const BitSet& rhs)
		{
			m_size = rhs.m_size;
			m_data = rhs.m_data;
			return *this;
		}

		inline bool get(size_t i) const
		{
			return m_data[i >> 6] & (1LL << (i & 63)) ? true : false;
		}

		inline bool get_safe(size_t i) const
		{
			if (i < m_size)
			{
				return get(i);
			}
			return false;
		}

		inline void set(size_t i, bool b)
		{
			if (b)
			{
				m_data[i >> 6] |= (1LL << (i & 63));
			}
			else
			{
				m_data[i >> 6] &= ~(1LL << (i & 63));
			}
		}

		inline void set_safe(size_t i, bool b)
		{
			if (i < m_size)
			{
				set(i, b);
				return;
			}

			if (b)
			{
				resize(i + 1);
				set(i, b);
			}
		}

		inline void insert(size_t i)
		{
			m_data[i >> 6] |= (1LL << (i & 63));
		}

		inline void erase(size_t i)
		{
			m_data[i >> 6] &= ~(1LL << (i & 63));
		}

		BitSet operator~() const
		{
			return set_complement();
		}

		BitSet operator-() const
		{
			return set_complement();
		}

		BitSet operator+(const BitSet& rhs) const
		{
			BitSet ret(std::max(m_data.size(), rhs.m_data.size()));
			for (size_t i = 0; i < ret.m_data.size(); ++i)
			{
				ret.m_data[i] = (i < m_data.size() ? m_data[i] : 0) | (i < rhs.m_data.size() ? rhs.m_data[i] : 0);
			}
			return ret;
		}

		void operator+=(const BitSet& rhs)
		{
			for (size_t i = 0; i < m_data.size(); ++i)
			{
				m_data[i] |= (i < rhs.m_data.size() ? rhs.m_data[i] : 0);
			}
		}

		BitSet operator-(const BitSet& rhs) const
		{
			BitSet ret(std::max(m_size, rhs.m_size));
			for (size_t i = 0; i < ret.m_data.size(); ++i)
			{
				ret.m_data[i] = (i < m_data.size() ? m_data[i] : 0) & ~(i < rhs.m_data.size() ? rhs.m_data[i] : 0);
			}
			return ret;
		}

		void operator-=(const BitSet& rhs)
		{
			for (size_t i = 0; i < m_data.size(); ++i)
			{
				m_data[i] &= (~(i < rhs.m_data.size() ? rhs.m_data[i] : 0));
			}
		}

		BitSet operator&(const BitSet& rhs) const
		{
			BitSet ret(std::max(m_size, rhs.m_size));
			for (size_t i = 0; i < std::min(m_data.size(), rhs.m_data.size()); ++i)
			{
				ret.m_data[i] = m_data[i] & rhs.m_data[i];
			}
			return ret;
		}

		void operator&=(const BitSet& rhs)
		{
			for (size_t i = 0; i < m_data.size(); ++i)
			{
				m_data[i] &= (i < rhs.m_data.size() ? rhs.m_data[i] : 0);
			}
		}

		BitSet set_complement() const
		{
			BitSet ret(m_size);
			for (size_t i = 0; i < m_data.size(); ++i)
			{
				ret.m_data[i] = ~m_data[i];
			}
			return ret;
		}

		ItemIterator begin() const
		{
			return ItemIterator(this, false);
		}

		ItemIterator end() const
		{
			return ItemIterator(this, true);
		}

	private:
		size_t					m_size;
		std::vector<uint64_t>   m_data;
	};

	inline BitSet set_union(const BitSet& lhs, const BitSet& rhs)
	{
		return lhs + rhs;
	}

	inline BitSet set_difference(const BitSet& lhs, const BitSet& rhs)
	{
		return lhs - rhs;
	}

	inline BitSet set_intersection(const BitSet& lhs, const BitSet& rhs)
	{
		return lhs & rhs;
	}
}
