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
				int buf_size = (int)m_owner->data.size();
				for (; m_curr_i < buf_size; ++m_curr_i)
				{
					if (m_owner->data[m_curr_i] == 0)
						continue;
					int k = (m_curr_i < buf_size - 1) ? 63 : ((m_owner->size - 1) & 63);
					for (; m_curr_j <= k; ++m_curr_j)
					{
						if (m_owner->data[m_curr_i] & (1LL << m_curr_j))
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
				m_index = (uint32_t)m_owner->size;
			}

			const BitSet *m_owner {nullptr};
			int			m_curr_i;
			int			m_curr_j;
			uint32_t	m_index;
		};

		class ValueProxy
		{
		public:
			bool operator=(bool val)
			{
				m_owner->Set(m_index, val);
				return val;
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
			size = _size;
			data.resize((size + 63) / 64, 0);
		}

		BitSet(const BitSet& rhs)
		{
			size = rhs.size;
			data = rhs.data;
		}

		BitSet(const std::initializer_list<uint32_t>& list)
		{
			size = 0;
			for (uint32_t i : list)
			{
				if (i > size)
					size = i;
			};
			size = size + 1;
			data.resize((size + 63) / 64, 0);
			for (uint32_t i : list)
			{
				data[i >> 6] |= (1LL << (i & 63));
			}
		}

		BitSet(size_t _size, const std::vector<uint32_t>& list)
		{
			size = _size;
			data.resize((size + 63) / 64, 0);
			for (uint32_t i : list)
			{
				data[i >> 6] |= (1LL << (i & 63));
			}
		}

		std::string ToString() const
		{
			if (size == 0)
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

		std::vector<uint32_t> ToVector() const
		{
			if (size == 0)
				return {};
			std::vector<uint32_t> ret;
			for (size_t i = 0; i < data.size(); ++i)
			{
				if (data[i] == 0)
					continue;
				int k = (i < data.size() - 1) ? 63 : ((size - 1) & 63);
				for (int j = 0; j <= k; ++j)
				{
					if (data[i] & (1LL << j))
					{
						ret.push_back((uint32_t)(i * 64 + j));
					}
				}
			}
			return ret;
		}

		size_t GetSize() const
		{
			return size;
		}

		void Resize(size_t new_size)
		{
			if (new_size == size)
			{
				return;
			}

			data.resize((new_size + 63) / 64);
			if (new_size < size)
			{
				size = new_size;
			}
			else
			{
				for (size_t i = (size + 63) / 64; i < data.size(); ++i)
				{
					data[i] = 0;
				}
				if (size > 0)
				{
					size_t k = (size + 63) / 64 - 1;
					for (int i = size & 63; i < 64; ++i)
					{
						data[k] &= ~(1LL << i);
					}
				}
				size = new_size;
			}
		}

		void Clear()
		{
			for (size_t i = 0; i < data.size(); ++i)
			{
				data[i] = 0;
			}
		}

		bool operator[](size_t i) const
		{
			return data[i >> 6] & (1LL << (i & 63)) ? true : false;
		}

		ValueProxy operator[](size_t i)
		{
			return ValueProxy(this, i);
		}

		const BitSet& operator=(const BitSet& rhs)
		{
			size = rhs.size;
			data = rhs.data;
			return *this;
		}

		bool Get(size_t i) const
		{
			return data[i >> 6] & (1LL << (i & 63)) ? true : false;
		}

		void Set(size_t i, bool b)
		{
			if (b)
			{
				data[i >> 6] |= (1LL << (i & 63));
			}
			else
			{
				data[i >> 6] &= ~(1LL << (i & 63));
			}
		}

		void Insert(size_t i)
		{
			data[i >> 6] |= (1LL << (i & 63));
		}

		void Erase(size_t i)
		{
			data[i >> 6] &= ~(1LL << (i & 63));
		}

		BitSet operator~() const
		{
			return SetComplement();
		}

		BitSet operator-() const
		{
			return SetComplement();
		}

		BitSet operator+(const BitSet& rhs) const
		{
			BitSet ret(std::max(data.size(), rhs.data.size()));
			for (size_t i = 0; i < ret.data.size(); ++i)
			{
				ret.data[i] = (i < data.size() ? data[i] : 0) | (i < rhs.data.size() ? rhs.data[i] : 0);
			}
			return ret;
		}

		void operator+=(const BitSet& rhs)
		{
			for (size_t i = 0; i < data.size(); ++i)
			{
				data[i] |= (i < rhs.data.size() ? rhs.data[i] : 0);
			}
		}

		BitSet operator-(const BitSet& rhs) const
		{
			BitSet ret(std::max(size, rhs.size));
			for (size_t i = 0; i < ret.data.size(); ++i)
			{
				ret.data[i] = (i < data.size() ? data[i] : 0) & ~(i < rhs.data.size() ? rhs.data[i] : 0);
			}
			return ret;
		}

		void operator-=(const BitSet& rhs)
		{
			for (size_t i = 0; i < data.size(); ++i)
			{
				data[i] &= (~(i < rhs.data.size() ? rhs.data[i] : 0));
			}
		}

		BitSet operator&(const BitSet& rhs) const
		{
			BitSet ret(std::max(size, rhs.size));
			for (size_t i = 0; i < std::min(data.size(), rhs.data.size()); ++i)
			{
				ret.data[i] = data[i] & rhs.data[i];
			}
			return ret;
		}

		void operator&=(const BitSet& rhs)
		{
			for (size_t i = 0; i < data.size(); ++i)
			{
				data[i] &= (i < rhs.data.size() ? rhs.data[i] : 0);
			}
		}

		BitSet SetComplement() const
		{
			BitSet ret(size);
			for (size_t i = 0; i < data.size(); ++i)
			{
				ret.data[i] = ~data[i];
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
		size_t					size;
		std::vector<uint64_t>   data;
	};

	inline BitSet SetUnion(const BitSet& lhs, const BitSet& rhs)
	{
		return lhs + rhs;
	}

	inline BitSet SetDifference(const BitSet& lhs, const BitSet& rhs)
	{
		return lhs - rhs;
	}

	inline BitSet SetIntersection(const BitSet& lhs, const BitSet& rhs)
	{
		return lhs & rhs;
	}
}
