#pragma once

#include <string>
#include <vector>

class bit_set
{
public:
	explicit bit_set(size_t size = 0)
	{
		m_size = size;
		m_set.resize((m_size + 63) / 64, 0);
	}

	bit_set(const bit_set& rhs)
	{
		m_size = rhs.m_size;
		m_set = rhs.m_set;
	}

	bit_set(const std::initializer_list<uint32_t>& list)
	{
		m_size = 0;
		for (uint32_t i : list)
		{
			if (i > m_size)
				m_size = i;
		};
		m_size = m_size + 1;
		m_set.resize((m_size + 63) / 64, 0);
		for (uint32_t i : list)
		{
			m_set[i >> 6] |= (1LL << (i & 63));
		}
	}

	bit_set(size_t size, const std::vector<uint32_t>& list)
	{
		m_size = size;
		m_set.resize((m_size + 63) / 64, 0);
		for (uint32_t i : list)
		{
			m_set[i >> 6] |= (1LL << (i & 63));
		}
	}

	std::string to_string() const
	{
		if (m_size == 0)
			return "";
		std::string str;
		for (size_t i = 0; i < m_set.size(); ++i)
		{
			if (m_set[i] == 0)
				continue;
			int k = (i < m_set.size() - 1) ? 63 : ((m_size - 1) & 63);
			for (int j = 0; j <= k; ++j)
			{
				if (m_set[i] & (1LL << j))
				{
					str += std::to_string(i * 64 + j) + ", ";
				}
			}
		}
		return str;
	}

	std::vector<uint32_t> to_vector() const
	{
		if (m_size == 0)
			return {};
		std::vector<uint32_t> ret;
		for (size_t i = 0; i < m_set.size(); ++i)
		{
			if (m_set[i] == 0)
				continue;
			int k = (i < m_set.size() - 1) ? 63 : ((m_size - 1) & 63);
			for (int j = 0; j <= k; ++j)
			{
				if (m_set[i] & (1LL << j))
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

	void resize(size_t new_size)
	{
		if (new_size == m_size)
		{
			return;
		}

		m_set.resize((new_size + 63) / 64);
		if (new_size < m_size)
		{
			m_size = new_size;
		}
		else
		{
			for (size_t i = (m_size + 63) / 64; i < m_set.size(); ++i)
			{
				m_set[i] = 0;
			}
			if (m_size > 0)
			{
				size_t k = (m_size + 63) / 64 - 1;
				for (int i = m_size & 63; i < 64; ++i)
				{
					m_set[k] &= ~(1LL << i);
				}
			}
			m_size = new_size;
		}
	}


	void clear()
	{
		for (size_t i = 0; i < m_set.size(); ++i)
		{
			m_set[i] = 0;
		}
	}

	bool operator[](size_t i) const
	{
		return m_set[i >> 6] & (1LL << (i & 63)) ? true : false;
	}

	const bit_set& operator=(const bit_set& rhs)
	{
		m_size = rhs.m_size;
		m_set = rhs.m_set;
		return *this;
	}

	bool get(size_t i) const
	{
		return m_set[i >> 6] & (1LL << (i & 63)) ? true : false;
	}

	void set(size_t i, bool b)
	{
		if (b)
		{
			m_set[i >> 6] |= (1LL << (i & 63));
		}
		else
		{
			m_set[i >> 6] &= ~(1LL << (i & 63));
		}
	}

	void insert(size_t i)
	{
		m_set[i >> 6] |= (1LL << (i & 63));
	}

	void erase(size_t i)
	{
		m_set[i >> 6] &= ~(1LL << (i & 63));
	}

	bool get_safe(size_t i) const
	{
		if (0 <= i && i < m_size)
		{
			return get(i);
		}
		return false;
	}

	void set_safe(size_t i, bool b)
	{
		if (0 <= i && i < m_size)
		{
			set(i, b);
		}
	}

	bit_set operator~() const
	{
		return set_complement();
	}

	bit_set operator-() const
	{
		return set_complement();
	}

	bit_set operator+(const bit_set& rhs) const
	{
		bit_set ret(std::max(m_set.size(), rhs.m_set.size()));
		for (size_t i = 0; i < ret.m_set.size(); ++i)
		{
			ret.m_set[i] = (i < m_set.size() ? m_set[i] : 0) | (i < rhs.m_set.size() ? rhs.m_set[i] : 0);
		}
		return ret;
	}

	void operator+=(const bit_set& rhs)
	{
		for (size_t i = 0; i < m_set.size(); ++i)
		{
			m_set[i] |= (i < rhs.m_set.size() ? rhs.m_set[i] : 0);
		}
	}

	bit_set operator-(const bit_set& rhs) const
	{
		bit_set ret(std::max(m_size, rhs.m_size));
		for (size_t i = 0; i < ret.m_set.size(); ++i)
		{
			ret.m_set[i] = (i < m_set.size() ? m_set[i] : 0) & ~(i < rhs.m_set.size() ? rhs.m_set[i] : 0);
		}
		return ret;
	}

	void operator-=(const bit_set& rhs)
	{
		for (size_t i = 0; i < m_set.size(); ++i)
		{
			m_set[i] &= (~(i < rhs.m_set.size() ? rhs.m_set[i] : 0));
		}
	}

	bit_set operator&(const bit_set& rhs) const
	{
		bit_set ret(std::max(m_size, rhs.m_size));
		for (size_t i = 0; i < std::min(m_set.size(), rhs.m_set.size()); ++i)
		{
			ret.m_set[i] = m_set[i] & rhs.m_set[i];
		}
		return ret;
	}

	void operator&=(const bit_set& rhs)
	{
		for (size_t i = 0; i < m_set.size(); ++i)
		{
			m_set[i] &= (i < rhs.m_set.size() ? rhs.m_set[i] : 0);
		}
	}

	bit_set set_complement() const
	{
		bit_set ret(m_size);
		for (size_t i = 0; i < m_set.size(); ++i)
		{
			ret.m_set[i] = ~m_set[i];
		}
		return ret;
	}

private:
	size_t					m_size;
	std::vector<uint64_t>   m_set;
};

inline bit_set set_union(const bit_set& lhs, const bit_set& rhs)
{
	return lhs + rhs;
}

inline bit_set set_difference(const bit_set& lhs, const bit_set& rhs)
{
	return lhs - rhs;
}

inline bit_set set_intersection(const bit_set& lhs, const bit_set& rhs)
{
	return lhs & rhs;
}
