#include <assert.h>
#include <string>
#include <vector>

class bit_set
{
public:
	explicit bit_set(size_t size)
	{
		m_size = size;
		m_set.resize((m_size + 63) / 64, 0);
	}

	bit_set(const bit_set& rhs)
	{
		m_size = rhs.m_size;
		m_set = rhs.m_set;
	}

	bit_set(const std::initializer_list<int>& list)
	{
		m_size = 0;
		for (int i : list)
		{
			if (i > m_size)
				m_size = i;
		};
		m_size = m_size + 1;
		m_set.resize((m_size + 63) / 64, 0);
		for (int i : list)
		{
			m_set[i >> 8] |= (1i64 << (i & 63));
		}
	}

	std::string debug_str() const
	{
		std::string str;
		for (size_t i = 0; i < m_set.size(); ++i)
		{
			if (m_set[i] == 0)
				continue;
			int k = (i < m_set.size() - 1) ? 64 : (m_size & 63);
			for (int j = 0; j < k; ++j)
			{
				if (m_set[i] & (1i64 << j))
				{
					str += std::to_string(i * 64 + j) + ", ";
				}
			}
		}
		return str;
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
			size_t k = (m_size + 63) / 64 - 1;
			for (int i = m_size & 63; i < 64; ++i)
			{
				m_set[k] &= ~(1i64 << i);
			}
			m_size = new_size;
		}
	}

	bool operator()(int i) const
	{
		assert(0 <= i && i < m_size);
		return m_set[i >> 8] & (1i64 << (i & 63)) ? true : false;
	}

	const bit_set& operator=(const bit_set& rhs)
	{
		m_size = rhs.m_size;
		m_set = rhs.m_set;
		return *this;
	}

	bool get(int i) const
	{
		assert(0 <= i && i < m_size);
		return m_set[i >> 8] & (1i64 << (i & 63)) ? true : false;
	}

	void set(int i, bool b)
	{
		assert(0 <= i && i < m_size);
		if (b)
		{
			m_set[i >> 8] |= (1i64 << (i & 63));
		}
		else
		{
			m_set[i >> 8] &= ~(1i64 << (i & 63));
		}
	}

	bit_set operator~() const
	{
		bit_set ret(m_size);
		for (size_t i = 0; i < m_set.size(); ++i)
		{
			ret.m_set[i] = ~m_set[i];
		}
		return ret;
	}

	bit_set operator-() const
	{
		return operator~();
	}

	bit_set operator+(const bit_set& rhs) const
	{
		bit_set ret(std::max(m_set.size(), rhs.m_set.size()));
		for (size_t i = 0; i < std::min(m_set.size(), rhs.m_set.size()); ++i)
		{
			ret.m_set[i] = m_set[i] | rhs.m_set[i];
		}
		return ret;
	}

	bit_set operator-(const bit_set& rhs) const
	{
		bit_set ret(std::max(m_size, rhs.m_size));
		for (size_t i = 0; i < std::min(m_set.size(), rhs.m_set.size()); ++i)
		{
			ret.m_set[i] = m_set[i] & (~rhs.m_set[i]);
		}
		return ret;
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

	const bit_set& operator+=(const bit_set& rhs)
	{
		if (m_size < rhs.m_size)
		{
			resize(rhs.m_size);
		}
		for (size_t i = 0; i < rhs.m_set.size(); ++i)
		{
			m_set[i] |= rhs.m_set[i];
		}
		return *this;
	}

	const bit_set& operator-=(const bit_set& rhs)
	{
		if (m_size < rhs.m_size)
		{
			resize(rhs.m_size);
		}
		for (size_t i = 0; i < rhs.m_set.size(); ++i)
		{
			m_set[i] &= (~rhs.m_set[i]);
		}
		return *this;
	}

	const bit_set& operator&=(const bit_set& rhs)
	{
		if (m_size < rhs.m_size)
		{
			resize(rhs.m_size);
		}
		for (size_t i = 0; i < rhs.m_set.size(); ++i)
		{
			m_set[i] &= rhs.m_set[i];
		}
		return *this;
	}

private:
	size_t					m_size;
	std::vector<uint64_t>   m_set;
};