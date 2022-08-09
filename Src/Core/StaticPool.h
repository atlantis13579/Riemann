#pragma once

template<typename T, int SIZE>
class StaticPool
{
public:
	StaticPool() : m_size(0) {}

	T* Get()
	{
		if (m_size >= sizeof(m_pool) / sizeof(m_pool[0]))
		{
			return nullptr;
		}
		return &m_pool[m_size++];
	}

private:
	int	m_size;
	T	m_pool[SIZE];
};
