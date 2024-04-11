#pragma once

#include "Lock.h"

namespace Riemann
{
	template<typename T, int Capacity>
	class StaticPool
	{
	public:
		StaticPool() : m_size(0) {}

		T* get()
		{
			if (m_size >= Capacity)
			{
				return nullptr;
			}
			return &m_data[m_size++];
		}

	private:
		T	m_data[Capacity];
		int	m_size;
	};

	template<typename T, int Capacity>
	class ThreadSafeStaticPool
	{
	public:
		ThreadSafeStaticPool() : m_size(0) {}

		T* get()
		{
			T* p = nullptr;
			m_lock.lock();
			if (m_size < Capacity)
			{
				p = &m_data[m_size++];
			}
			m_lock.unlock();
			return p;
		}

	private:
		T			m_data[Capacity];
		int			m_size;
		SpinLock	m_lock;
	};
}