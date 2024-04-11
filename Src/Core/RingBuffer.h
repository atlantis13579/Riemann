#pragma once

#include "Lock.h"

namespace Riemann
{
	template <typename T, int Capacity>
	class RingBuffer
	{
	public:
		RingBuffer()
		{
			clear();
		}

		inline bool full() const
		{
			int next = (m_write + 1) % Capacity;
			return next == m_read;
		}

		inline bool clear() const
		{
			m_read = m_write = 0;
		}

		inline bool empty() const
		{
			return m_read == m_write;
		}

		inline bool push(const T& v)
		{
			int next = (m_write + 1) % Capacity;
			if (next != m_read)
			{
				m_data[m_write] = v;
				m_write = next;
				return true;
			}
			return false;
		}

		inline T pop()
		{
			if (m_read != m_write)
			{
				T item = m_data[m_read];
				m_read = (m_read + 1) % Capacity;
				return item;
			}
			return T();
		}

	private:
		T			m_data[Capacity];
		int			m_write;
		int			m_read;
	};

	template <typename T, int Capacity>
	class ThreadSafeRingBuffer
	{
	public:
		ThreadSafeRingBuffer()
		{
			clear();
		}

		void	clear()
		{
			m_lock.lock();
			m_write = m_read = 0;
			m_lock.unlock();
		}

		int		size()
		{
			m_lock.lock();
			int n = (m_write + Capacity - m_read) % Capacity;
			m_lock.unlock();
			return n;
		}

		bool	push(const T& v)
		{
			bool succ = false;
			m_lock.lock();
			int next = (m_write + 1) % Capacity;
			if (next != m_read)
			{
				m_data[m_write] = v;
				m_write = next;
				succ = true;
			}
			m_lock.unlock();
			return succ;
		}

		T		pop()
		{
			T ret = T(0);
			m_lock.lock();
			if (m_read != m_write)
			{
				ret = m_data[m_read];
				m_read = (m_read + 1) % Capacity;
			}
			m_lock.unlock();
			return ret;
		}

	private:
		T			m_data[Capacity];
		int			m_write;
		int			m_read;
		SpinLock	m_lock;
	};

	template <typename T, int Capacity>
	class LockFreeRingBuffer
	{
	};
}