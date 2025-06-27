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
			return m_size == Capacity;
		}

		inline void clear()
		{
			m_head = 0;
			m_size = 0;
		}

		inline bool empty() const
		{
			return m_size == 0;
		}

		inline size_t size() const
		{
			return (size_t)m_size;
		}

		inline bool push(const T& v, const bool replace_on_full)
		{
			if (m_size < Capacity)
			{
				m_data[(m_head + m_size) % Capacity] = v;
				m_size++;
				return true;
			}
			if (replace_on_full)
			{
				m_data[(m_head + m_size) % Capacity] = v;
				m_head = (m_head + 1) % Capacity;
				return true;
			}
			return false;
		}

		inline T pop()
		{
			if (m_size > 0)
			{
				T item = m_data[(m_head + m_size) % Capacity];
				m_head = (m_head + 1) % Capacity;
				m_size--;
				return item;
			}
			return T();
		}

		class Iterator
		{
		public:
			inline const T& operator*() const
			{
				return m_owner->m_data[(m_owner->m_head + m_index) % Capacity];
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
			friend class RingBuffer;

			Iterator(RingBuffer* _owner, int _index) : m_owner(_owner), m_index(_index)
			{
			}

		public:
			const RingBuffer*	m_owner{ nullptr };
			int					m_index { 0 };
		};

		Iterator begin()
		{
			return Iterator(this, 0);
		}

		Iterator end()
		{
			return Iterator(this, m_size);
		}

	private:
		T			m_data[Capacity];
		int			m_head = 0;
		int			m_size = 0;
	};

	template <typename T, int Capacity>
	class ThreadSafeRingBuffer
	{
	public:
		ThreadSafeRingBuffer()
		{
			clear();
		}

		inline bool full() const
		{
			return m_size == Capacity;
		}

		inline void clear()
		{
			m_head = 0;
			m_size = 0;
		}

		inline bool empty() const
		{
			return m_size == 0;
		}

		inline size_t size() const
		{
			return (size_t)m_size;
		}

		inline bool push(const T& v, bool replace_on_full)
		{
			lock();
			bool succ = false;
			if (m_size < Capacity)
			{
				m_data[(m_head + m_size) % Capacity] = v;
				m_size++;
				succ = true;
			}
			else if (replace_on_full)
			{
				m_data[(m_head + m_size) % Capacity] = v;
				m_head = (m_head + 1) % Capacity;
				succ = true;
			}
			unlock();
			return succ;
		}

		inline T pop()
		{
			T item = T(0);
			lock();
			if (m_size > 0)
			{
				item = m_data[(m_head + m_size) % Capacity];
				m_head = (m_head + 1) % Capacity;
				m_size--;
			}
			unlock();
			return item;
		}

		class Iterator
		{
		public:
			inline const T& operator*() const
			{
				return m_owner->m_data[(m_owner->m_head + m_index) % Capacity];
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
			friend class ThreadSafeRingBuffer;

			Iterator(ThreadSafeRingBuffer* _owner, int _index) : m_owner(_owner), m_index(_index)
			{
			}

		public:
			const ThreadSafeRingBuffer*	m_owner{ nullptr };
			int							m_index{ 0 };
		};

		Iterator begin()
		{
			return Iterator(this, 0);
		}

		Iterator end()
		{
			return Iterator(this, m_size);
		}

		inline void lock()
		{
			m_lock.lock();
		}

		inline void unlock()
		{
			m_lock.unlock();
		}

	private:
		T			m_data[Capacity];
		int			m_head = 0;
		int			m_size = 0;
		SpinLock	m_lock;
	};
}