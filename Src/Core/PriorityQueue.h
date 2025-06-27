#pragma once

#include <vector>

namespace Riemann
{
	template <typename T>
	class PriorityPool
	{
	public:
		explicit PriorityPool(int n = 128)
		{
			m_size = 0;
			m_heap.resize(n);
		}

		~PriorityPool()
		{
		}

		inline PriorityPool& operator=(PriorityPool& rhs)
		{
			m_size = rhs.m_size;
			m_heap = rhs.m_heap;
			return *this;
		}

		int size() const
		{
			return m_size;
		}

		inline void clear()
		{
			m_size = 0;
		}

		inline T* top()
		{
			if (m_size == 0)
			{
				return nullptr;
			}
			return m_heap[0];
		}

		inline T* pop()
		{
			if (m_size == 0)
			{
				return nullptr;
			}
			T* result = m_heap[0];
			--m_size;
			trickleDown(0, m_heap[m_size]);
			return result;
		}

		inline void push(T* node)
		{
			if (m_size + 1 >= m_heap.size())
			{
				int expansion_size = m_size;
				if (expansion_size > 4096)
					expansion_size = 4096;
				else if (expansion_size < 128)
					expansion_size = 128;
				m_heap.resize(m_heap.size() + expansion_size);
			}
			++m_size;
			bubbleUp(m_size - 1, node);
		}

		inline void update(T* node)
		{
			for (int i = 0; i < m_size; ++i)
			{
				if (m_heap[i] == node)
				{
					modify(i, node);
					return;
				}
			}
		}

		inline void remove(T* node)
		{
			for (int i = 0; i < m_size; ++i)
			{
				if (m_heap[i] == node)
				{
					m_heap[i] = m_heap[m_size - 1];
					m_size--;

					modify(i, m_heap[i]);
					return;
				}
			}
		}

		inline bool empty() const
		{
			return m_size == 0;
		}

		inline int capacity() const
		{
			return (int)m_heap.size() - 1;
		}

	private:
		void modify(int i, T* node)
		{
			int parent = (i - 1) / 2;
			if (node < m_heap[parent])
			{
				bubbleUp(i, node);
			}
			else
			{
				trickleDown(i, node);
			}
		}

		void bubbleUp(int i, T* node)
		{
			int parent = (i - 1) / 2;
			while ((i > 0) && (*node < *m_heap[parent]))
			{
				m_heap[i] = m_heap[parent];
				i = parent;
				parent = (i - 1) / 2;
			}
			m_heap[i] = node;
		}

		void trickleDown(int i, T* node)
		{
			int child = (i * 2) + 1;
			while (child < m_size)
			{
				if (((child + 1) < m_size) && (*m_heap[child + 1] < *m_heap[child]))
				{
					++child;
				}
				m_heap[i] = m_heap[child];
				i = child;
				child = (i * 2) + 1;
			}
			bubbleUp(i, node);
		}

		std::vector<T*> m_heap;
		int				m_size;
	};

	template <typename T>
	class PriorityQueue
	{
	public:
		explicit PriorityQueue(int n = 128)
		{
			m_size = 0;
			m_heap.resize(n);
		}

		~PriorityQueue()
		{
		}

		inline PriorityQueue& operator=(PriorityQueue& rhs)
		{
			m_size = rhs.m_size;
			m_heap = rhs.m_heap;
			return *this;
		}

		int size() const
		{
			return m_size;
		}

		inline void clear()
		{
			m_size = 0;
		}

		inline T top()
		{
			return m_heap[0];
		}

		inline T pop()
		{
			T result = m_heap[0];
			--m_size;
			trickleDown(0, m_heap[m_size]);
			return result;
		}

		inline void push(const T& node)
		{
			if (m_size + 1 >= m_heap.size())
			{
				int expansion_size = m_size;
				if (expansion_size > 4096)
					expansion_size = 4096;
				else if (expansion_size < 128)
					expansion_size = 128;
				m_heap.resize(m_heap.size() + expansion_size);
			}
			++m_size;
			bubbleUp(m_size - 1, node);
		}

		inline void update(const T& node)
		{
			for (int i = 0; i < m_size; ++i)
			{
				if (m_heap[i] == node)
				{
					modify(i, node);
					return;
				}
			}
		}

		inline void remove(const T& node)
		{
			for (int i = 0; i < m_size; ++i)
			{
				if (m_heap[i] == node)
				{
					m_heap[i] = m_heap[m_size-1];
					m_size--;

					modify(i, m_heap[i]);
					return;
				}
			}
		}

		inline bool empty() const
		{
			return m_size == 0;
		}

		inline int capacity() const
		{
			return (int)m_heap.size() - 1;
		}

	private:
		void modify(int i, const T& node)
		{
			int parent = (i - 1) / 2;
			if (node < m_heap[parent])
			{
				bubbleUp(i, node);
			}
			else
			{
				trickleDown(i, node);
			}
		}

		void bubbleUp(int i, const T& node)
		{
			int parent = (i - 1) / 2;
			while ((i > 0) && (node < m_heap[parent]))
			{
				m_heap[i] = m_heap[parent];
				i = parent;
				parent = (i - 1) / 2;
			}
			m_heap[i] = node;
		}

		void trickleDown(int i, const T& node)
		{
			int child = (i * 2) + 1;
			while (child < m_size)
			{
				if (((child + 1) < m_size) && (m_heap[child + 1] < m_heap[child]))
				{
					++child;
				}
				m_heap[i] = m_heap[child];
				i = child;
				child = (i * 2) + 1;
			}
			bubbleUp(i, node);
		}

		std::vector<T>	m_heap;
		int				m_size;
	};
}