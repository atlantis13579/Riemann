#pragma once

namespace Riemann
{
	template<typename T>
	class List
	{
	public:
		List() : m_root(nullptr), m_count(0) {}

		T* back()
		{
			return m_root;
		}

		int size() const
		{
			return m_count;
		}

		void append(T* p)
		{
			p->prev = nullptr;
			p->next = m_root;
			if (m_root) m_root->prev = p;
			m_root = p;
			++m_count;
		}

		void remove(T* p)
		{
			if (p->next) p->next->prev = p->prev;
			if (p->prev) p->prev->next = p->next;
			if (p == m_root) m_root = p->next;
			--m_count;
		}

	private:
		T* m_root;
		int m_count;
	};


	template<typename T, int Capacity>
	class StaticList
	{
	public:
		StaticList() : m_root(nullptr), m_count(0)
		{
			for (int i = 0; i < Capacity; ++i)
			{
				append(&m_pool[Capacity - i - 1]);
			}
		}

		T* back()
		{
			return m_root;
		}

		int size() const
		{
			return m_count;
		}

		void append(T* p)
		{
			p->prev = nullptr;
			p->next = m_root;
			if (m_root) m_root->prev = p;
			m_root = p;
			++m_count;
		}

		void remove(T* p)
		{
			if (p->next) p->next->prev = p->prev;
			if (p->prev) p->prev->next = p->next;
			if (p == m_root) m_root = p->next;
			--m_count;
		}

		inline bool empty() const
		{
			return m_root == nullptr;
		}

		T* pop()
		{
			if (m_root == nullptr) return nullptr;
			T* p = m_root;
			remove(m_root);
			return p;
		}

	private:
		T*	m_root;
		int m_count;
		T	m_pool[Capacity];
	};
}