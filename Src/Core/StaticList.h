#pragma once

namespace Riemann
{
	template<typename T>
	class List
	{
	public:
		List() : m_root(nullptr), m_size(0) {}

		T* back()
		{
			return m_root;
		}

		int size() const
		{
			return m_size;
		}

		bool empty() const
		{
			return m_root == nullptr;
		}

		void append(T* p)
		{
			p->prev = nullptr;
			p->next = m_root;
			if (m_root) m_root->prev = p;
			m_root = p;
			++m_size;
		}

		void remove(T* p)
		{
			if (p->next) p->next->prev = p->prev;
			if (p->prev) p->prev->next = p->next;
			if (p == m_root) m_root = p->next;
			--m_size;
		}

		class Iterator
		{
		public:
			inline const T& operator*() const
			{
				return *m_item;
			}

			inline const Iterator& operator++()
			{
				m_item = m_item->next;
				return *this;
			}

			inline bool operator != (const Iterator& rhs) const
			{
				return m_item != rhs.m_item;
			}

		private:
			explicit Iterator(T* item) : m_item(item)
			{
			}

		public:
			const T* m_item{ nullptr };
		};

		Iterator begin()
		{
			return Iterator(m_root);
		}

		Iterator end()
		{
			return Iterator(nullptr);
		}

	private:
		T* m_root;
		int m_size;
	};


	template<typename T, int Capacity>
	class StaticList
	{
	public:
		StaticList() : m_root(nullptr), m_size(0)
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
			return m_size;
		}

		void append(T* p)
		{
			p->prev = nullptr;
			p->next = m_root;
			if (m_root) m_root->prev = p;
			m_root = p;
			++m_size;
		}

		void remove(T* p)
		{
			if (p->next) p->next->prev = p->prev;
			if (p->prev) p->prev->next = p->next;
			if (p == m_root) m_root = p->next;
			--m_size;
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

		class Iterator
		{
		public:
			inline const T& operator*() const
			{
				return *m_item;
			}

			inline const Iterator& operator++()
			{
				m_item = m_item->next;
				return *this;
			}

			inline bool operator != (const Iterator& rhs) const
			{
				return m_item != rhs.m_item;
			}

		private:
			explicit Iterator(T* item) : m_item(item)
			{
			}

		public:
			const T* m_item{ nullptr };
		};

		Iterator begin()
		{
			return Iterator(m_root);
		}

		Iterator end()
		{
			return Iterator(nullptr);
		}

	private:
		T*	m_root;
		int m_size;
		T	m_pool[Capacity];
	};
}