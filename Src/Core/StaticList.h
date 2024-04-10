#pragma once

namespace Riemann
{
	template<typename T>
	class List
	{
	public:
		T* root;
		int count;
		List() : root(nullptr), count(0) {}

		void Append(T* p)
		{
			p->prev = nullptr;
			p->next = root;
			if (root) root->prev = p;
			root = p;
			++count;
		}

		void Remove(T* p)
		{
			if (p->next) p->next->prev = p->prev;
			if (p->prev) p->prev->next = p->next;
			if (p == root) root = p->next;
			--count;
		}
	};


	template<typename T, int Capacity>
	class StaticList
	{
	public:
		StaticList() : root(nullptr), count(0)
		{
			for (int i = 0; i < Capacity; ++i)
			{
				Append(&m_pool[Capacity - i - 1]);
			}
		}

		void Append(T* p)
		{
			p->prev = nullptr;
			p->next = root;
			if (root) root->prev = p;
			root = p;
			++count;
		}

		void Remove(T* p)
		{
			if (p->next) p->next->prev = p->prev;
			if (p->prev) p->prev->next = p->next;
			if (p == root) root = p->next;
			--count;
		}

		inline bool Empty() const
		{
			return root == nullptr;
		}

		T* Pop()
		{
			if (root == nullptr) return nullptr;
			T* p = root;
			Remove(root);
			return p;
		}

	private:
		T*	root;
		int count;
		T	m_pool[Capacity];
	};
}