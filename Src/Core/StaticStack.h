#pragma once

#include <vector>

namespace Riemann
{
	template<class T, int MaxDepth>
	class StaticStack
	{
	public:
		StaticStack()
		{
			m_top = 0;
		}

		void restore(const StaticStack& rhs)
		{
			m_top = rhs.m_top;
			memcpy(m_stack, rhs.m_stack, sizeof(T) * m_top);
		}

		void push(T p)
		{
			m_stack[m_top++] = p;
		}

		T pop()
		{
			return m_stack[--m_top];
		}

		T top()
		{
			return m_stack[m_top - 1];
		}

		int	depth() const
		{
			return m_top;
		}
	
		bool full() const
		{
			return m_top >= MaxDepth;
		}

		int size() const
		{
			return m_top;
		}

		bool empty() const
		{
			return m_top == 0;
		}

		void clear()
		{
			m_top = 0;
		}

		class Iterator
		{
		public:
			inline const T& operator*() const
			{
				return m_owner->m_stack[m_index];
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
			friend class StaticStack;

			Iterator(const StaticStack* _owner, const int _index) : m_owner(_owner), m_index(_index)
			{
			}

		public:
			const StaticStack*	m_owner{ nullptr };
			int					m_index{ 0 };
		};

		Iterator begin()
		{
			return Iterator(this, 0);
		}

		Iterator end()
		{
			return Iterator(this, m_top);
		}

	private:
		int	m_top;
		T	m_stack[MaxDepth];
	};


	template<class T>
	class Stack
	{
	public:
		Stack()
		{
			m_top = 0;
			m_stack.resize(32);
		}

		void push(T* p)
		{
			if (m_top >= (int)m_stack.size())
			{
				m_stack.resize(m_stack.size() + 32);
			}
			m_stack[m_top++] = p;
		}

		T* pop()
		{
			return m_stack[--m_top];
		}

		T* top()
		{
			return m_stack[m_top - 1];
		}

		int	depth() const
		{
			return m_top;
		}

		int size() const
		{
			return m_top;
		}

		bool empty() const
		{
			return m_top == 0;
		}

		class Iterator
		{
		public:
			inline const T& operator*() const
			{
				return m_owner->m_stack[m_index];
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
			friend class Stack;

			Iterator(const Stack* _owner, const int _index) : m_owner(_owner), m_index(_index)
			{
			}

		public:
			const Stack*	m_owner{ nullptr };
			int				m_index{ 0 };
		};

		Iterator begin()
		{
			return Iterator(this, 0);
		}

		Iterator end()
		{
			return Iterator(this, m_top);
		}

	private:
		int				m_top;
		std::vector<T*> m_stack;
	};
}