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
			m_Stack.resize(32);
		}

		void push(T* p)
		{
			if (m_top >= (int)m_Stack.size())
			{
				m_Stack.resize(m_Stack.size() + 32);
			}
			m_Stack[m_top++] = p;
		}

		T* pop()
		{
			return m_Stack[--m_top];
		}

		T* top()
		{
			return m_Stack[m_top - 1];
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

	private:
		int				m_top;
		std::vector<T*> m_Stack;
	};
}