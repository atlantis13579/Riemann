#pragma once

#include <vector>

template<class TNode, int MaxDepth>
class StaticStack
{
public:
	StaticStack()
	{
		m_Top = 0;
	}

	void Restore(const StaticStack& rhs)
	{
		m_Top = rhs.m_Top;
		memcpy(m_Stack, rhs.m_Stack, sizeof(TNode) * m_Top);
	}

	void Push(TNode p)
	{
		m_Stack[m_Top++] = p;
	}

	TNode Pop()
	{
		return m_Stack[--m_Top];
	}

	TNode Top()
	{
		return m_Stack[m_Top - 1];
	}

	int	Depth() const
	{
		return m_Top;
	}
	
	bool Full() const
	{
		return m_Top >= MaxDepth;
	}

	bool Empty() const
	{
		return m_Top == 0;
	}

	void Clear()
	{
		m_Top = 0;
	}

private:
	int		m_Top;
	TNode	m_Stack[MaxDepth];
};


template<class TNode>
class Stack
{
public:
	Stack()
	{
		m_Top = 0;
		m_Stack.resize(32);
	}

	void Push(TNode* p)
	{
		if (m_Top >= (int)m_Stack.size())
		{
			m_Stack.resize(m_Stack.size() + 32);
		}
		m_Stack[m_Top++] = p;
	}

	TNode* Pop()
	{
		return m_Stack[--m_Top];
	}

	TNode* Top()
	{
		return m_Stack[m_Top - 1];
	}

	int	Depth() const
	{
		return m_Top;
	}

	bool Empty() const
	{
		return m_Top == 0;
	}

private:
	int					m_Top;
	std::vector<TNode*> m_Stack;
};
