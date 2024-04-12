#pragma once

template<typename TContainer, typename TValue>
class TIndexBasedEnumerable
{
public:
	class Iterator
	{
	public:
		inline TValue operator*() const
		{
			return m_container->GetValue(m_index);
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
		friend class TIndexBasedEnumerable;

		Iterator(TContainer* _container, int _index) : m_container(_container), m_index(_index)
		{
		}

	private:
		TContainer* m_container{ nullptr };
		int			m_index{ 0 };
	};

	Iterator begin()
	{
		return Iterator(m_container, 0);
	}

	Iterator end()
	{
		return Iterator(m_container, (int)m_container->size());
	}

	TIndexBasedEnumerable(TContainer *_container) : m_container(_container)

private:
	TContainer* m_container{ nullptr };
};