#pragma once

#include <assert.h>
#include <vector>

namespace Riemann
{
	template<class T, int BlockSize = 8>
	class ListSet
	{
	public:
		struct Header
		{
			int size;
			int first_block;
		};

		struct Block
		{
			T data[BlockSize];
			int next_block;
		};

		class ListProxy
		{
		public:
			ListProxy(ListSet *_owner, int _index)
			{
				m_owner = _owner;
				m_index = _index;
			}

			class ValueIterator
			{
			public:
				inline const T& operator*() const
				{
					return m_curr_block->data[m_curr_block_idx];
				}

				inline T& operator*()
				{
					return m_curr_block->data[m_curr_block_idx];
				}

				inline const ValueIterator& operator++()
				{
					m_index++;
					m_curr_block_idx++;
					if (m_curr_block_idx == BlockSize)
					{
						m_curr_block_idx = 0;
						m_curr_block = m_owner->get_block(m_curr_block->next_block);
					}
					return *this;
				}

				inline bool operator != (const ValueIterator& rhs) const
				{
					return m_index != rhs.m_index;
				}

				inline int key() const
				{
					return m_index;
				}
				
				inline const T& value() const
				{
					return m_curr_block->data[m_curr_block_idx];
				}

			private:
				friend class ListProxy;

				ValueIterator(ListSet *_owner, Block *_curr_block, int _index)
				{
					m_owner = _owner;
					m_index = _index;
					m_curr_block = _curr_block;
					m_curr_block_idx = 0;
				}

			private:
				ListSet		*m_owner { nullptr };
				Block		*m_curr_block { nullptr };
				int			m_curr_block_idx { 0 };
				int			m_index { 0 };
			};

			inline T& operator[](int idx)
			{
				Header *head = m_owner->get_header(m_index);
				if (idx < 0 || idx > head->size)
				{
					assert(false);
					T *data = nullptr;		// out of index, make it crash
					return data[idx];
				}

				Block *block = m_owner->get_block(head->first_block);
				assert(block);
				if (idx < BlockSize)
				{
					return block->data[idx];
				}

				while (idx >= BlockSize)
				{
					idx -= BlockSize;
					block = m_owner->get_block(block->next_block);
					assert(block);
				}
				
				return block->data[idx];
			}

			inline const T& operator[](int idx) const
			{
				const Header* head = m_owner->get_header(m_index);
				if (idx < 0 || idx > head->size)
				{
					assert(false);
					T* data = nullptr;		// out of index, make it crash
					return data[idx];
				}

				const Block* block = m_owner->get_block(head->first_block);
				assert(block);
				if (idx < BlockSize)
				{
					return block->data[idx];
				}

				while (idx >= BlockSize)
				{
					idx -= BlockSize;
					block = m_owner->get_block(block->next_block);
					assert(block);
				}

				return block->data[idx];
			}

			ValueIterator begin() const
			{
				Header* head = m_owner->get_header(m_index);
				Block* block = head ? m_owner->get_block(head->first_block) : nullptr;
				return ValueIterator(m_owner, block, 0);
			}

			ValueIterator end() const
			{
				Header* head = m_owner->get_header(m_index);
				return ValueIterator(m_owner, nullptr, head ? head->size : 0);
			}

			inline int find(const T &val) const
			{
				for (auto it = begin(); it != end(); ++it)
				{
					if (*it == val)
					{
						return it.key();
					}
				}

				return -1;
			}

			inline void push_back(const T &val)
			{
				Header* head = m_owner->get_header(m_index);
				Block* block = m_owner->get_block(head->first_block);
				if (block == nullptr)
				{
					head->first_block = m_owner->allocate(nullptr);
					block = m_owner->get_block(head->first_block);
				}
				assert(block);

				if (head->size < BlockSize)
				{
					block->data[head->size++] = val;
					return;
				}
				
				int idx = head->size;
				Block *prev_block = block;
				while (idx >= BlockSize)
				{
					idx -= BlockSize;
					assert(block);
					prev_block = block;
					block = m_owner->get_block(block->next_block);
				}

				if (block == nullptr)
				{
					assert(prev_block);
					block = m_owner->get_block(m_owner->allocate(prev_block));
				}

				block->data[idx] = val;
				head->size++;
			}

			int remove(const T& val)
			{
				for (auto it = begin(); it != end(); ++it)
				{
					if (*it == val)
					{
						int index = it.key();
						remove_at(index, false);
						return index;
					}
				}

				return -1;
			}

			bool remove_at(int idx, bool preserve_order = true)
			{
				Header* head = m_owner->get_header(m_index);
				if (idx < 0 || idx >= head->size)
				{
					return false;
				}

				Block* block = m_owner->get_block(head->first_block);
				assert(block);

				if (head->size <= BlockSize)
				{
					if (preserve_order)
					{
						for (int i = idx; i < head->size - 1; ++i)
						{
							block->data[i] = block->data[i + 1];
						}
					}
					else
					{
						block->data[idx] = block->data[head->size - 1];
					}
					head->size--;
					return true;
				}

				int move_num = head->size - idx - 1;

				while (idx >= BlockSize)
				{
					idx -= BlockSize;
					assert(block);
					block = m_owner->get_block(block->next_block);
				}

				if (preserve_order)
				{
					for (int i = 0; i < move_num; ++i)
					{
						if (idx < BlockSize - 1)
						{
							block->data[idx] = block->data[idx + 1];
						}
						else
						{
							Block *next_block = m_owner->get_block(block->next_block);
							assert (next_block);
							block->data[idx] = next_block->data[0];

							if (i == move_num - 1)
							{
								m_owner->release(block->next_block);
								block->next_block = -1;
								break;
							}

							block = next_block;
						}
						idx = (idx + 1) % BlockSize;
					}
				}
				else
				{
					Block *block2 = block;
					while (block2->next_block >= 0)
					{
						block2 = m_owner->get_block(block2->next_block);
					}

					int idx2 = (head->size - 1) % BlockSize;
					block->data[idx] = block2->data[idx2];
				}

				head->size--;

				return true;
			}

			template<typename TFunction>
			void all_iteration(TFunction func)
			{
				Header* head = m_owner->get_header(m_index);
				if (head->size == 0)
				{
					return;
				}

				Block* curr_block = m_owner->get_block(head->first_block);
				for (int i = 0, j = 0; i < head->size; ++i, ++j)
				{
					if (j == BlockSize)
					{
						j = 0;
						curr_block = m_owner->get_block(curr_block->next_block);
					}

					func(curr_block->data[j]);
				}
			}

			void operator=(const std::initializer_list<T>& rhs)
			{
				clear();
				for (const T& i : rhs)
				{
					push_back(i);
				}
			}

			void from_vector(const std::vector<T>& rhs)
			{
				clear();
				for (const T& i : rhs)
				{
					push_back(i);
				}
			}

			std::vector<T> to_vector() const
			{
				std::vector<T> v;

				for (auto it = begin(); it != end(); ++it)
				{
					v.push_back(*it);
				}

				return v;
			}

			int size() const
			{
				Header* head = m_owner->get_header(m_index);
				return head ? head->size : 0;
			}

			bool empty() const
			{
				return size() == 0;
			}

			void clear()
			{
				Header* head = m_owner->get_header(m_index);
				Block* block = m_owner->get_block(head->first_block);
				while (block)
				{
					int next = block->next_block;
					block = m_owner->get_block(next);
					m_owner->release(next);
				}
				m_owner->release(head->first_block);
				head->size = 0;
				head->first_block = -1;
			}

		private:
			ListSet *m_owner;
			int		m_index;
		};

		class ValueEnumerable
		{
		public:
			ValueEnumerable(const ListSet* _owner, int _index) : m_owner(_owner)
			{
				m_index = _index;
			}

			class Iterator
			{
			public:
				inline const T& operator*() const
				{
					return m_curr_block->data[m_curr_block_idx];
				}

				inline const Iterator& operator++()
				{
					m_index++;
					m_curr_block_idx++;
					if (m_curr_block_idx == BlockSize)
					{
						m_curr_block_idx = 0;
						m_curr_block = m_owner->get_block(m_curr_block->next_block);
					}
					return *this;
				}

				inline bool operator != (const Iterator& rhs) const
				{
					return m_index != rhs.m_index;
				}

				inline int key() const
				{
					return m_index;
				}

				inline const T& value() const
				{
					return m_curr_block->data[m_curr_block_idx];
				}

			private:
				friend class ValueEnumerable;

				Iterator(const ListSet* _owner, const Block* _curr_block, int _index) : m_owner(_owner), m_curr_block(_curr_block)
				{
					m_index = _index;
					m_curr_block_idx = 0;
				}

			private:
				const Block*	m_curr_block{ nullptr };
				int				m_curr_block_idx{ 0 };
				int				m_index{ 0 };

				const ListSet* m_owner{ nullptr };
			};

			inline const T& operator[](int idx) const
			{
				const Header* head = m_owner->get_header(m_index);
				if (idx < 0 || idx > head->size)
				{
					assert(false);
					T* data = nullptr;		// out of index, make it crash
					return data[idx];
				}

				const Block* block = m_owner->get_block(head->first_block);
				assert(block);
				if (idx < BlockSize)
				{
					return block->data[idx];
				}

				while (idx >= BlockSize)
				{
					idx -= BlockSize;
					block = m_owner->get_block(block->next_block);
					assert(block);
				}

				return block->data[idx];
			}

			int size() const
			{
				const Header* head = m_owner->get_header(m_index);
				return head->size;
			}

			bool empty() const
			{
				return size() == 0;
			}

			template<typename TFunction>
			void all_iteration(TFunction func)
			{
				const Header* head = m_owner->get_header(m_index);
				if (head->size == 0)
				{
					return;
				}

				const Block* curr_block = m_owner->get_block(head->first_block);
				for (int i = 0, j = 0; i < head->size; ++i, ++j)
				{
					if (j == BlockSize)
					{
						j = 0;
						curr_block = m_owner->get_block(curr_block->next_block);
					}

					func(curr_block->data[j]);
				}
			}

			Iterator begin() const
			{
				const Header* head = m_owner->get_header(m_index);
				const Block* block = head ? m_owner->get_block(head->first_block) : nullptr;
				return Iterator(m_owner, block, 0);
			}

			Iterator end() const
			{
				const Header* head = m_owner->get_header(m_index);
				return Iterator(m_owner, nullptr, head ? head->size : 0);
			}

			inline int find(const T& val) const
			{
				for (Iterator it = begin(); it != end(); ++it)
				{
					if (*it == val)
					{
						return it.key();
					}
				}

				return -1;
			}

			std::vector<T> to_vector() const
			{
				std::vector<T> v;

				for (auto it = begin(); it != end(); ++it)
				{
					v.push_back(*it);
				}

				return v;
			}

		private:
			const	ListSet* m_owner;
			int		m_index;
		};

		class ListIterator
		{
		public:
			inline ListProxy operator*() const
			{
				return ListProxy(m_owner, m_index);
			}

			inline const ListIterator& operator++()
			{
				m_index++;
				return *this;
			}

			inline bool operator != (const ListIterator& rhs) const
			{
				return m_index != rhs.m_index;
			}

		private:
			friend class ListSet;

			ListIterator(ListSet* _owner, int _index) : m_owner(_owner), m_index(_index)
			{
			}

		private:
			int		m_index{ 0 };
			ListSet* m_owner{ nullptr };
		};

		ListSet()
		{
		}

		ListSet(int size)
		{
			resize(size);
		}

		static int GetBlockSize()
		{
			return BlockSize;
		}

		void resize(int new_size)
		{
			int old_size = (int)m_headers.size();
			m_headers.resize(new_size);
			if (new_size > old_size)
			{
				for (int i = old_size; i < new_size; ++i)
				{
					m_headers[i].size = 0;
					m_headers[i].first_block = -1;
				}
			}
		}

		int append()
		{
			int s = size();
			resize(s + 1);
			return s;
		}

		void clear()
		{
			m_headers.clear();
			m_blocks.clear();
			m_freelist.clear();
		}

		inline int size() const
		{
			return (int)m_headers.size();
		}

		inline bool empty() const
		{
			return m_headers.empty();
		}

		inline int max_elements() const
		{
			int max_val = 0;
			for (size_t i = 0; i < m_headers.size(); ++i)
			{
				int val = m_headers[i].size;
				if (val > max_val)
				{
					max_val = val;
				}
			}
			return max_val;
		}

		inline ListProxy operator[](int idx)
		{
			return ListProxy(this, idx);
		}

		inline ValueEnumerable operator[](int idx) const
		{
			return ValueEnumerable(this, idx);
		}

		ListIterator begin()
		{
			return ListIterator(this, 0);
		}

		ListIterator end()
		{
			return ListIterator(this, (int)m_headers.size());
		}

	private:
		int allocate(Block *prev_block)
		{
			int free_block;
			if (m_freelist.size() > 0)
			{
				free_block = m_freelist.back();
				if (prev_block)
				{
					prev_block->next_block = free_block;
				}
				m_freelist.pop_back();
			}
			else
			{
				free_block = (int)m_blocks.size();
				int new_size = free_block + 1;
				if (prev_block)
				{
					prev_block->next_block = free_block;		// do this before resize
				}
				m_blocks.resize(new_size);
			}
			m_blocks[free_block].next_block = -1;

			return free_block;
		}

		void release(int block)
		{
			if (block >= 0)
			{
				assert(block < (int)m_blocks.size());
				m_freelist.push_back(block);
			}
		}

		inline Header* get_header(int idx)
		{
			return idx < (int)m_headers.size() ? &m_headers[idx] : nullptr;
		}

		inline const Header* get_header(int idx) const
		{
			return idx < (int)m_headers.size() ? &m_headers[idx] : nullptr;
		}

		inline Block* get_block(int idx)
		{
			if (idx < 0)
			{
				return nullptr;
			}
			return &m_blocks[idx];
		}

		inline const Block* get_block(int idx) const
		{
			if (idx < 0)
			{
				return nullptr;
			}
			return &m_blocks[idx];
		}

	private:
		std::vector<Header>		m_headers;
		std::vector<Block>		m_blocks;
		std::vector<int>		m_freelist;
	};
}