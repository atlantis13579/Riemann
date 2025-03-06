#pragma once

#include <atomic>
#include "Base.h"

namespace Riemann
{
	template <class T>
	class RefCount
	{
	public:
		RefCount() {}
		RefCount(const RefCount& rhs) {}
		~RefCount()
		{
			int value = mRefCount.load(std::memory_order_relaxed);
			ASSERT(value == 0);
		}

		inline RefCount& operator=(const RefCount&)
		{
			return *this;
		}

		void SetHoldsMemory(bool val)
		{
			mHoldsMemory = val;
		}

		inline int	GetRefcount() const
		{
			return mRefCount.load(std::memory_order_relaxed);
		}

		inline void	AddRefcount()
		{
			mRefCount.fetch_add(1, std::memory_order_relaxed);
		}

		inline void	DecRefcount()
		{
			if (mRefCount.fetch_sub(1, std::memory_order_release) == 1)
			{
				atomic_thread_fence(std::memory_order_acquire);
				if (mHoldsMemory)
				{
					delete static_cast<const T*>(this);
				}
			}
		}

	protected:
		bool						mHoldsMemory = true;
		mutable std::atomic<int>	mRefCount = 0;
	};

	template <class T>
	class Ref
	{
	public:
		Ref() : m_ptr(nullptr) { }
		Ref(T* p) : m_ptr(p) { AddRefcount(); }
		Ref(const Ref<T>& r) : m_ptr(r.m_ptr) { AddRefcount(); }
		Ref(Ref<T>&& r) : m_ptr(r.m_ptr) { r.m_ptr = nullptr; }
		~Ref() { DecRefcount(); }

		inline Ref<T>&		operator = (T* p) { if (m_ptr != p) { DecRefcount(); m_ptr = p; AddRefcount(); } return *this; }
		inline Ref<T>&		operator = (const Ref<T>& r) { if (m_ptr != r.m_ptr) { DecRefcount(); m_ptr = r.m_ptr; AddRefcount(); } return *this; }
		inline Ref<T>&		operator = (Ref<T>&& r) { if (m_ptr != r.m_ptr) { DecRefcount(); m_ptr = r.m_ptr; r.m_ptr = nullptr; } return *this; }

		inline				operator T* () const { return m_ptr; }
		inline T*			operator -> () const { return m_ptr; }
		inline T&			operator * () const { return *m_ptr; }
		inline T*			get() const { return m_ptr; }

		inline bool			operator == (const T* r) const { return m_ptr == r; }
		inline bool			operator == (const Ref<T>& r) const { return m_ptr == r.m_ptr; }
		inline bool			operator != (const T* r) const { return m_ptr != r; }
		inline bool			operator != (const Ref<T>& r) const { return m_ptr != r.m_ptr; }

	private:
		inline void			AddRefcount() { if (m_ptr != nullptr) m_ptr->AddRefcount(); }
		inline void			DecRefcount() { if (m_ptr != nullptr) m_ptr->Release(); }

		T* m_ptr;
	};

	template <class T>
	class ConstRef
	{
	public:
		ConstRef() : m_ptr(nullptr) { }
		ConstRef(const T* r) : m_ptr(r) { AddRefcount(); }
		ConstRef(const ConstRef<T>& r) : m_ptr(r.m_ptr) { AddRefcount(); }
		ConstRef(ConstRef<T>&& r) : m_ptr(r.m_ptr) { r.m_ptr = nullptr; }
		ConstRef(const Ref<T>& r) : m_ptr(r.m_ptr) { AddRefcount(); }
		ConstRef(Ref<T>&& r) : m_ptr(r.m_ptr) { r.m_ptr = nullptr; }
		~ConstRef() { DecRefcount(); }

		inline ConstRef<T>&	operator = (const T* r) { if (m_ptr != r) { DecRefcount(); m_ptr = r; AddRefcount(); } return *this; }
		inline ConstRef<T>& operator = (const ConstRef<T>& r) { if (m_ptr != r.m_ptr) { DecRefcount(); m_ptr = r.m_ptr; AddRefcount(); } return *this; }
		inline ConstRef<T>& operator = (ConstRef<T>&& r) { if (m_ptr != r.m_ptr) { DecRefcount(); m_ptr = r.m_ptr; r.m_ptr = nullptr; } return *this; }
		inline ConstRef<T>& operator = (const Ref<T>& r) { if (m_ptr != r.m_ptr) { DecRefcount(); m_ptr = r.m_ptr; AddRefcount(); } return *this; }
		inline ConstRef<T>& operator = (Ref<T>&& r) { if (m_ptr != r.m_ptr) { DecRefcount(); m_ptr = r.m_ptr; r.m_ptr = nullptr; } return *this; }

		inline				operator const T* () const { return m_ptr; }
		inline const T*		operator -> () const { return m_ptr; }
		inline const T&		operator * () const { return *m_ptr; }
		inline const T*		get() const { return m_ptr; }

		inline bool			operator == (const T* r) const { return m_ptr == r; }
		inline bool			operator == (const ConstRef<T>& r) const { return m_ptr == r.m_ptr; }
		inline bool			operator == (const Ref<T>& r) const { return m_ptr == r.m_ptr; }
		inline bool			operator != (const T* r) const { return m_ptr != r; }
		inline bool			operator != (const ConstRef<T>& r) const { return m_ptr != r.m_ptr; }
		inline bool			operator != (const Ref<T>& r) const { return m_ptr != r.m_ptr; }

	private:
		inline void			AddRefcount() { if (m_ptr != nullptr) m_ptr->AddRefcount(); }
		inline void			DecRefcount() { if (m_ptr != nullptr) m_ptr->DecRefcount(); }

		const T* m_ptr;
	};
}