#pragma once

#include "../Core/ListSet.h"

namespace Riemann
{
	struct Bond
	{
		enum class BondType
		{
			Default = 0,
			Connect = 1,
			Supports = 2,
			Supported = 3,
		};

		int				v0;
		int				v1;
		BondType		type{ BondType::Default };
		float			strength{ 0.0f };

		inline bool operator< (const Bond& rhs)
		{
			return (v0 < rhs.v0) || (v0 == rhs.v0 && v1 < rhs.v1);
		}
	};

	class ConnectionGraph
	{
	public:
		const ListSet<Bond>&	GetBonds() const { return m_bonds; }

	private:
		ListSet<Bond>	m_bonds;
	};
}	// namespace Riemann
