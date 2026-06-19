#pragma once

namespace Riemann
{
	class PositionConstraint
	{
	public:
		virtual ~PositionConstraint() {}
		virtual void Setup() = 0;
		virtual void Solve() = 0;
	};
}