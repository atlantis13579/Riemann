#pragma once

namespace Riemann
{
	struct PhysicsContactInfo;

	class GeometryAggregate
	{
	public:
		virtual ~GeometryAggregate() = default;
		virtual void OnContact(const PhysicsContactInfo& ContactInfo) { (void)ContactInfo; }
	};
}
