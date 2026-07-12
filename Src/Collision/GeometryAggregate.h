#pragma once

namespace Riemann
{
	struct PhysicsContactInfo;
	struct PhysicsTriggerInfo;

	class GeometryAggregate
	{
	public:
		virtual ~GeometryAggregate() = default;
		virtual void OnContact(const PhysicsContactInfo& ContactInfo) { (void)ContactInfo; }
		virtual void OnTrigger(const PhysicsTriggerInfo& TriggerInfo) { (void)TriggerInfo; }
	};
}
