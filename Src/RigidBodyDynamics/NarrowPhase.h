#pragma once

#include <vector>

#include "BroadPhase.h"
#include "CollidingContact.h"
#include "MeshContactGenerator.h"

namespace Riemann
{
	class ContactManifold;

	class NarrowPhase
	{
	public:
		virtual ~NarrowPhase() {}
		virtual void BeginFrame(uint64_t Frame) { (void)Frame; }
		virtual void ResetState() {}
		virtual void Invalidate(const Geometry* Shape) { (void)Shape; }
		virtual size_t GetCachedPairCount() const { return 0; }
		virtual void SetTriangleContactsEnabled(bool Enabled) { (void)Enabled; }
		virtual void SetContactGeneration(const ContactGenerationSettings& Settings, float Dt) { (void)Settings; (void)Dt; }
		virtual void CollisionDetection(GeometryWorldStateSpan states,
			const std::vector<OverlapPair>& overlaps,
			std::vector<ContactManifold*>* contact) = 0;

		static NarrowPhase* Create_GJKEPA(const MeshContactSettings& MeshSettings = MeshContactSettings());
		static NarrowPhase* Create_SAT();
		static NarrowPhase* Create_PCM(const ContactPersistenceSettings& Settings = ContactPersistenceSettings(),
			const MeshContactSettings& MeshSettings = MeshContactSettings());
	};
}
