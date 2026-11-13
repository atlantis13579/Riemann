#pragma once

#include <vector>
#include <memory>
#include <stdint.h>
#include "../Maths/Vector3.h"
#include "../Core/BatchList.h"

namespace Riemann
{
	class Geometry;
	struct GeometryWorldState;

	struct ContactPersistenceSettings
	{
		ContactPersistenceSettings();
		bool IsValid() const;
		float MatchDistance;
		float BreakDistance;
		float NormalCosine;
		float RelativeTranslationThreshold;
		float RelativeRotationCosine;
		uint64_t RefreshInterval;
		uint64_t RetentionFrames;
		float WarmStartFactor;
		float MinTimeStepRatio;
		float MaxTimeStepRatio;
	};

	struct Contact
	{
	public:
		Contact()
		{
			totalImpulseNormal = 0.0f;
			totalImpulseTangent = 0.0f;
			totalImpulseBinormal = 0.0f;
			PositionLocalA = PositionLocalB = PositionWorldA = PositionWorldB = Vector3::Zero();
			Normal = Tangent = Binormal = Vector3::Zero();
			PenetrationDepth = 0.0f;
		}

		Vector3	PositionWorldA;
		Vector3	PositionWorldB;
		Vector3	PositionLocalA;
		Vector3	PositionLocalB;
		Vector3	Normal;
		Vector3	Tangent;
		Vector3	Binormal;
		float	PenetrationDepth;
		float RestOffset = 0.0f;
		float SolverSlop = -1.0f; // Negative selects the legacy solver's tolerance.

		//	For warm start
		float	totalImpulseNormal;
		float	totalImpulseTangent;
		float	totalImpulseBinormal;
		uint32_t FeatureA = UINT32_MAX;
		uint32_t FeatureB = UINT32_MAX;
		uint64_t PersistentId = 0;
		float CachedDt = 0.0f;
	};

#define MAX_CONTACT_POINTS 4

	class ContactManifold
	{
	public:
		int indexA = 0;
		int indexB = 0;
		uint32_t PatchId = 0;
		Contact ContactPoints[MAX_CONTACT_POINTS + 1];
		int	NumContactPointCount = 0;

		ContactManifold* next = nullptr;

		void	Reset()
		{
			NumContactPointCount = 0;
		}

		void	AddNewContact(int a, int b, const Contact& result);

	private:
		bool	MergeManifold();
		bool	MergeManifold2();
	};

	class ContactManifoldManager
	{
	public:
		explicit ContactManifoldManager(const ContactPersistenceSettings& Settings = ContactPersistenceSettings());
		~ContactManifoldManager();
		void BeginFrame(uint64_t Frame);
		void Reset();
		void Invalidate(const Geometry* Shape);
		std::vector<uint32_t> GetPatchIds(const GeometryWorldState& A, const GeometryWorldState& B) const;
		void RetainPatches(const GeometryWorldState& A, const GeometryWorldState& B, const std::vector<uint32_t>& PatchIds);
		bool Refresh(const GeometryWorldState& A, const GeometryWorldState& B, ContactManifold* Manifold);
		int Match(const GeometryWorldState& A, const GeometryWorldState& B, ContactManifold* Manifold, bool Impulses);
		void Store(const GeometryWorldState& A, const GeometryWorldState& B, ContactManifold& Manifold,
			float Dt, bool Regenerated = true, bool Accumulate = false);
		size_t GetPairCount() const;

	private:
		struct Storage;
		std::unique_ptr<Storage> m_Storage;
	};

	class ContactReport
	{
	public:
		virtual ~ContactReport() {}
	};

}
