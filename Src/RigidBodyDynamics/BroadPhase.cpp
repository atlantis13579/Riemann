
#include "BroadPhase.h"
#include "RigidBody.h"
#include "../Collision/DynamicAABBTree.h"
#include "../Collision/GeometryObject.h"
#include "../Collision/SAP_Incremental.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <unordered_map>
#include <vector>

namespace Riemann
{
	static bool IsMovingRigid(Geometry* geom)
	{
		RigidBody* body = geom->GetParent<RigidBody>();
		if (body == nullptr)
		{
			return false;
		}
		return body->mRigidType != RigidType::Static;
	}

	static bool IsValidPair(Geometry* geom1, Geometry* geom2)
	{
		if (geom1->GetParent<void*>() == geom2->GetParent<void*>())
		{
			return false;
		}
		return IsMovingRigid(geom1) || IsMovingRigid(geom2);
	}

	class BroadPhaseAllPairsImplementation : public BroadPhase
	{
	public:
		virtual ~BroadPhaseAllPairsImplementation() {}

		virtual void ProduceOverlaps(const std::vector<Geometry*>& geoms, std::vector<OverlapPair>* overlaps) override final
		{
			overlaps->clear();

			int n = (int)geoms.size();
			for (int i = 0; i < n; ++i)
			for (int j = 0; j < n; ++j)
			{
				if (i == j) continue;
				overlaps->emplace_back(i, j);
			}
		}
	};

	class BroadPhaseBruteforceImplementation : public BroadPhase
	{
	public:
		virtual ~BroadPhaseBruteforceImplementation() {}

		virtual void ProduceOverlaps(const std::vector<Geometry*>& geoms, std::vector<OverlapPair>* overlaps) override final
		{
			overlaps->clear();

			int n = (int)geoms.size();
			for (int i = 0; i < n; ++i)
			for (int j = i + 1; j < n; ++j)
			{
				Geometry* gi = geoms[i];
				Geometry* gj = geoms[j];
				if (!IsValidPair(gi, gj))
					continue;

				const Box3& box1 = gi->GetBoundingVolume_WorldSpace();
				const Box3& box2 = gj->GetBoundingVolume_WorldSpace();
				if (box1.Intersect(box2))
				{
					overlaps->emplace_back(i, j);
				}
			}
		}

	};

	class BroadPhaseSAPImplementation : public BroadPhase, public SAP::BoundingVolumeProxy
	{
	public:
		BroadPhaseSAPImplementation()
		{
			m_SAP = new IncrementalSAP(this, { 0, 1, 2 });
		}

		virtual ~BroadPhaseSAPImplementation()
		{
			if (m_SAP)
			{
				delete m_SAP;
			}
		}

		void SetDirty()
		{
			if (m_SAP)
			{
				m_SAP->SetDirty();
			}
		}

	public:

		virtual void ProduceOverlaps(const std::vector<Geometry*>& geoms, std::vector<OverlapPair>* overlaps) override final
		{
			if (geoms.empty())
			{
				return;
			}

			m_pObjects = &geoms;
			m_SAP->IncrementalPrune(&m_Overlaps);

			overlaps->clear();
			for (auto it : m_Overlaps)
			{
				int i, j;
				SAP::UnpackOverlapKey(it, &i, &j);
				Geometry* gi = geoms[i];
				Geometry* gj = geoms[j];
				if (!IsValidPair(gi, gj))
					continue;
				overlaps->emplace_back(i, j);
			}
		}

	private:
		virtual int     GetBoundingVolumeCount() const override final
		{
			return (int)m_pObjects->size();
		}

		virtual float* GetBoundingVolumeCoordinate(int bv_i, bool left, int axis) const override final
		{
			const Box3& box = m_pObjects->at(bv_i)->GetBoundingVolume_WorldSpace();
			float* p = (float*)&box;
			return left ? p + axis : p + 3 + axis;
		}

		virtual bool    Overlaps(int bv_i, int bv_j) const override final
		{
			const Box3& box1 = m_pObjects->at(bv_i)->GetBoundingVolume_WorldSpace();
			const Box3& box2 = m_pObjects->at(bv_j)->GetBoundingVolume_WorldSpace();
			return box1.Intersect(box2);
		}

		virtual uint64_t	CalculateBoundingVolumeHash() const override final
		{
			if (m_pObjects == nullptr || m_pObjects->empty())
			{
				return 0;
			}

			uint64_t hash = 0;
			for (size_t i = 0; i < m_pObjects->size(); ++i)
			{
				hash ^= reinterpret_cast<uint64_t>(m_pObjects->at(i));
				hash *= static_cast<uint64_t>(1099511628211ULL);
			}
			return hash;
		}

	private:
		IncrementalSAP* m_SAP;
		std::set<OverlapKey>			m_Overlaps;
		const std::vector<Geometry*>* m_pObjects;
	};

	class BroadPhaseABPImplementation : public BroadPhase
	{
	public:
		virtual ~BroadPhaseABPImplementation() {}

		virtual void ProduceOverlaps(const std::vector<Geometry*>& geoms, std::vector<OverlapPair>* overlaps) override final
		{
			overlaps->clear();
			m_Geoms = &geoms;
			m_Overlaps = overlaps;
			m_UniquePairs.clear();

			if (geoms.size() < 2)
			{
				return;
			}

			std::vector<int> sortedObjects;
			sortedObjects.reserve(geoms.size());
			for (size_t i = 0; i < geoms.size(); ++i)
			{
				sortedObjects.push_back((int)i);
			}
			SortByMinX(&sortedObjects);
			PruneRecursive(sortedObjects, 0);
		}

	private:
		enum
		{
			BucketCount = 5,
			PruneThreshold = 512,
			MaxDepth = 8,
		};

		bool OverlapsYZ(const Box3& box1, const Box3& box2) const
		{
			if (box1.Min.y > box2.Max.y || box2.Min.y > box1.Max.y)
			{
				return false;
			}

			if (box1.Min.z > box2.Max.z || box2.Min.z > box1.Max.z)
			{
				return false;
			}

			return true;
		}

		void SortByMinX(std::vector<int>* indices) const
		{
			std::sort(indices->begin(), indices->end(), [this](int lhs, int rhs)
				{
					const Box3& box1 = m_Geoms->at(lhs)->GetBoundingVolume_WorldSpace();
					const Box3& box2 = m_Geoms->at(rhs)->GetBoundingVolume_WorldSpace();
					if (box1.Min.x == box2.Min.x)
					{
						return lhs < rhs;
					}
					return box1.Min.x < box2.Min.x;
				});
		}

		void AddPair(int id1, int id2)
		{
			if (id1 == id2)
			{
				return;
			}

			if (id1 > id2)
			{
				std::swap(id1, id2);
			}

			Geometry* geom1 = m_Geoms->at(id1);
			Geometry* geom2 = m_Geoms->at(id2);
			if (!IsValidPair(geom1, geom2))
			{
				return;
			}

			const Box3& box1 = geom1->GetBoundingVolume_WorldSpace();
			const Box3& box2 = geom2->GetBoundingVolume_WorldSpace();
			if (!box1.Intersect(box2))
			{
				return;
			}

			const OverlapKey key = SAP::PackOverlapKey(id1, id2);
			if (!m_UniquePairs.insert(key).second)
			{
				return;
			}

			m_Overlaps->emplace_back(id1, id2);
		}

		void CompleteBoxPruning(const std::vector<int>& sortedObjects)
		{
			const size_t n = sortedObjects.size();
			for (size_t i = 0; i < n; ++i)
			{
				const int id1 = sortedObjects[i];
				const Box3& box1 = m_Geoms->at(id1)->GetBoundingVolume_WorldSpace();

				for (size_t j = i + 1; j < n; ++j)
				{
					const int id2 = sortedObjects[j];
					const Box3& box2 = m_Geoms->at(id2)->GetBoundingVolume_WorldSpace();
					if (box2.Min.x > box1.Max.x)
					{
						break;
					}

					if (OverlapsYZ(box1, box2))
					{
						AddPair(id1, id2);
					}
				}
			}
		}

		void BipartiteBoxPruning(const std::vector<int>& sortedObjects1, const std::vector<int>& sortedObjects2)
		{
			const size_t n1 = sortedObjects1.size();
			const size_t n2 = sortedObjects2.size();
			if (n1 == 0 || n2 == 0)
			{
				return;
			}

			size_t running2 = 0;
			for (size_t i = 0; i < n1 && running2 < n2; ++i)
			{
				const int id1 = sortedObjects1[i];
				const Box3& box1 = m_Geoms->at(id1)->GetBoundingVolume_WorldSpace();
				while (running2 < n2 && m_Geoms->at(sortedObjects2[running2])->GetBoundingVolume_WorldSpace().Min.x < box1.Min.x)
				{
					++running2;
				}

				for (size_t j = running2; j < n2; ++j)
				{
					const int id2 = sortedObjects2[j];
					const Box3& box2 = m_Geoms->at(id2)->GetBoundingVolume_WorldSpace();
					if (box2.Min.x > box1.Max.x)
					{
						break;
					}

					if (OverlapsYZ(box1, box2))
					{
						AddPair(id1, id2);
					}
				}
			}

			size_t running1 = 0;
			for (size_t i = 0; i < n2 && running1 < n1; ++i)
			{
				const int id2 = sortedObjects2[i];
				const Box3& box2 = m_Geoms->at(id2)->GetBoundingVolume_WorldSpace();
				while (running1 < n1 && m_Geoms->at(sortedObjects1[running1])->GetBoundingVolume_WorldSpace().Min.x <= box2.Min.x)
				{
					++running1;
				}

				for (size_t j = running1; j < n1; ++j)
				{
					const int id1 = sortedObjects1[j];
					const Box3& box1 = m_Geoms->at(id1)->GetBoundingVolume_WorldSpace();
					if (box1.Min.x > box2.Max.x)
					{
						break;
					}

					if (OverlapsYZ(box1, box2))
					{
						AddPair(id1, id2);
					}
				}
			}
		}

		int ClassifyBox(const Box3& box, float limitY, float limitZ) const
		{
			static const int Codes[16] = { 4, 4, 4, -1, 4, 3, 2, -1, 4, 1, 0, -1, -1, -1, -1, -1 };

			const bool upperPart = box.Min.z > limitZ;
			const bool rightPart = box.Min.y > limitY;
			const bool lowerPart = box.Max.z < limitZ;
			const bool leftPart = box.Max.y < limitY;

			const int code = (rightPart ? 1 : 0) | (leftPart ? 2 : 0) | (upperPart ? 4 : 0) | (lowerPart ? 8 : 0);
			const int bucket = Codes[code];
			return bucket >= 0 ? bucket : 4;
		}

		bool SplitBuckets(const std::vector<int>& sortedObjects, std::array<std::vector<int>, BucketCount>* buckets)
		{
			if (sortedObjects.size() < 2)
			{
				return false;
			}

			float minY = std::numeric_limits<float>::max();
			float minZ = std::numeric_limits<float>::max();
			float maxY = -std::numeric_limits<float>::max();
			float maxZ = -std::numeric_limits<float>::max();

			for (int id : sortedObjects)
			{
				const Box3& box = m_Geoms->at(id)->GetBoundingVolume_WorldSpace();
				minY = std::min(minY, box.Min.y);
				minZ = std::min(minZ, box.Min.z);
				maxY = std::max(maxY, box.Max.y);
				maxZ = std::max(maxZ, box.Max.z);
			}

			const float limitY = (minY + maxY) * 0.5f;
			const float limitZ = (minZ + maxZ) * 0.5f;
			for (std::vector<int>& bucket : *buckets)
			{
				bucket.clear();
			}

			for (int id : sortedObjects)
			{
				const Box3& box = m_Geoms->at(id)->GetBoundingVolume_WorldSpace();
				(*buckets)[ClassifyBox(box, limitY, limitZ)].push_back(id);
			}

			for (const std::vector<int>& bucket : *buckets)
			{
				if (bucket.size() == sortedObjects.size())
				{
					return false;
				}
			}

			return true;
		}

		void PruneRecursive(const std::vector<int>& sortedObjects, int depth)
		{
			if (sortedObjects.size() <= PruneThreshold || depth >= MaxDepth)
			{
				CompleteBoxPruning(sortedObjects);
				return;
			}

			std::array<std::vector<int>, BucketCount> buckets;
			if (!SplitBuckets(sortedObjects, &buckets))
			{
				CompleteBoxPruning(sortedObjects);
				return;
			}

			for (int i = 0; i < BucketCount; ++i)
			{
				if (!buckets[i].empty())
				{
					PruneRecursive(buckets[i], depth + 1);
				}
			}

			const std::vector<int>& crossBucket = buckets[BucketCount - 1];
			if (!crossBucket.empty())
			{
				for (int i = 0; i < BucketCount - 1; ++i)
				{
					BipartiteBoxPruning(buckets[i], crossBucket);
				}
			}
		}

	private:
		const std::vector<Geometry*>* m_Geoms = nullptr;
		std::vector<OverlapPair>* m_Overlaps = nullptr;
		std::set<OverlapKey> m_UniquePairs;
	};

	class BroadPhaseMBPImplementation : public BroadPhase
	{
	public:
		virtual ~BroadPhaseMBPImplementation() {}

		virtual void ProduceOverlaps(const std::vector<Geometry*>& geoms, std::vector<OverlapPair>* overlaps) override final
		{
			overlaps->clear();
			m_Geoms = &geoms;
			m_Overlaps = overlaps;
			m_UniquePairs.clear();

			if (geoms.size() < 2)
			{
				return;
			}

			Box3 sceneBounds;
			if (!BuildSceneBounds(&sceneBounds))
			{
				return;
			}

			const Vector3 sceneSize = sceneBounds.GetSize();
			const Vector3 step(sceneSize.x / (float)AxisDivisions, sceneSize.y / (float)AxisDivisions, sceneSize.z / (float)AxisDivisions);

			std::array<std::vector<int>, RegionCount> regions;
			for (int i = 0; i < (int)geoms.size(); ++i)
			{
				const Box3& box = geoms[i]->GetBoundingVolume_WorldSpace();
				const int minX = AxisCell(box.Min.x, sceneBounds.Min.x, step.x);
				const int maxX = AxisCell(box.Max.x, sceneBounds.Min.x, step.x);
				const int minY = AxisCell(box.Min.y, sceneBounds.Min.y, step.y);
				const int maxY = AxisCell(box.Max.y, sceneBounds.Min.y, step.y);
				const int minZ = AxisCell(box.Min.z, sceneBounds.Min.z, step.z);
				const int maxZ = AxisCell(box.Max.z, sceneBounds.Min.z, step.z);

				for (int z = minZ; z <= maxZ; ++z)
				{
					for (int y = minY; y <= maxY; ++y)
					{
						for (int x = minX; x <= maxX; ++x)
						{
							regions[RegionIndex(x, y, z)].push_back(i);
						}
					}
				}
			}

			for (std::vector<int>& regionObjects : regions)
			{
				if (regionObjects.size() < 2)
				{
					continue;
				}

				SortByMinX(&regionObjects);
				CompleteBoxPruning(regionObjects);
			}
		}

	private:
		enum
		{
			AxisDivisions = 2,
			RegionCount = AxisDivisions * AxisDivisions * AxisDivisions,
		};

		bool OverlapsYZ(const Box3& box1, const Box3& box2) const
		{
			if (box1.Min.y > box2.Max.y || box2.Min.y > box1.Max.y)
			{
				return false;
			}

			if (box1.Min.z > box2.Max.z || box2.Min.z > box1.Max.z)
			{
				return false;
			}

			return true;
		}

		void SortByMinX(std::vector<int>* indices) const
		{
			std::sort(indices->begin(), indices->end(), [this](int lhs, int rhs)
				{
					const Box3& box1 = m_Geoms->at(lhs)->GetBoundingVolume_WorldSpace();
					const Box3& box2 = m_Geoms->at(rhs)->GetBoundingVolume_WorldSpace();
					if (box1.Min.x == box2.Min.x)
					{
						return lhs < rhs;
					}
					return box1.Min.x < box2.Min.x;
				});
		}

		void AddPair(int id1, int id2)
		{
			if (id1 == id2)
			{
				return;
			}

			if (id1 > id2)
			{
				std::swap(id1, id2);
			}

			Geometry* geom1 = m_Geoms->at(id1);
			Geometry* geom2 = m_Geoms->at(id2);
			if (!IsValidPair(geom1, geom2))
			{
				return;
			}

			const Box3& box1 = geom1->GetBoundingVolume_WorldSpace();
			const Box3& box2 = geom2->GetBoundingVolume_WorldSpace();
			if (!box1.Intersect(box2))
			{
				return;
			}

			const OverlapKey key = SAP::PackOverlapKey(id1, id2);
			if (!m_UniquePairs.insert(key).second)
			{
				return;
			}

			m_Overlaps->emplace_back(id1, id2);
		}

		void CompleteBoxPruning(const std::vector<int>& sortedObjects)
		{
			const size_t n = sortedObjects.size();
			for (size_t i = 0; i < n; ++i)
			{
				const int id1 = sortedObjects[i];
				const Box3& box1 = m_Geoms->at(id1)->GetBoundingVolume_WorldSpace();

				for (size_t j = i + 1; j < n; ++j)
				{
					const int id2 = sortedObjects[j];
					const Box3& box2 = m_Geoms->at(id2)->GetBoundingVolume_WorldSpace();
					if (box2.Min.x > box1.Max.x)
					{
						break;
					}

					if (OverlapsYZ(box1, box2))
					{
						AddPair(id1, id2);
					}
				}
			}
		}

		bool BuildSceneBounds(Box3* bounds) const
		{
			if (m_Geoms->empty())
			{
				return false;
			}

			*bounds = m_Geoms->at(0)->GetBoundingVolume_WorldSpace();
			for (size_t i = 1; i < m_Geoms->size(); ++i)
			{
				bounds->Encapsulate(m_Geoms->at(i)->GetBoundingVolume_WorldSpace());
			}
			return true;
		}

		static int ClampCell(int cell)
		{
			if (cell < 0)
			{
				return 0;
			}
			if (cell >= AxisDivisions)
			{
				return AxisDivisions - 1;
			}
			return cell;
		}

		static int AxisCell(float value, float origin, float step)
		{
			if (step <= std::numeric_limits<float>::epsilon())
			{
				return 0;
			}
			return ClampCell((int)std::floor((value - origin) / step));
		}

		static int RegionIndex(int x, int y, int z)
		{
			return x + y * AxisDivisions + z * AxisDivisions * AxisDivisions;
		}

	private:
		const std::vector<Geometry*>* m_Geoms = nullptr;
		std::vector<OverlapPair>* m_Overlaps = nullptr;
		std::set<OverlapKey> m_UniquePairs;
	};

	class BroadPhaseDynamicAABBImplementation : public BroadPhase
	{
	public:
		BroadPhaseDynamicAABBImplementation(DynamicAABBTree* tree)
		{
			m_tree = tree;
		}
		virtual ~BroadPhaseDynamicAABBImplementation() {}

		virtual void ProduceOverlaps(const std::vector<Geometry*>& geoms, std::vector<OverlapPair>* overlaps) override final
		{
			overlaps->clear();
			if (m_tree == nullptr || geoms.empty())
			{
				return;
			}

			std::unordered_map<Geometry*, int> geomToIndex;
			geomToIndex.reserve(geoms.size());
			for (int i = 0; i < (int)geoms.size(); ++i)
			{
				geomToIndex[geoms[i]] = i;
			}

			std::vector<void*> result;
			int n = (int)geoms.size();
			for (int i = 0; i < n; ++i)
			{
				Geometry* gi = geoms[i];
				if (!IsMovingRigid(gi))
					continue;

				m_tree->Query(gi->GetBoundingVolume_WorldSpace(), &result);
				for (int j = 0; j < (int)result.size(); ++j)
				{
					Geometry* gj = static_cast<Geometry*>(result[j]);
					auto it = geomToIndex.find(gj);
					if (it == geomToIndex.end())
					{
						continue;
					}

					const int index2 = it->second;
					if (index2 == i)
					{
						continue;
					}

					if (!IsValidPair(gi, gj))
					{
						continue;
					}

					if (IsMovingRigid(gj) && index2 < i)
					{
						continue;
					}

					if (!gi->GetBoundingVolume_WorldSpace().Intersect(gj->GetBoundingVolume_WorldSpace()))
					{
						continue;
					}

					overlaps->emplace_back(std::min(i, index2), std::max(i, index2));
				}
			}
		}

	private:
		DynamicAABBTree* m_tree;
	};

	BroadPhase* BroadPhase::Create_SAP()
	{
		return new BroadPhaseSAPImplementation();
	}

	BroadPhase* BroadPhase::Create_ABP()
	{
		return new BroadPhaseABPImplementation();
	}

	BroadPhase* BroadPhase::Create_MBP()
	{
		return new BroadPhaseMBPImplementation();
	}

	BroadPhase* BroadPhase::Create_DynamicAABB(DynamicAABBTree* tree)
	{
		return new BroadPhaseDynamicAABBImplementation(tree);
	}

	BroadPhase* BroadPhase::Create_Bruteforce()
	{
		return new BroadPhaseBruteforceImplementation();
	}

	BroadPhase* BroadPhase::Create_AllPairs()
	{
		return new BroadPhaseAllPairsImplementation();
	}
}
