#include "AABBPruner.h"

#include <algorithm>
#include <float.h>

#include "GeometryQuery.h"
#include "GeometryObject.h"

namespace Riemann
{
	static int NormalizeMaxObjects(int maxObjects)
	{
		return maxObjects < 0 ? INT_MAX : maxObjects;
	}

	static int NormalizeBucketSize(int bucketSize)
	{
		return bucketSize > 0 ? bucketSize : 16;
	}

	static bool HasCapacity(size_t count, int maxObjects)
	{
		return count < (size_t)maxObjects;
	}

	static int GetLargestAxis(const Box3& box)
	{
		const Vector3 size = box.GetSize();
		if (size.y > size.x && size.y >= size.z)
		{
			return 1;
		}
		if (size.z > size.x && size.z >= size.y)
		{
			return 2;
		}
		return 0;
	}

	static bool RayCastObjectRange(const Ray3& ray, Geometry* const* objects, int count, const RayCastOption* option, RayCastResult* result)
	{
		bool hit = false;

		for (int i = 0; i < count; ++i)
		{
			Geometry* object = objects[i];
			if (object == nullptr || !object->IsQueryEnabled())
			{
				continue;
			}

			float t;
			const Box3& box = object->GetBoundingVolume_WorldSpace();
			if (!ray.IntersectAABB(box.Min, box.Max, &t) || t >= option->MaxDist)
			{
				continue;
			}

			if (option->Type != RayCastOption::RAYCAST_PENETRATE && t >= result->hitTimeMin)
			{
				continue;
			}

			if (object == option->Cache.prevhitGeom)
			{
				continue;
			}

			if (!object->RayCast(ray.Origin, ray.Dir, option, result))
			{
				continue;
			}

			hit = true;
			result->hit = true;
			if (option->Type == RayCastOption::RAYCAST_ANY)
			{
				result->hitGeom = object;
				result->hitTimeMin = result->hitTime;
				result->hitPoint = ray.PointAt(result->hitTimeMin);
				return true;
			}

			if (option->Type == RayCastOption::RAYCAST_PENETRATE)
			{
				result->hitGeometries.push_back(object);
				if (option->MaxObjects > 0 && result->hitGeometries.size() >= (size_t)option->MaxObjects)
				{
					return true;
				}
			}

			if (result->hitTime < result->hitTimeMin)
			{
				result->hitTimeMin = result->hitTime;
				result->hitGeom = object;
			}
		}

		return hit;
	}

	static bool IntersectObjectRange(const Geometry* geometry, Geometry* const* objects, int count, const IntersectOption* option, IntersectResult* result)
	{
		bool hit = false;
		const Box3& queryBox = geometry->GetBoundingVolume_WorldSpace();

		for (int i = 0; i < count; ++i)
		{
			Geometry* object = objects[i];
			if (object == nullptr || !object->IsQueryEnabled())
			{
				continue;
			}

			if (!queryBox.Intersect(object->GetBoundingVolume_WorldSpace()))
			{
				continue;
			}

			if (option->Filter && !option->Filter->IsCollidable(option->FilterData, object->GetFilterData()))
			{
				continue;
			}

			result->AddTestCount(1);
			if (!geometry->Intersect(object))
			{
				continue;
			}

			hit = true;
			result->overlaps = true;
			if (result->overlapGeoms.size() < option->maxOverlaps)
			{
				result->overlapGeoms.push_back(object);
			}

			if (result->overlapGeoms.size() >= option->maxOverlaps)
			{
				return true;
			}
		}

		return hit;
	}

	static bool SweepObjectRange(const Geometry* geometry, Geometry* const* objects, int count, const Vector3& direction, const SweepOption* option, SweepResult* result)
	{
		bool hit = false;

		for (int i = 0; i < count; ++i)
		{
			Geometry* object = objects[i];
			if (object == nullptr || !object->IsQueryEnabled())
			{
				continue;
			}

			float fastT;
			const Box3& box = object->GetBoundingVolume_WorldSpace();
			if (!geometry->SweepTestFast(direction, box.Min, box.Max, &fastT))
			{
				continue;
			}

			if (option->Type != SweepOption::SWEEP_PENETRATE && (fastT >= result->hitTimeMin || fastT >= option->MaxDist))
			{
				continue;
			}

			float t;
			Vector3 position;
			Vector3 normal;
			if (!geometry->Sweep(direction, object, &position, &normal, &t))
			{
				continue;
			}

			hit = true;
			result->hit = true;
			if (option->Type == SweepOption::SWEEP_ANY)
			{
				result->hitGeom = object;
				result->hitTimeMin = t;
				result->hitPosition = position;
				result->hitNormal = normal;
				return true;
			}

			if (option->Type == SweepOption::SWEEP_PENETRATE)
			{
				result->hitGeometries.push_back(object);
			}

			if (t < result->hitTimeMin)
			{
				result->hitTimeMin = t;
				result->hitGeom = object;
				result->hitPosition = position;
				result->hitNormal = normal;
			}
		}

		return hit;
	}

	class AABBSimplePruner : public AABBPruner
	{
	public:
		explicit AABBSimplePruner(int maxObjects = 64)
		{
			m_MaxObjects = NormalizeMaxObjects(maxObjects);
		}

		virtual ~AABBSimplePruner() override {}

		virtual void Clear() override
		{
			m_Objects.clear();
		}

		virtual bool Add(Geometry* object) override
		{
			if (object == nullptr || !object->IsQueryEnabled())
			{
				return false;
			}
			if (!HasCapacity(m_Objects.size(), m_MaxObjects))
			{
				return false;
			}
			if (FindObject(object) >= 0)
			{
				return false;
			}

			m_Objects.push_back(object);
			return true;
		}

		virtual bool Remove(Geometry* object) override
		{
			const int index = FindObject(object);
			if (index < 0)
			{
				return false;
			}

			m_Objects.erase(m_Objects.begin() + index);
			return true;
		}

		virtual bool Update(Geometry* object) override
		{
			if (object == nullptr)
			{
				return false;
			}

			const int index = FindObject(object);
			if (!object->IsQueryEnabled())
			{
				return index >= 0 ? Remove(object) : false;
			}
			if (index < 0)
			{
				return Add(object);
			}
			return true;
		}

		virtual bool RayCast(const Ray3& ray, const RayCastOption* option, RayCastResult* result) const override
		{
			result->Reset();
			if (option == nullptr || m_Objects.empty())
			{
				return false;
			}

			const bool hit = RayCastObjectRange(ray, m_Objects.data(), (int)m_Objects.size(), option, result);
			if (hit || !result->hitGeometries.empty())
			{
				if (result->hitTimeMin < FLT_MAX)
				{
					result->hitPoint = ray.PointAt(result->hitTimeMin);
				}
				return true;
			}
			return false;
		}

		virtual bool Intersect(const Geometry* geometry, const IntersectOption* option, IntersectResult* result) const override
		{
			result->Reset();
			result->overlaps = false;
			if (geometry == nullptr || option == nullptr || m_Objects.empty())
			{
				return false;
			}
			return IntersectObjectRange(geometry, m_Objects.data(), (int)m_Objects.size(), option, result);
		}

		virtual bool Sweep(const Geometry* geometry, const Vector3& direction, const SweepOption* option, SweepResult* result) const override
		{
			result->Reset();
			if (geometry == nullptr || option == nullptr || m_Objects.empty())
			{
				return false;
			}
			return SweepObjectRange(geometry, m_Objects.data(), (int)m_Objects.size(), direction, option, result);
		}

		virtual void CollectAABBs(std::vector<Box3>* aabbs) const override
		{
			if (aabbs == nullptr)
			{
				return;
			}

			for (Geometry* object : m_Objects)
			{
				if (object && object->IsQueryEnabled())
				{
					aabbs->push_back(object->GetBoundingVolume_WorldSpace());
				}
			}
		}

		virtual int GetObjectCount() const override
		{
			return (int)m_Objects.size();
		}

	private:
		int FindObject(const Geometry* object) const
		{
			if (object == nullptr)
			{
				return -1;
			}

			const std::vector<Geometry*>::const_iterator iter = std::find(m_Objects.begin(), m_Objects.end(), object);
			if (iter == m_Objects.end())
			{
				return -1;
			}
			return (int)(iter - m_Objects.begin());
		}

	private:
		std::vector<Geometry*>	m_Objects;
		int						m_MaxObjects;
	};

	class AABBBucketPruner : public AABBPruner
	{
	private:
		struct Bucket
		{
			Box3	bounds;
			int		first;
			int		count;
		};

	public:
		explicit AABBBucketPruner(int maxObjects = INT_MAX, int bucketSize = 16)
		{
			m_MaxObjects = NormalizeMaxObjects(maxObjects);
			m_BucketSize = NormalizeBucketSize(bucketSize);
			m_Dirty = true;
		}

		virtual ~AABBBucketPruner() override {}

		virtual void Clear() override
		{
			m_Objects.clear();
			m_SortedObjects.clear();
			m_Buckets.clear();
			m_Dirty = false;
		}

		virtual bool Add(Geometry* object) override
		{
			if (object == nullptr || !object->IsQueryEnabled())
			{
				return false;
			}
			if (!HasCapacity(m_Objects.size(), m_MaxObjects))
			{
				return false;
			}
			if (FindObject(object) >= 0)
			{
				return false;
			}

			m_Objects.push_back(object);
			m_Dirty = true;
			return true;
		}

		virtual bool Remove(Geometry* object) override
		{
			const int index = FindObject(object);
			if (index < 0)
			{
				return false;
			}

			m_Objects.erase(m_Objects.begin() + index);
			m_Dirty = true;
			return true;
		}

		virtual bool Update(Geometry* object) override
		{
			if (object == nullptr)
			{
				return false;
			}

			const int index = FindObject(object);
			if (!object->IsQueryEnabled())
			{
				return index >= 0 ? Remove(object) : false;
			}
			if (index < 0)
			{
				return Add(object);
			}

			m_Dirty = true;
			return true;
		}

		virtual bool RayCast(const Ray3& ray, const RayCastOption* option, RayCastResult* result) const override
		{
			result->Reset();
			if (option == nullptr || m_Objects.empty())
			{
				return false;
			}

			BuildBuckets();

			bool hit = false;
			for (const Bucket& bucket : m_Buckets)
			{
				float t;
				if (!ray.IntersectAABB(bucket.bounds.Min, bucket.bounds.Max, &t) || t >= option->MaxDist)
				{
					continue;
				}
				if (option->Type != RayCastOption::RAYCAST_PENETRATE && t >= result->hitTimeMin)
				{
					continue;
				}

				const bool bucketHit = RayCastObjectRange(ray, m_SortedObjects.data() + bucket.first, bucket.count, option, result);
				if (bucketHit)
				{
					hit = true;
					if (option->Type == RayCastOption::RAYCAST_ANY)
					{
						return true;
					}
					if (option->Type == RayCastOption::RAYCAST_PENETRATE &&
						option->MaxObjects > 0 &&
						result->hitGeometries.size() >= (size_t)option->MaxObjects)
					{
						return true;
					}
				}
			}

			if (hit || !result->hitGeometries.empty())
			{
				if (result->hitTimeMin < FLT_MAX)
				{
					result->hitPoint = ray.PointAt(result->hitTimeMin);
				}
				return true;
			}
			return false;
		}

		virtual bool Intersect(const Geometry* geometry, const IntersectOption* option, IntersectResult* result) const override
		{
			result->Reset();
			result->overlaps = false;
			if (geometry == nullptr || option == nullptr || m_Objects.empty())
			{
				return false;
			}

			BuildBuckets();

			bool hit = false;
			const Box3& queryBox = geometry->GetBoundingVolume_WorldSpace();
			for (const Bucket& bucket : m_Buckets)
			{
				if (!queryBox.Intersect(bucket.bounds))
				{
					continue;
				}

				const bool bucketHit = IntersectObjectRange(geometry, m_SortedObjects.data() + bucket.first, bucket.count, option, result);
				if (bucketHit)
				{
					hit = true;
					if (result->overlapGeoms.size() >= option->maxOverlaps)
					{
						return true;
					}
				}
			}
			return hit;
		}

		virtual bool Sweep(const Geometry* geometry, const Vector3& direction, const SweepOption* option, SweepResult* result) const override
		{
			result->Reset();
			if (geometry == nullptr || option == nullptr || m_Objects.empty())
			{
				return false;
			}

			BuildBuckets();

			bool hit = false;
			for (const Bucket& bucket : m_Buckets)
			{
				float fastT;
				if (!geometry->SweepTestFast(direction, bucket.bounds.Min, bucket.bounds.Max, &fastT))
				{
					continue;
				}
				if (option->Type != SweepOption::SWEEP_PENETRATE && (fastT >= result->hitTimeMin || fastT >= option->MaxDist))
				{
					continue;
				}

				const bool bucketHit = SweepObjectRange(geometry, m_SortedObjects.data() + bucket.first, bucket.count, direction, option, result);
				if (bucketHit)
				{
					hit = true;
					if (option->Type == SweepOption::SWEEP_ANY)
					{
						return true;
					}
				}
			}

			return hit || !result->hitGeometries.empty();
		}

		virtual void CollectAABBs(std::vector<Box3>* aabbs) const override
		{
			if (aabbs == nullptr)
			{
				return;
			}

			BuildBuckets();
			for (const Bucket& bucket : m_Buckets)
			{
				aabbs->push_back(bucket.bounds);
			}
		}

		virtual int GetObjectCount() const override
		{
			return (int)m_Objects.size();
		}

	private:
		int FindObject(const Geometry* object) const
		{
			if (object == nullptr)
			{
				return -1;
			}

			const std::vector<Geometry*>::const_iterator iter = std::find(m_Objects.begin(), m_Objects.end(), object);
			if (iter == m_Objects.end())
			{
				return -1;
			}
			return (int)(iter - m_Objects.begin());
		}

		void BuildBuckets() const
		{
			if (!m_Dirty)
			{
				return;
			}

			m_SortedObjects.clear();
			m_Buckets.clear();

			Box3 globalBox = Box3::Empty();
			for (Geometry* object : m_Objects)
			{
				if (object == nullptr || !object->IsQueryEnabled())
				{
					continue;
				}

				const Box3& bounds = object->GetBoundingVolume_WorldSpace();
				globalBox.Encapsulate(bounds);
				m_SortedObjects.push_back(object);
			}

			if (m_SortedObjects.empty())
			{
				m_Dirty = false;
				return;
			}

			const int sortAxis = GetLargestAxis(globalBox);
			std::sort(m_SortedObjects.begin(), m_SortedObjects.end(),
				[sortAxis](Geometry* a, Geometry* b)
				{
					const float ca = a->GetBoundingVolume_WorldSpace().GetCenter()[sortAxis];
					const float cb = b->GetBoundingVolume_WorldSpace().GetCenter()[sortAxis];
					return ca < cb;
				});

			m_Buckets.reserve((m_SortedObjects.size() + (size_t)m_BucketSize - 1) / (size_t)m_BucketSize);
			for (size_t first = 0; first < m_SortedObjects.size(); first += (size_t)m_BucketSize)
			{
				const size_t count = std::min((size_t)m_BucketSize, m_SortedObjects.size() - first);
				Bucket bucket;
				bucket.bounds = Box3::Empty();
				bucket.first = (int)first;
				bucket.count = (int)count;

				for (size_t i = 0; i < count; ++i)
				{
					bucket.bounds.Encapsulate(m_SortedObjects[first + i]->GetBoundingVolume_WorldSpace());
				}
				m_Buckets.push_back(bucket);
			}

			m_Dirty = false;
		}

	private:
		std::vector<Geometry*>			m_Objects;
		mutable std::vector<Geometry*>	m_SortedObjects;
		mutable std::vector<Bucket>		m_Buckets;
		int								m_MaxObjects;
		int								m_BucketSize;
		mutable bool					m_Dirty;
	};

	AABBPruner::AABBPruner()
	{
	}

	AABBPruner::~AABBPruner()
	{
	}

	AABBPruner* AABBPruner::CreateSimple(int maxObjects)
	{
		return new AABBSimplePruner(maxObjects);
	}

	AABBPruner* AABBPruner::CreateBucketStyle(int maxObjects, int bucketSize)
	{
		return new AABBBucketPruner(maxObjects, bucketSize);
	}
}
