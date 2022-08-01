
#include <assert.h>
#include "DynamicAABBTree.h"
#include "GeometryQuery.h"
#include "GeometryObject.h"

DynamicAABBTree::DynamicAABBTree()
{
	m_root = -1;

	m_nodeCapacity = 16;
	m_nodeCount = 0;
	m_nodes.resize(m_nodeCapacity);
	memset(&m_nodes[0], 0, m_nodeCapacity * sizeof(Node));

	for (int i = 0; i < m_nodeCapacity - 1; ++i)
	{
		m_nodes[i].next = i + 1;
		m_nodes[i].height = -1;
	}
	m_nodes[m_nodeCapacity-1].next = -1;
	m_nodes[m_nodeCapacity-1].height = -1;
	m_freeList = 0;
}

DynamicAABBTree::~DynamicAABBTree()
{
}

static bool RayIntersectGeometry(const Ray3d& Ray, void* userData, const RayCastOption* Option, RayCastResult* Result)
{
	Geometry *Geom = static_cast<Geometry*>(userData);
	
	bool hit = Geom->RayCast(Ray.Origin, Ray.Dir, Option, Result);
	if (hit)
	{
		if (Option->Type == RayCastOption::RAYCAST_PENETRATE)
		{
			Result->hitGeometries.push_back(Geom);
		}

		if (Result->hitTime < Result->hitTimeMin)
		{
			Result->hitTimeMin = Result->hitTime;
			Result->hitGeom = Geom;
		}
		return true;
	}
	return false;
}

bool DynamicAABBTree::RayCast(const Ray3d& Ray, const RayCastOption* Option, RayCastResult *Result) const
{
	Result->hit = false;
	Result->hitTestCount = 0;
	Result->hitTimeMin = FLT_MAX;
	Result->hitGeom = nullptr;
	
	if (m_root == -1)
	{
		return false;
	}

	float t1, t2;
	const Node* p = &m_nodes[m_root];
	if (p == nullptr || !Ray.IntersectAABB(p->aabb.mMin, p->aabb.mMax, &t1) || t1 >= Option->MaxDist)
	{
		return false;
	}

	StaticStack<int, TREE_MAX_DEPTH> stack;
	stack.Push(m_root);

	while (!stack.Empty())
	{
		p = &m_nodes[stack.Pop()];
		while (p)
		{
			if (p->IsLeaf())
			{
				if (RayIntersectGeometry(Ray, p->userData, Option, Result))
				{
					if (Option->Type == RayCastOption::RAYCAST_ANY)
					{
						Result->hitPoint = Ray.PointAt(Result->hitTimeMin);
						return true;
					}
				}
				break;
			}

			assert(p->child1 != -1 && p->child2 != -1);
			const Node* child1 = &m_nodes[p->child1];
			const Node* child2 = &m_nodes[p->child2];

			Result->AddTestCount(2);

			bool hit1 = Ray.IntersectAABB(child1->aabb.mMin, child1->aabb.mMax, &t1);
			bool hit2 = Ray.IntersectAABB(child2->aabb.mMin, child2->aabb.mMax, &t2);

			if (Option->Type != RayCastOption::RAYCAST_PENETRATE)
			{
				hit1 = hit1 && t1 < Result->hitTimeMin && t1 < Option->MaxDist;
				hit2 = hit2 && t2 < Result->hitTimeMin && t2 < Option->MaxDist;
			}
			
			assert(!stack.Full());
			if (hit1 && hit2)
			{
				if (t1 < t2)
				{
					stack.Push(p->child2);
					p = child1;
				}
				else
				{
					stack.Push(p->child1);
					p = child2;
				}
				continue;
			}
			else if (hit1)
			{
				p = child1;
				continue;
			}
			else if (hit2)
			{
				p = child2;
				continue;
			}

			break;
		}

	}

	if (Result->hit || Result->hitGeometries.size() > 0)
	{
		Result->hitPoint = Ray.PointAt(Result->hitTimeMin);
		return true;
	}
	return false;
}

static bool OverlapGeometry(const Geometry *geometry, void* userData, const OverlapOption* Option, OverlapResult* Result)
{
	Geometry *candidate = static_cast<Geometry*>(userData);

	if (Option->Filter && !Option->Filter->IsCollidable(Option->FilterData, candidate->GetFilterData()))
	{
		return false;
	}
	
	Result->AddTestCount(1);
	
	bool overlap = geometry->Overlap(candidate);
	if (overlap)
	{
		Result->overlaps = true;
		if (Result->overlapGeoms.size() < Option->maxOverlaps)
		{
			Result->overlapGeoms.push_back(candidate);
		}

		if (Result->overlapGeoms.size() >= Option->maxOverlaps)
		{
			return true;
		}
	}
	return Result->overlaps;
}

bool DynamicAABBTree::Overlap(const Geometry *geometry, const OverlapOption* Option, OverlapResult *Result) const
{
	Result->overlaps = false;
	Result->overlapGeoms.clear();
	
	if (m_root == -1)
	{
		return false;
	}

	const Box3d &aabb = geometry->GetBoundingVolume_WorldSpace();
	const Node* p = &m_nodes[m_root];
	if (p == nullptr || !aabb.Intersect(p->aabb.mMin, p->aabb.mMax))
	{
		return false;
	}

	StaticStack<int, TREE_MAX_DEPTH> stack;
	stack.Push(m_root);

	while (!stack.Empty())
	{
		const Node* p = &m_nodes[stack.Pop()];
		while (p)
		{
			if (p->IsLeaf())
			{
				bool overlap = OverlapGeometry(geometry, p->userData, Option, Result);
				if (overlap)
				{
					if (Result->overlapGeoms.size() >= Option->maxOverlaps)
					{
						return true;
					}
				}
				break;
			}

			assert(p->child1 != -1 && p->child2 != -1);
			const Node* child1 = &m_nodes[p->child1];
			const Node* child2 = &m_nodes[p->child2];

			bool intersect1 = aabb.Intersect(child1->aabb.mMin, child1->aabb.mMax);
			bool intersect2 = aabb.Intersect(child2->aabb.mMin, child2->aabb.mMax);

			Result->AddTestCount(2);
			
			assert(!stack.Full());
			if (intersect1 && intersect2)
			{
				stack.Push(p->child2);
				p = child1;
				continue;
			}
			else if (intersect1)
			{
				p = child1;
				continue;
			}
			else if (intersect2)
			{
				p = child2;
				continue;
			}

			break;
		}

	}

	return Result->overlaps;
}

bool DynamicAABBTree::Query(const Box3d& aabb, std::vector<void*> *Result) const
{
	Result->clear();
	
	if (m_root == -1)
	{
		return false;
	}

	const Node* p = &m_nodes[m_root];
	if (p == nullptr || !aabb.Intersect(p->aabb.mMin, p->aabb.mMax))
	{
		return false;
	}

	StaticStack<int, TREE_MAX_DEPTH> stack;
	stack.Push(m_root);

	while (!stack.Empty())
	{
		int Id = stack.Pop();
		const Node* p = &m_nodes[Id];
		while (p)
		{
			if (p->IsLeaf())
			{
				Result->push_back(p->userData);
				break;
			}

			assert(p->child1 != -1 && p->child2 != -1);
			const Node* child1 = &m_nodes[p->child1];
			const Node* child2 = &m_nodes[p->child2];

			bool intersect1 = aabb.Intersect(child1->aabb.mMin, child1->aabb.mMax);
			bool intersect2 = aabb.Intersect(child2->aabb.mMin, child2->aabb.mMax);

			assert(!stack.Full());
			if (intersect1 && intersect2)
			{
				stack.Push(p->child2);
				p = child1;
				continue;
			}
			else if (intersect1)
			{
				p = child1;
				continue;
			}
			else if (intersect2)
			{
				p = child2;
				continue;
			}

			break;
		}

	}

	return !Result->empty();
}

int DynamicAABBTree::Add(const Box3d& aabb, void* userData)
{
	int nodeId = AllocNode();

	Vector3 thickness(kAABBThickness, kAABBThickness, kAABBThickness);
	m_nodes[nodeId].aabb.mMin = aabb.mMin - thickness;
	m_nodes[nodeId].aabb.mMax = aabb.mMax + thickness;
	m_nodes[nodeId].userData = userData;
	m_nodes[nodeId].height = 0;
	m_nodes[nodeId].moved = true;

	InsertLeaf(nodeId);

	return nodeId;
}

void DynamicAABBTree::Remove(int nodeId)
{
	assert(0 <= nodeId && nodeId < m_nodeCapacity);
	assert(m_nodes[nodeId].IsLeaf());

	RemoveLeaf(nodeId);
	FreeNode(nodeId);
}

bool DynamicAABBTree::Update(int nodeId, const Box3d& aabb, const Vector3& displacement)
{
	assert(0 <= nodeId && nodeId < m_nodeCapacity);
	assert(m_nodes[nodeId].IsLeaf());

	Vector3 thickness(kAABBThickness, kAABBThickness, kAABBThickness);
	Box3d newAABB(aabb.mMin - thickness, aabb.mMax + thickness);

	// Predict AABB movement
	Vector3 dir = kAABBFattenScale * displacement;

	if (dir.x < 0.0f)
	{
		newAABB.mMin.x += dir.x;
	}
	else
	{
		newAABB.mMax.x += dir.x;
	}

	if (dir.y < 0.0f)
	{
		newAABB.mMin.y += dir.y;
	}
	else
	{
		newAABB.mMax.y += dir.y;
	}
	
	if (dir.z < 0.0f)
	{
		newAABB.mMin.z += dir.z;
	}
	else
	{
		newAABB.mMax.z += dir.z;
	}

	const Box3d& treeAABB = m_nodes[nodeId].aabb;
	if (treeAABB.IsInside(aabb))
	{
		Box3d largeAABB(newAABB.mMin - kAABBFattenScale * thickness, newAABB.mMax + kAABBFattenScale * thickness);
		if (largeAABB.IsInside(treeAABB))
		{
			// The large AABB contains the object AABB, no tree update needed.
			return false;
		}
	}

	RemoveLeaf(nodeId);

	m_nodes[nodeId].aabb = newAABB;

	InsertLeaf(nodeId);

	m_nodes[nodeId].moved = true;

	return true;
}

void DynamicAABBTree::Rebuild()
{
	int* nodes = new int[m_nodeCount];
	int count = 0;

	for (int i = 0; i < m_nodeCapacity; ++i)
	{
		if (m_nodes[i].height < 0)
		{
			// free node in pool
			continue;
		}

		if (m_nodes[i].IsLeaf())
		{
			m_nodes[i].parent = -1;
			nodes[count] = i;
			++count;
		}
		else
		{
			FreeNode(i);
		}
	}

	while (count > 1)
	{
		float minCost = FLT_MAX;
		int iMin = -1, jMin = -1;
		for (int i = 0; i < count; ++i)
		{
			Box3d aabbi = m_nodes[nodes[i]].aabb;

			for (int j = i + 1; j < count; ++j)
			{
				Box3d aabbj = m_nodes[nodes[j]].aabb;
				Box3d b = Box3d(aabbi, aabbj);
				float cost = b.GetVolume();
				if (cost < minCost)
				{
					iMin = i;
					jMin = j;
					minCost = cost;
				}
			}
		}

		int nodeId1 = nodes[iMin];
		int nodeId2 = nodes[jMin];
		Node* child1 = &m_nodes[nodeId1];
		Node* child2 = &m_nodes[nodeId2];

		int parentIndex = AllocNode();
		Node* parent = &m_nodes[parentIndex];
		parent->child1 = nodeId1;
		parent->child2 = nodeId2;
		parent->height = 1 + std::max(child1->height, child2->height);
		parent->aabb = Box3d(child1->aabb, child2->aabb);
		parent->parent = -1;

		child1->parent = parentIndex;
		child2->parent = parentIndex;

		nodes[jMin] = nodes[count-1];
		nodes[iMin] = parentIndex;
		--count;
	}

	m_root = nodes[0];
	delete []nodes;

	Validate();
}

bool DynamicAABBTree::Validate() const
{
	if (!ValidateRecursive(m_root))
		return false;
	
	if (!ValidateAABBs(m_root))
		return false;

	int freeCount = 0;
	int freeIndex = m_freeList;
	while (freeIndex != -1)
	{
		if (!(0 <= freeIndex && freeIndex < m_nodeCapacity))
			return false;
		freeIndex = m_nodes[freeIndex].next;
		++freeCount;
	}

	if (GetHeight() != ComputeHeight())
		return false;
	if (m_nodeCount + freeCount != m_nodeCapacity)
		return false;
	return true;
}

int DynamicAABBTree::AllocNode()
{
	if (m_freeList == -1)
	{
		assert(m_nodeCount == m_nodeCapacity);

		m_nodeCapacity *= 2;
		m_nodes.resize(m_nodeCapacity);
		for (int i = m_nodeCount; i < m_nodeCapacity - 1; ++i)
		{
			m_nodes[i].next = i + 1;
			m_nodes[i].height = -1;
		}
		m_nodes[m_nodeCapacity-1].next = -1;
		m_nodes[m_nodeCapacity-1].height = -1;
		m_freeList = m_nodeCount;
	}

	int nodeId = m_freeList;
	m_freeList = m_nodes[nodeId].next;
	m_nodes[nodeId].parent = -1;
	m_nodes[nodeId].child1 = -1;
	m_nodes[nodeId].child2 = -1;
	m_nodes[nodeId].height = 0;
	m_nodes[nodeId].userData = nullptr;
	m_nodes[nodeId].moved = false;
	++m_nodeCount;
	return nodeId;
}

void DynamicAABBTree::FreeNode(int nodeId)
{
	assert(0 <= nodeId && nodeId < m_nodeCapacity);
	assert(0 < m_nodeCount);
	m_nodes[nodeId].next = m_freeList;
	m_nodes[nodeId].height = -1;
	m_freeList = nodeId;
	--m_nodeCount;
}

void DynamicAABBTree::InsertLeaf(int leaf)
{
	if (m_root == -1)
	{
		m_root = leaf;
		m_nodes[m_root].parent = -1;
		return;
	}

	// Find the best node from bottom up
	Box3d leafAABB = m_nodes[leaf].aabb;
	int nodeId = m_root;
	while (m_nodes[nodeId].IsLeaf() == false)
	{
		float Vol = m_nodes[nodeId].aabb.GetVolume();
		Box3d mergedAABB = Box3d(m_nodes[nodeId].aabb, leafAABB);
		float mergedVol = mergedAABB.GetVolume();

		float cost = 2.0f * mergedVol;
		float inheritanceCost = 2.0f * (mergedVol - Vol);

		int child[2] = {m_nodes[nodeId].child1, m_nodes[nodeId].child2};
		float costChild[2];
		
		for (int k = 0; k < 2; ++k)
		{
			float ccost;
			int c = child[k];
			if (m_nodes[c].IsLeaf())
			{
				Box3d aabb = Box3d(leafAABB, m_nodes[c].aabb);
				ccost = aabb.GetVolume() + inheritanceCost;
			}
			else
			{
				Box3d aabb = Box3d(leafAABB, m_nodes[c].aabb);
				float oldVol = m_nodes[c].aabb.GetVolume();
				float newVol = aabb.GetVolume();
				ccost = (newVol - oldVol) + inheritanceCost;
			}
			costChild[k] = ccost;
		}

		if (cost < costChild[0] && cost < costChild[1])
		{
			break;
		}

		nodeId = costChild[0] < costChild[1] ? child[0] : child[1];
	}
	
	int oldParent = m_nodes[nodeId].parent;
	int newParent = AllocNode();
	m_nodes[newParent].parent = oldParent;
	m_nodes[newParent].userData = nullptr;
	m_nodes[newParent].aabb = Box3d(leafAABB, m_nodes[nodeId].aabb);
	m_nodes[newParent].height = m_nodes[nodeId].height + 1;

	if (oldParent != -1)
	{
		if (m_nodes[oldParent].child1 == nodeId)
		{
			m_nodes[oldParent].child1 = newParent;
		}
		else
		{
			m_nodes[oldParent].child2 = newParent;
		}

		m_nodes[newParent].child1 = nodeId;
		m_nodes[newParent].child2 = leaf;
		m_nodes[nodeId].parent = newParent;
		m_nodes[leaf].parent = newParent;
	}
	else
	{
		m_nodes[newParent].child1 = nodeId;
		m_nodes[newParent].child2 = leaf;
		m_nodes[nodeId].parent = newParent;
		m_nodes[leaf].parent = newParent;
		m_root = newParent;
	}

	// update AABBs from bottom up
	nodeId = m_nodes[leaf].parent;
	while (nodeId != -1)
	{
		nodeId = Balance(nodeId);

		int child1 = m_nodes[nodeId].child1;
		int child2 = m_nodes[nodeId].child2;

		assert(child1 != -1);
		assert(child2 != -1);

		m_nodes[nodeId].height = 1 + std::max(m_nodes[child1].height, m_nodes[child2].height);
		m_nodes[nodeId].aabb = Box3d(m_nodes[child1].aabb, m_nodes[child2].aabb);

		nodeId = m_nodes[nodeId].parent;
	}
}

void DynamicAABBTree::RemoveLeaf(int leaf)
{
	if (leaf == m_root)
	{
		m_root = -1;
		return;
	}

	int parent = m_nodes[leaf].parent;
	int grandParent = m_nodes[parent].parent;
	int nodeId = m_nodes[parent].child1 == leaf ? nodeId = m_nodes[parent].child2 :	nodeId = m_nodes[parent].child1;

	if (grandParent != -1)
	{
		if (m_nodes[grandParent].child1 == parent)
		{
			m_nodes[grandParent].child1 = nodeId;
		}
		else
		{
			m_nodes[grandParent].child2 = nodeId;
		}
		m_nodes[nodeId].parent = grandParent;
		FreeNode(parent);

		nodeId = grandParent;
		while (nodeId != -1)
		{
			nodeId = Balance(nodeId);

			int child1 = m_nodes[nodeId].child1;
			int child2 = m_nodes[nodeId].child2;

			m_nodes[nodeId].aabb = Box3d(m_nodes[child1].aabb, m_nodes[child2].aabb);
			m_nodes[nodeId].height = 1 + std::max(m_nodes[child1].height, m_nodes[child2].height);

			nodeId = m_nodes[nodeId].parent;
		}
	}
	else
	{
		m_root = nodeId;
		m_nodes[nodeId].parent = -1;
		FreeNode(parent);
	}
}

int DynamicAABBTree::Balance(int nodeA)
{
	assert(nodeA != -1);

	Node* A = &m_nodes[nodeA];
	if (A->IsLeaf() || A->height < 2)
	{
		return nodeA;
	}

	int nodeB = A->child1;
	int nodeC = A->child2;
	assert(0 <= nodeB && nodeB < m_nodeCapacity);
	assert(0 <= nodeC && nodeC < m_nodeCapacity);

	Node* B = &m_nodes[nodeB];
	Node* C = &m_nodes[nodeC];

	int balance = C->height - B->height;

	// Rotate C up
	if (balance > 1)
	{
		int nodeF = C->child1;
		int nodeG = C->child2;
		Node* F = &m_nodes[nodeF];
		Node* G = &m_nodes[nodeG];
		assert(0 <= nodeF && nodeF < m_nodeCapacity);
		assert(0 <= nodeG && nodeG < m_nodeCapacity);

		// Swap A and C
		C->child1 = nodeA;
		C->parent = A->parent;
		A->parent = nodeC;

		// A's old parent should point to C
		if (C->parent != -1)
		{
			if (m_nodes[C->parent].child1 == nodeA)
			{
				m_nodes[C->parent].child1 = nodeC;
			}
			else
			{
				assert(m_nodes[C->parent].child2 == nodeA);
				m_nodes[C->parent].child2 = nodeC;
			}
		}
		else
		{
			m_root = nodeC;
		}

		// Rotate
		if (F->height > G->height)
		{
			C->child2 = nodeF;
			A->child2 = nodeG;
			G->parent = nodeA;
			A->aabb = Box3d(B->aabb, G->aabb);
			C->aabb = Box3d(A->aabb, F->aabb);

			A->height = 1 + std::max(B->height, G->height);
			C->height = 1 + std::max(A->height, F->height);
		}
		else
		{
			C->child2 = nodeG;
			A->child2 = nodeF;
			F->parent = nodeA;
			A->aabb = Box3d(B->aabb, F->aabb);
			C->aabb = Box3d(A->aabb, G->aabb);

			A->height = 1 + std::max(B->height, F->height);
			C->height = 1 + std::max(A->height, G->height);
		}

		return nodeC;
	}
	
	// Rotate B up
	if (balance < -1)
	{
		int nodeD = B->child1;
		int nodeE = B->child2;
		Node* D = &m_nodes[nodeD];
		Node* E = &m_nodes[nodeE];
		assert(0 <= nodeD && nodeD < m_nodeCapacity);
		assert(0 <= nodeE && nodeE < m_nodeCapacity);

		// Swap A and B
		B->child1 = nodeA;
		B->parent = A->parent;
		A->parent = nodeB;

		// A's old parent should point to B
		if (B->parent != -1)
		{
			if (m_nodes[B->parent].child1 == nodeA)
			{
				m_nodes[B->parent].child1 = nodeB;
			}
			else
			{
				assert(m_nodes[B->parent].child2 == nodeA);
				m_nodes[B->parent].child2 = nodeB;
			}
		}
		else
		{
			m_root = nodeB;
		}

		// Rotate
		if (D->height > E->height)
		{
			B->child2 = nodeD;
			A->child1 = nodeE;
			E->parent = nodeA;
			A->aabb = Box3d(C->aabb, E->aabb);
			B->aabb = Box3d(A->aabb, D->aabb);

			A->height = 1 + std::max(C->height, E->height);
			B->height = 1 + std::max(A->height, D->height);
		}
		else
		{
			B->child2 = nodeE;
			A->child1 = nodeD;
			D->parent = nodeA;
			A->aabb = Box3d(C->aabb, D->aabb);
			B->aabb = Box3d(A->aabb, E->aabb);

			A->height = 1 + std::max(C->height, D->height);
			B->height = 1 + std::max(A->height, E->height);
		}

		return nodeB;
	}

	return nodeA;
}

int DynamicAABBTree::GetHeight() const
{
	if (m_root == -1)
	{
		return 0;
	}

	return m_nodes[m_root].height;
}

int DynamicAABBTree::ComputeHeight(int nodeId) const
{
	assert(0 <= nodeId && nodeId < m_nodeCapacity);
	const Node* node = &m_nodes[nodeId];

	if (node->IsLeaf())
	{
		return 0;
	}

	int height1 = ComputeHeight(node->child1);
	int height2 = ComputeHeight(node->child2);
	return 1 + std::max(height1, height2);
}

int DynamicAABBTree::ComputeHeight() const
{
	int height = ComputeHeight(m_root);
	return height;
}

float DynamicAABBTree::GetVolumeRatio() const
{
	if (m_root == -1)
	{
		return 0.0f;
	}

	const Node* root = &m_nodes[m_root];
	float rootVol = root->aabb.GetVolume();

	float totalVol = 0.0f;
	for (int i = 0; i < m_nodeCapacity; ++i)
	{
		const Node* node = &m_nodes[i];
		if (node->height < 0)
		{
			continue;
		}

		totalVol += node->aabb.GetVolume();
	}

	return totalVol / rootVol;
}

int DynamicAABBTree::GetMaxBalance() const
{
	int maxBalance = 0;
	for (int i = 0; i < m_nodeCapacity; ++i)
	{
		const Node* node = &m_nodes[i];
		if (node->height <= 1)
		{
			continue;
		}

		assert(node->IsLeaf() == false);

		int child1 = node->child1;
		int child2 = node->child2;
		int balance = std::abs(m_nodes[child2].height - m_nodes[child1].height);
		maxBalance = std::max(maxBalance, balance);
	}

	return maxBalance;
}

bool DynamicAABBTree::ValidateRecursive(int nodeId) const
{
	if (nodeId == -1)
	{
		return true;
	}

	if (nodeId == m_root)
	{
		if (m_nodes[nodeId].parent != -1)
			return false;
	}

	const Node* node = &m_nodes[nodeId];

	int child1 = node->child1;
	int child2 = node->child2;

	if (node->IsLeaf())
	{
		if (child1 != -1 || child2 != -1 || node->height != 0)
			return false;
		return true;
	}

	if (!(0 <= child1 && child1 < m_nodeCapacity))
		return false;
	
	if (!(0 <= child2 && child2 < m_nodeCapacity))
		return false;

	if (m_nodes[child1].parent != nodeId)
		return false;
	
	if (m_nodes[child2].parent != nodeId)
		return false;

	return ValidateRecursive(child1) && ValidateRecursive(child2);
}

bool DynamicAABBTree::ValidateAABBs(int nodeId) const
{
	if (nodeId == -1)
	{
		return true;
	}

	const Node* node = &m_nodes[nodeId];

	int child1 = node->child1;
	int child2 = node->child2;

	if (node->IsLeaf())
	{
		if (child1 != -1 || child2 != -1 || node->height != 0)
			return false;
		return true;
	}

	if (!(0 <= child1 && child1 < m_nodeCapacity))
		return false;
	
	if (!(0 <= child2 && child2 < m_nodeCapacity))
		return false;

	int height1 = m_nodes[child1].height;
	int height2 = m_nodes[child2].height;
	int height = 1 + std::max(height1, height2);
	
	if (node->height != height)
		return false;

	Box3d aabb = Box3d(m_nodes[child1].aabb, m_nodes[child2].aabb);

	if (aabb.mMin != node->aabb.mMin)
		return false;
	
	if (aabb.mMax != node->aabb.mMax)
		return false;

	return ValidateAABBs(child1) && ValidateAABBs(child2);
}


