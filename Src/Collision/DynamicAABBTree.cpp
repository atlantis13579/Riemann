
#include <assert.h>
#include "DynamicAABBTree.h"

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

	StaticStack<uint32_t, TREE_MAX_DEPTH> stack;
	stack.Push(m_root);

	while (!stack.Empty())
	{
		const Node* p = &m_nodes[stack.Pop()];
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
			const Node* Left = &m_nodes[p->child1];
			const Node* Right = &m_nodes[p->child2];

			Result->AddTestCount(2);

			bool hit1 = Ray.IntersectAABB(Left->aabb.mMin, Left->aabb.mMax, &t1);
			bool hit2 = Ray.IntersectAABB(Right->aabb.mMin, Right->aabb.mMax, &t2);

			if (Option->Type != RayCastOption::RAYCAST_PENETRATE)
			{
				hit1 = hit1 && t1 < Result->hitTimeMin && t1 < Option->MaxDist;
				hit2 = hit2 && t2 < Result->hitTimeMin && t2 < Option->MaxDist;
			}
			
			if (hit1 && hit2)
			{
				if (t1 < t2)
				{
					p = Left;
					stack.Push(p->child1);
				}
				else
				{
					p = Right;
					stack.Push(p->child2);
				}
				continue;
			}
			else if (hit1)
			{
				p = Left;
				continue;
			}
			else if (hit2)
			{
				p = Right;
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

static bool OverlapGeometry(Geometry *geometry, void* userData, const OverlapOption* Option, OverlapResult* Result)
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

bool DynamicAABBTree::Overlap(Geometry *geometry, const OverlapOption* Option, OverlapResult *Result) const
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

	StaticStack<uint32_t, TREE_MAX_DEPTH> stack;
	stack.Push(0);

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
			const Node* Left = &m_nodes[p->child1];
			const Node* Right = &m_nodes[p->child2];

			bool intersect1 = aabb.Intersect(Left->aabb.mMin, Left->aabb.mMax);
			bool intersect2 = aabb.Intersect(Right->aabb.mMin, Right->aabb.mMax);

			Result->AddTestCount(2);
			
			if (intersect1 && intersect2)
			{
				float d1 = (aabb.GetCenter() - Left->aabb.GetCenter()).SquareLength();
				float d2 = (aabb.GetCenter() - Right->aabb.GetCenter()).SquareLength();

				if (d1 < d2)
				{
					p = Left;
					stack.Push(p->child1);
				}
				else
				{
					p = Right;
					stack.Push(p->child2);
				}
				continue;
			}
			else if (intersect1)
			{
				p = Left;
				continue;
			}
			else if (intersect2)
			{
				p = Right;
				continue;
			}

			break;
		}

	}

	return Result->overlaps;
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

		int index1 = nodes[iMin];
		int index2 = nodes[jMin];
		Node* child1 = &m_nodes[index1];
		Node* child2 = &m_nodes[index2];

		int parentIndex = AllocNode();
		Node* parent = &m_nodes[parentIndex];
		parent->child1 = index1;
		parent->child2 = index2;
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
	int index = m_root;
	while (m_nodes[index].IsLeaf() == false)
	{
		float vol = m_nodes[index].aabb.GetVolume();
		Box3d mergedAABB = Box3d(m_nodes[index].aabb, leafAABB);
		float mergedVol = mergedAABB.GetVolume();

		float cost = 2.0f * mergedVol;
		float inheritanceCost = 2.0f * (mergedVol - vol);

		int child[2] = {m_nodes[index].child1, m_nodes[index].child2};
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

		index = costChild[0] < costChild[1] ? child[0] : child[1];
	}
	
	int oldParent = m_nodes[index].parent;
	int newParent = AllocNode();
	m_nodes[newParent].parent = oldParent;
	m_nodes[newParent].userData = nullptr;
	m_nodes[newParent].aabb = Box3d(leafAABB, m_nodes[index].aabb);
	m_nodes[newParent].height = m_nodes[index].height + 1;

	if (oldParent != -1)
	{
		if (m_nodes[oldParent].child1 == index)
		{
			m_nodes[oldParent].child1 = newParent;
		}
		else
		{
			m_nodes[oldParent].child2 = newParent;
		}

		m_nodes[newParent].child1 = index;
		m_nodes[newParent].child2 = leaf;
		m_nodes[index].parent = newParent;
		m_nodes[leaf].parent = newParent;
	}
	else
	{
		m_nodes[newParent].child1 = index;
		m_nodes[newParent].child2 = leaf;
		m_nodes[index].parent = newParent;
		m_nodes[leaf].parent = newParent;
		m_root = newParent;
	}

	// update AABBs from bottom up
	index = m_nodes[leaf].parent;
	while (index != -1)
	{
		index = Balance(index);

		int child1 = m_nodes[index].child1;
		int child2 = m_nodes[index].child2;

		assert(child1 != -1);
		assert(child2 != -1);

		m_nodes[index].height = 1 + std::max(m_nodes[child1].height, m_nodes[child2].height);
		m_nodes[index].aabb = Box3d(m_nodes[child1].aabb, m_nodes[child2].aabb);

		index = m_nodes[index].parent;
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
	int index = m_nodes[parent].child1 == leaf ? index = m_nodes[parent].child2 :	index = m_nodes[parent].child1;

	if (grandParent != -1)
	{
		if (m_nodes[grandParent].child1 == parent)
		{
			m_nodes[grandParent].child1 = index;
		}
		else
		{
			m_nodes[grandParent].child2 = index;
		}
		m_nodes[index].parent = grandParent;
		FreeNode(parent);

		index = grandParent;
		while (index != -1)
		{
			index = Balance(index);

			int child1 = m_nodes[index].child1;
			int child2 = m_nodes[index].child2;

			m_nodes[index].aabb = Box3d(m_nodes[child1].aabb, m_nodes[child2].aabb);
			m_nodes[index].height = 1 + std::max(m_nodes[child1].height, m_nodes[child2].height);

			index = m_nodes[index].parent;
		}
	}
	else
	{
		m_root = index;
		m_nodes[index].parent = -1;
		FreeNode(parent);
	}
}

int DynamicAABBTree::Balance(int iA)
{
	assert(iA != -1);

	Node* A = &m_nodes[iA];
	if (A->IsLeaf() || A->height < 2)
	{
		return iA;
	}

	int iB = A->child1;
	int iC = A->child2;
	assert(0 <= iB && iB < m_nodeCapacity);
	assert(0 <= iC && iC < m_nodeCapacity);

	Node* B = &m_nodes[iB];
	Node* C = &m_nodes[iC];

	int balance = C->height - B->height;

	// Rotate C up
	if (balance > 1)
	{
		int iF = C->child1;
		int iG = C->child2;
		Node* F = &m_nodes[iF];
		Node* G = &m_nodes[iG];
		assert(0 <= iF && iF < m_nodeCapacity);
		assert(0 <= iG && iG < m_nodeCapacity);

		// Swap A and C
		C->child1 = iA;
		C->parent = A->parent;
		A->parent = iC;

		// A's old parent should point to C
		if (C->parent != -1)
		{
			if (m_nodes[C->parent].child1 == iA)
			{
				m_nodes[C->parent].child1 = iC;
			}
			else
			{
				assert(m_nodes[C->parent].child2 == iA);
				m_nodes[C->parent].child2 = iC;
			}
		}
		else
		{
			m_root = iC;
		}

		// Rotate
		if (F->height > G->height)
		{
			C->child2 = iF;
			A->child2 = iG;
			G->parent = iA;
			A->aabb = Box3d(B->aabb, G->aabb);
			C->aabb = Box3d(A->aabb, F->aabb);

			A->height = 1 + std::max(B->height, G->height);
			C->height = 1 + std::max(A->height, F->height);
		}
		else
		{
			C->child2 = iG;
			A->child2 = iF;
			F->parent = iA;
			A->aabb = Box3d(B->aabb, F->aabb);
			C->aabb = Box3d(A->aabb, G->aabb);

			A->height = 1 + std::max(B->height, F->height);
			C->height = 1 + std::max(A->height, G->height);
		}

		return iC;
	}
	
	// Rotate B up
	if (balance < -1)
	{
		int iD = B->child1;
		int iE = B->child2;
		Node* D = &m_nodes[iD];
		Node* E = &m_nodes[iE];
		assert(0 <= iD && iD < m_nodeCapacity);
		assert(0 <= iE && iE < m_nodeCapacity);

		// Swap A and B
		B->child1 = iA;
		B->parent = A->parent;
		A->parent = iB;

		// A's old parent should point to B
		if (B->parent != -1)
		{
			if (m_nodes[B->parent].child1 == iA)
			{
				m_nodes[B->parent].child1 = iB;
			}
			else
			{
				assert(m_nodes[B->parent].child2 == iA);
				m_nodes[B->parent].child2 = iB;
			}
		}
		else
		{
			m_root = iB;
		}

		// Rotate
		if (D->height > E->height)
		{
			B->child2 = iD;
			A->child1 = iE;
			E->parent = iA;
			A->aabb = Box3d(C->aabb, E->aabb);
			B->aabb = Box3d(A->aabb, D->aabb);

			A->height = 1 + std::max(C->height, E->height);
			B->height = 1 + std::max(A->height, D->height);
		}
		else
		{
			B->child2 = iE;
			A->child1 = iD;
			D->parent = iA;
			A->aabb = Box3d(C->aabb, D->aabb);
			B->aabb = Box3d(A->aabb, E->aabb);

			A->height = 1 + std::max(C->height, D->height);
			B->height = 1 + std::max(A->height, E->height);
		}

		return iB;
	}

	return iA;
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


