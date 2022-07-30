#pragma once

#include <assert.h>
#include <float.h>
#include <algorithm>
#include <vector>
#include "../Maths/Box3d.h"

static const float kAabbExtension = 0.1f;
static const float kAabbMultiplier = 4.0f;

class DynamicAABBTree
{
public:
	struct Node
	{
		bool IsLeaf() const
		{
			return child1 == -1;
		}

		union
		{
			int parent;
			int next;
		};
		Box3d aabb;
		int child1;
		int child2;
		int height;
		void *userData;
		bool moved;
	};
	
	DynamicAABBTree()
	{
		m_root = -1;

		m_nodeCapacity = 16;
		m_nodeCount = 0;
		m_nodes.resize(m_nodeCapacity);
		memset(&m_nodes[0], 0, m_nodeCapacity * sizeof(Node));

		// Build a linked list for the free list.
		for (int i = 0; i < m_nodeCapacity - 1; ++i)
		{
			m_nodes[i].next = i + 1;
			m_nodes[i].height = -1;
		}
		m_nodes[m_nodeCapacity-1].next = -1;
		m_nodes[m_nodeCapacity-1].height = -1;
		m_freeList = 0;

		m_insertionCount = 0;
	}
	
	~DynamicAABBTree()
	{
	}

	int CreateProxy(const Box3d& aabb, void* userData)
	{
		int proxyId = AllocateNode();

		// Fatten the aabb.
		Vector3 r(kAabbExtension, kAabbExtension, kAabbExtension);
		m_nodes[proxyId].aabb.mMin = aabb.mMin - r;
		m_nodes[proxyId].aabb.mMax = aabb.mMax + r;
		m_nodes[proxyId].userData = userData;
		m_nodes[proxyId].height = 0;
		m_nodes[proxyId].moved = true;

		InsertLeaf(proxyId);

		return proxyId;
	}

	void DestroyProxy(int proxyId)
	{
		assert(0 <= proxyId && proxyId < m_nodeCapacity);
		assert(m_nodes[proxyId].IsLeaf());

		RemoveLeaf(proxyId);
		FreeNode(proxyId);
	}

	bool MoveProxy(int proxyId, const Box3d& aabb, const Vector3& displacement)
	{
		assert(0 <= proxyId && proxyId < m_nodeCapacity);
		assert(m_nodes[proxyId].IsLeaf());

		// Extend AABB
		Box3d fatAABB;
		Vector3 r(kAabbExtension, kAabbExtension, kAabbExtension);
		fatAABB.mMin = aabb.mMin - r;
		fatAABB.mMax = aabb.mMax + r;

		// Predict AABB movement
		Vector3 d = kAabbMultiplier * displacement;

		if (d.x < 0.0f)
		{
			fatAABB.mMin.x += d.x;
		}
		else
		{
			fatAABB.mMax.x += d.x;
		}

		if (d.y < 0.0f)
		{
			fatAABB.mMin.y += d.y;
		}
		else
		{
			fatAABB.mMax.y += d.y;
		}

		const Box3d& treeAABB = m_nodes[proxyId].aabb;
		if (aabb.IsInside(treeAABB))
		{
			// The tree AABB still contains the object, but it might be too large.
			// Perhaps the object was moving fast but has since gone to sleep.
			// The huge AABB is larger than the new fat AABB.
			Box3d hugeAABB;
			hugeAABB.mMin = fatAABB.mMin - kAabbMultiplier * r;
			hugeAABB.mMax = fatAABB.mMax + kAabbMultiplier * r;

			if (treeAABB.IsInside(hugeAABB))
			{
				// The tree AABB contains the object AABB and the tree AABB is
				// not too large. No tree update needed.
				return false;
			}

			// Otherwise the tree AABB is huge and needs to be shrunk
		}

		RemoveLeaf(proxyId);

		m_nodes[proxyId].aabb = fatAABB;

		InsertLeaf(proxyId);

		m_nodes[proxyId].moved = true;

		return true;
	}
	

	void RebuildBottomUp()
	{
		int* nodes = new int[m_nodeCount];
		int count = 0;

		// Build array of leaves. Free the rest.
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

			int parentIndex = AllocateNode();
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

	void ShiftOrigin(const Vector3& newOrigin)
	{
		// Build array of leaves. Free the rest.
		for (int i = 0; i < m_nodeCapacity; ++i)
		{
			m_nodes[i].aabb.mMin -= newOrigin;
			m_nodes[i].aabb.mMax -= newOrigin;
		}
	}
	
private:
	int AllocateNode()
	{
		// Expand the node pool as needed.
		if (m_freeList == -1)
		{
			assert(m_nodeCount == m_nodeCapacity);

			// The free list is empty. Rebuild a bigger pool.
			m_nodeCapacity *= 2;
			m_nodes.resize(m_nodeCapacity);

			// Build a linked list for the free list. The parent
			// pointer becomes the "next" pointer.
			for (int i = m_nodeCount; i < m_nodeCapacity - 1; ++i)
			{
				m_nodes[i].next = i + 1;
				m_nodes[i].height = -1;
			}
			m_nodes[m_nodeCapacity-1].next = -1;
			m_nodes[m_nodeCapacity-1].height = -1;
			m_freeList = m_nodeCount;
		}

		// Peel a node off the free list.
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
	
	void FreeNode(int nodeId)
	{
		assert(0 <= nodeId && nodeId < m_nodeCapacity);
		assert(0 < m_nodeCount);
		m_nodes[nodeId].next = m_freeList;
		m_nodes[nodeId].height = -1;
		m_freeList = nodeId;
		--m_nodeCount;
	}
	

	void InsertLeaf(int leaf)
	{
		++m_insertionCount;

		if (m_root == -1)
		{
			m_root = leaf;
			m_nodes[m_root].parent = -1;
			return;
		}

		// Find the best sibling for this node
		Box3d leafAABB = m_nodes[leaf].aabb;
		int index = m_root;
		while (m_nodes[index].IsLeaf() == false)
		{
			int child1 = m_nodes[index].child1;
			int child2 = m_nodes[index].child2;

			float area = m_nodes[index].aabb.GetVolume();

			Box3d MergedAABB = Box3d(m_nodes[index].aabb, leafAABB);
			float MergedArea = MergedAABB.GetVolume();

			// Cost of creating a new parent for this node and the new leaf
			float cost = 2.0f * MergedArea;

			// Minimum cost of pushing the leaf further down the tree
			float inheritanceCost = 2.0f * (MergedArea - area);

			// Cost of descending into child1
			float cost1;
			if (m_nodes[child1].IsLeaf())
			{
				Box3d aabb = Box3d(leafAABB, m_nodes[child1].aabb);
				cost1 = aabb.GetVolume() + inheritanceCost;
			}
			else
			{
				Box3d aabb = Box3d(leafAABB, m_nodes[child1].aabb);
				float oldArea = m_nodes[child1].aabb.GetVolume();
				float newArea = aabb.GetVolume();
				cost1 = (newArea - oldArea) + inheritanceCost;
			}

			// Cost of descending into child2
			float cost2;
			if (m_nodes[child2].IsLeaf())
			{
				Box3d aabb = Box3d(leafAABB, m_nodes[child2].aabb);
				cost2 = aabb.GetVolume() + inheritanceCost;
			}
			else
			{
				Box3d aabb = Box3d(leafAABB, m_nodes[child2].aabb);
				float oldArea = m_nodes[child2].aabb.GetVolume();
				float newArea = aabb.GetVolume();
				cost2 = newArea - oldArea + inheritanceCost;
			}

			// Descend according to the minimum cost.
			if (cost < cost1 && cost < cost2)
			{
				break;
			}

			// Descend
			if (cost1 < cost2)
			{
				index = child1;
			}
			else
			{
				index = child2;
			}
		}

		int sibling = index;

		// Create a new parent.
		int oldParent = m_nodes[sibling].parent;
		int newParent = AllocateNode();
		m_nodes[newParent].parent = oldParent;
		m_nodes[newParent].userData = nullptr;
		m_nodes[newParent].aabb = Box3d(leafAABB, m_nodes[sibling].aabb);
		m_nodes[newParent].height = m_nodes[sibling].height + 1;

		if (oldParent != -1)
		{
			// The sibling was not the root.
			if (m_nodes[oldParent].child1 == sibling)
			{
				m_nodes[oldParent].child1 = newParent;
			}
			else
			{
				m_nodes[oldParent].child2 = newParent;
			}

			m_nodes[newParent].child1 = sibling;
			m_nodes[newParent].child2 = leaf;
			m_nodes[sibling].parent = newParent;
			m_nodes[leaf].parent = newParent;
		}
		else
		{
			// The sibling was the root.
			m_nodes[newParent].child1 = sibling;
			m_nodes[newParent].child2 = leaf;
			m_nodes[sibling].parent = newParent;
			m_nodes[leaf].parent = newParent;
			m_root = newParent;
		}

		// Walk back up the tree fixing heights and AABBs
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

		//Validate();
	}
	
	void RemoveLeaf(int leaf)
	{
		if (leaf == m_root)
		{
			m_root = -1;
			return;
		}

		int parent = m_nodes[leaf].parent;
		int grandParent = m_nodes[parent].parent;
		int sibling;
		if (m_nodes[parent].child1 == leaf)
		{
			sibling = m_nodes[parent].child2;
		}
		else
		{
			sibling = m_nodes[parent].child1;
		}

		if (grandParent != -1)
		{
			// Destroy parent and connect sibling to grandParent.
			if (m_nodes[grandParent].child1 == parent)
			{
				m_nodes[grandParent].child1 = sibling;
			}
			else
			{
				m_nodes[grandParent].child2 = sibling;
			}
			m_nodes[sibling].parent = grandParent;
			FreeNode(parent);

			// Adjust ancestor bounds.
			int index = grandParent;
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
			m_root = sibling;
			m_nodes[sibling].parent = -1;
			FreeNode(parent);
		}

		//Validate();
	}

	int Balance(int iA)
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

	int GetHeight() const
	{
		if (m_root == -1)
		{
			return 0;
		}

		return m_nodes[m_root].height;
	}
	
	int ComputeHeight(int nodeId) const
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
	
	int ComputeHeight() const
	{
		int height = ComputeHeight(m_root);
		return height;
	}

	void ValidateStructure(int index) const
	{
		if (index == -1)
		{
			return;
		}

		if (index == m_root)
		{
			assert(m_nodes[index].parent == -1);
		}

		const Node* node = &m_nodes[index];

		int child1 = node->child1;
		int child2 = node->child2;

		if (node->IsLeaf())
		{
			assert(child1 == -1);
			assert(child2 == -1);
			assert(node->height == 0);
			return;
		}

		assert(0 <= child1 && child1 < m_nodeCapacity);
		assert(0 <= child2 && child2 < m_nodeCapacity);

		assert(m_nodes[child1].parent == index);
		assert(m_nodes[child2].parent == index);

		ValidateStructure(child1);
		ValidateStructure(child2);
	}

	void ValidateMetrics(int index) const
	{
		if (index == -1)
		{
			return;
		}

		const Node* node = &m_nodes[index];

		int child1 = node->child1;
		int child2 = node->child2;

		if (node->IsLeaf())
		{
			assert(child1 == -1);
			assert(child2 == -1);
			assert(node->height == 0);
			return;
		}

		assert(0 <= child1 && child1 < m_nodeCapacity);
		assert(0 <= child2 && child2 < m_nodeCapacity);

		int height1 = m_nodes[child1].height;
		int height2 = m_nodes[child2].height;
		int height;
		height = 1 + std::max(height1, height2);
		assert(node->height == height);

		Box3d aabb;
		aabb = Box3d(m_nodes[child1].aabb, m_nodes[child2].aabb);

		assert(aabb.mMin == node->aabb.mMin);
		assert(aabb.mMax == node->aabb.mMax);

		ValidateMetrics(child1);
		ValidateMetrics(child2);
	}

	void Validate() const
	{
		ValidateStructure(m_root);
		ValidateMetrics(m_root);

		int freeCount = 0;
		int freeIndex = m_freeList;
		while (freeIndex != -1)
		{
			assert(0 <= freeIndex && freeIndex < m_nodeCapacity);
			freeIndex = m_nodes[freeIndex].next;
			++freeCount;
		}

		assert(GetHeight() == ComputeHeight());
		assert(m_nodeCount + freeCount == m_nodeCapacity);
	}

	int GetMaxBalance() const
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

	
private:
	int m_root;
	std::vector<Node> m_nodes;
	int m_nodeCount;
	int m_nodeCapacity;
	int m_freeList;
	int m_insertionCount;
};
