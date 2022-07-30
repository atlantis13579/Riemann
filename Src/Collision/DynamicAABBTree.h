#pragma once

#include <assert.h>
#include <float.h>
#include <algorithm>
#include <vector>
#include "../Maths/Box3d.h"

class DynamicAABBTree
{
public:
	static constexpr float kAABBThickness = 0.1f;
	static constexpr float kAABBMultiplier = 4.0f;
	
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

		for (int i = 0; i < m_nodeCapacity - 1; ++i)
		{
			m_nodes[i].next = i + 1;
			m_nodes[i].height = -1;
		}
		m_nodes[m_nodeCapacity-1].next = -1;
		m_nodes[m_nodeCapacity-1].height = -1;
		m_freeList = 0;
	}
	
	~DynamicAABBTree()
	{
	}

	int AddNode(const Box3d& aabb, void* userData)
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

	void RemoveNode(int nodeId)
	{
		assert(0 <= nodeId && nodeId < m_nodeCapacity);
		assert(m_nodes[nodeId].IsLeaf());

		RemoveLeaf(nodeId);
		FreeNode(nodeId);
	}

	bool MoveNode(int nodeId, const Box3d& aabb, const Vector3& displacement)
	{
		assert(0 <= nodeId && nodeId < m_nodeCapacity);
		assert(m_nodes[nodeId].IsLeaf());

		Vector3 thickness(kAABBThickness, kAABBThickness, kAABBThickness);
		Box3d newAABB(aabb.mMin - thickness, aabb.mMax + thickness);

		// Predict AABB movement
		Vector3 dir = kAABBMultiplier * displacement;

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
			Box3d largeAABB(newAABB.mMin - kAABBMultiplier * thickness, newAABB.mMax + kAABBMultiplier * thickness);
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
	

	void Rebuild()
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
	
	bool Validate() const
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

private:
	int AllocNode()
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
	
	void RemoveLeaf(int leaf)
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
	
	bool ValidateRecursive(int index) const
	{
		if (index == -1)
		{
			return true;
		}

		if (index == m_root)
		{
			if (m_nodes[index].parent != -1)
				return false;
		}

		const Node* node = &m_nodes[index];

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

		if (m_nodes[child1].parent != index)
			return false;
		
		if (m_nodes[child2].parent != index)
			return false;

		return ValidateRecursive(child1) && ValidateRecursive(child2);
	}

	bool ValidateAABBs(int index) const
	{
		if (index == -1)
		{
			return true;
		}

		const Node* node = &m_nodes[index];

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

		Box3d aabb;
		aabb = Box3d(m_nodes[child1].aabb, m_nodes[child2].aabb);

		if (aabb.mMin != node->aabb.mMin)
			return false;
		
		if (aabb.mMax != node->aabb.mMax)
			return false;

		return ValidateAABBs(child1) && ValidateAABBs(child2);
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
};
