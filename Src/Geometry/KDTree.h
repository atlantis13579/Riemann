#pragma once

#include <algorithm>
#include <vector>

#include "../Maths/Vector3.h"

namespace Riemann
{
	class KDTree
	{
	public:
		KDTree()
		{
			m_offset = 0;
			m_root = 0;
			m_size = -1;
		}

		~KDTree()
		{
		}

		void Build(const std::vector<Vector3>& vB, int vpNum, float eps)
		{
			m_vb = vB;
			m_size = vpNum;
			std::vector<int> indices(vpNum);
			for (int i = 0; i < vpNum; ++i)
			{
				indices[i] = i;
			}
			m_nodes.resize(vpNum);
			m_root = _build(indices.data(), 0, m_size, 0, eps);
		}

		void Query(const Vector3& center, float radius, std::vector<int>& v_hit) const
		{
			_query(m_root, center, radius * radius, v_hit);
		}

	private:
		void _query(int node, const Vector3& center, float sqr_radius, std::vector<int>& v_hit) const
		{
			if (center.x > m_nodes[node].maxBound.x || center.x < m_nodes[node].minBound.x ||
				center.y > m_nodes[node].maxBound.y || center.y < m_nodes[node].minBound.y ||
				center.z > m_nodes[node].maxBound.z || center.z < m_nodes[node].minBound.z)
			{
				return;
			}
			int vIndex = m_nodes[node].index;

			if ((m_vb[vIndex] - center).SquareLength() <= sqr_radius)
			{
				v_hit.push_back(vIndex);
			}

			if (m_nodes[node].left)
				_query(m_nodes[node].left, center, sqr_radius, v_hit);

			if (m_nodes[node].right)
				_query(m_nodes[node].right, center, sqr_radius, v_hit);
		}


		struct Vector3Cmp
		{
			int Dim;
			Vector3* vB;
			Vector3Cmp(int _D, Vector3* _vB) {
				this->Dim = _D;
				this->vB = _vB;
			}
			bool operator()(int vp1, int vp2) const {
				if (Dim == 0)
					return vB[vp1].x < vB[vp2].x;
				else if (Dim == 1)
					return vB[vp1].y < vB[vp2].y;
				return vB[vp1].z < vB[vp2].z;
			}
		};

		int _build(int* indices, int l, int r, int dim, float eps)
		{
			if (l >= r) return 0;
			int mid = (l + r) >> 1;
			std::nth_element(indices + l, indices + mid, indices + r, Vector3Cmp(dim, m_vb.data()));
			int tIndex = m_offset;
			++m_offset;
			int midIndex = indices[mid];
			m_nodes[tIndex].index = midIndex;
			m_nodes[tIndex].dim = dim;
			m_nodes[tIndex].maxBound = m_vb[midIndex] + eps;
			m_nodes[tIndex].minBound = m_vb[midIndex] - eps;
			m_nodes[tIndex].left = _build(indices, l, mid, (dim + 1) % 3, eps);
			if (m_nodes[tIndex].left)
			{
				m_nodes[tIndex].maxBound = ElementMax(m_nodes[m_nodes[tIndex].left].maxBound, m_nodes[tIndex].maxBound);
				m_nodes[tIndex].minBound = ElementMin(m_nodes[m_nodes[tIndex].left].minBound, m_nodes[tIndex].minBound);
			}
			m_nodes[tIndex].right = _build(indices, mid + 1, r, (dim + 1) % 3, eps);
			if (m_nodes[tIndex].right)
			{
				m_nodes[tIndex].maxBound = ElementMax(m_nodes[m_nodes[tIndex].right].maxBound, m_nodes[tIndex].maxBound);
				m_nodes[tIndex].minBound = ElementMin(m_nodes[m_nodes[tIndex].right].minBound, m_nodes[tIndex].minBound);
			}
			return tIndex;
		}

		static inline Vector3 ElementMax(const Vector3& v1, const Vector3& v2) { return v1.Max(v2); }
		static inline Vector3 ElementMin(const Vector3& v1, const Vector3& v2) { return v1.Min(v2); }

		struct Node
		{
			int index;
			int left;
			int right;
			Vector3 minBound;
			Vector3 maxBound;
			int dim;
			Node()
			{
				index = 0;
				left = 0;
				right = 0;
				minBound = Vector3::Zero();
				maxBound = Vector3::Zero();
				dim = 0;
			}
		};

		std::vector<Node> m_nodes;
		std::vector<Vector3> m_vb;
		int m_offset;
		int m_size;
		int m_root;
	};

}