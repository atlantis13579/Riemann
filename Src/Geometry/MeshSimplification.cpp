
#include "../Maths/Vector3.h"
#include "../Maths/Vector4.h"
#include "../Maths/Matrix4.h"
#include "../Core/PriorityQueue.h"
#include "../CollisionPrimitive/StaticMesh.h"
#include "MeshSimplification.h"

#include <assert.h>
#include <stdio.h>
#include <unordered_map>
#include <string>

namespace Riemann
{
	const float INVERSE_LIMIT = -0.1f;

	inline Vector3 ElementMax(const Vector3& v1, const Vector3& v2) { return v1.Max(v2); }
	inline Vector3 ElementMin(const Vector3& v1, const Vector3& v2) { return v1.Min(v2); }

	class Edge;
	class Vertex {
	public:
		Vector3 pos;
		Matrix4 Q;
		int newIndex;

		std::vector<int> m_neighbors;
		std::vector<int> m_pairs;

		Vertex()
		{
			pos = Vector3::Zero();
			Q = Matrix4::Zero();
		}

		Vertex(const Vector3 &p)
		{
			pos = p;
			Q = Matrix4::Zero();
		}

		bool IsNeighbor(int index) const
		{
			for (size_t i = 0; i < m_neighbors.size(); ++i) {
				if (m_neighbors[i] == index) {
					return true;
				}
			}
			return false;
		}

		void AddNeighbor(int index)
		{
			m_neighbors.push_back(index);
		}

		void RemoveNeighbor(int index)
		{
			for (size_t i = 0; i < m_neighbors.size(); ++i)
			{
				if (m_neighbors[i] == index) {
					m_neighbors[i] = m_neighbors.back();
					m_neighbors.pop_back();
					return;
				}
			}
		}

		static Matrix4 dp_matrix(const Vector4& v)
		{
			Matrix4 dp;
			for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j)
				dp[i][j] = v[i] * v[j];
			return dp;
		}

		void ComputeQEM(const std::vector<Vertex>& vertices)
		{
			for (size_t i = 0; i < m_neighbors.size(); ++i)
			for (size_t j = i + 1; j < m_neighbors.size(); ++j)
			{
				if (vertices[m_neighbors[i]].IsNeighbor(m_neighbors[j]))
				{
					Vector3 norm = (vertices[m_neighbors[i]].pos - pos).Cross(vertices[m_neighbors[j]].pos - pos).SafeUnit();
					float w = -(pos.Dot(norm));
					Vector4 v4(norm.x, norm.y, norm.z, w);
					Q += dp_matrix(v4);
				}
			}
		}

		bool HasEdge(int index, const std::vector<Edge>& pairs) const;
		bool HasEdge(const Edge& pair, const std::vector<Edge>& pairs) const;

		void AddEdge(int index)
		{
			m_pairs.push_back(index);
		}

		void RemoveEdge(int index)
		{
			for (size_t i = 0; i < m_pairs.size(); ++i)
			{
				if (m_pairs[i] == index) {
					m_pairs[i] = m_pairs.back();
					m_pairs.pop_back();
					return;
				}
			}
		}
	};

	struct VertexCmp
	{
		int D;
		Vertex* vB;
		VertexCmp(int _D, Vertex* _vB) {
			this->D = _D;
			this->vB = _vB;
		}
		bool operator()(int vp1, int vp2) const {
			if (D == 0)
				return vB[vp1].pos.x < vB[vp2].pos.x;
			else if (D == 1)
				return vB[vp1].pos.y < vB[vp2].pos.y;
			else
				return vB[vp1].pos.z < vB[vp2].pos.z;
		}
	};

	class Edge
	{
	private:
		Vector3 optPos;
	public:
		float cost;
		float cost1;
		int index;
		int v[2];

		Edge()
		{
			cost = 0.0;
			optPos = Vector3::Zero();
			index = 0;
			v[0] = v[1] = 0;
		}

		Edge(int v0, int v1)
		{
			cost = 0.0;
			optPos = Vector3::Zero();
			index = 0;
			v[0] = v0;
			v[1] = v1;
		}

		Vector3 OptimalPos() const
		{
			return optPos;
		}

		bool operator==(const Edge& rhs) const
		{
			return index == rhs.index;
		}

		bool operator<(const Edge& p2) const
		{
			// return (cost < p2.cost) || ((fabsf(cost - p2.cost) < 1e-5) && (cost1 < p2.cost1));
			return cost < p2.cost;
		}

		void UpdateOptimalPos(const std::vector<Vertex>& vertices)
		{
			optPos = (vertices[v[0]].pos + vertices[v[1]].pos) / 2; //if no solution, choose the middle
			Matrix4 A = vertices[v[0]].Q + vertices[v[1]].Q;
			A(3, 0) = 0.0f;
			A(3, 1) = 0.0f;
			A(3, 2) = 0.0f;
			A(3, 3) = 1.0f;
			Vector4 Y(0.0, 0.0, 0.0, 1.0);
			for (int i = 0; i < 4; ++i)
			{
				A(i, 3) = -A(i, 3);
			}

			for (int i = 0; i < 3; ++i)
			{
				int j = 0;
				for (j = 0; j < 3; ++j)
				{
					if (fabsf(A(i, j)) >= 1e-6f)
					{
						break;
					}
				}
				if (j == 3) return; //no solution
				for (int p = 0; p < 3; ++p)
				{
					if (p != i) {
						float d = A(p, j) / A(i, j);
						for (int k = 0; k < 4; ++k)
						{
							A(p, k) = A(p, k) - A(i, k) * d;
						}
					}
				}
			}

			for (int i = 0; i < 3; ++i)
			{
				int count = 0;
				for (int j = 0; j < 3; ++j)
				{
					if (fabsf(A(i, j)) < 1e-6f) count++;
				}
				if (count == 3) return; //no solution
			}
			float index[3] = { 0 };
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					if (fabsf(A(i, j)) > 1e-6f)
					{
						index[j] = A(i, 3) / A(i, j);
					}
				}
			}
			optPos.x = index[0];
			optPos.y = index[1];
			optPos.z = index[2];
		}

		void updateCost(const std::vector<Vertex>& vertices)
		{
			Vector4 y(optPos.x, optPos.y, optPos.z, 1.0);
			Matrix4 A = vertices[v[0]].Q + vertices[v[1]].Q;
			Vector4 Ay = A * y;
			cost = y.Dot(Ay);
			cost1 = (vertices[v[0]].pos - vertices[v[1]].pos).SquareLength();
		}
	};

	bool Vertex::HasEdge(int index, const std::vector<Edge>& pairs) const
	{
		for (size_t i = 0; i < this->m_pairs.size(); ++i)
		{
			if (((pairs[index].v[0] == pairs[this->m_pairs[i]].v[0]) && (pairs[index].v[1] == pairs[this->m_pairs[i]].v[1]))
				|| ((pairs[index].v[0] == pairs[this->m_pairs[i]].v[1]) && (pairs[index].v[1] == pairs[this->m_pairs[i]].v[0])))
			{
				return true;
			}
		}
		return false;
	}

	bool Vertex::HasEdge(const Edge& pair, const std::vector<Edge>& pairs) const
	{
		for (size_t i = 0; i < this->m_pairs.size(); ++i)
		{
			if (((pair.v[0] == pairs[this->m_pairs[i]].v[0]) && (pair.v[1] == pairs[this->m_pairs[i]].v[1]))
				|| ((pair.v[0] == pairs[this->m_pairs[i]].v[1]) && (pair.v[1] == pairs[this->m_pairs[i]].v[0])))
			{
				return true;
			}
		}
		return false;
	}

	class Kdtree
	{
	private:
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
		Vertex* m_vb;
		int m_offset;
		int m_size;

	public:
		int m_root;

		Kdtree()
		{
			m_offset = 0;
			m_root = 0;
			m_vb = nullptr;
			m_size = -1;
		}

		~Kdtree()
		{
		}

		int buildLayer(int* indices, int l, int r, int dim, float t)
		{
			if (l >= r) return 0;
			int mid = (l + r) >> 1;
			std::nth_element(indices + l, indices + mid, indices + r, VertexCmp(dim, m_vb));
			int tIndex = m_offset;
			++m_offset;
			int midIndex = indices[mid];
			m_nodes[tIndex].index = midIndex;
			m_nodes[tIndex].dim = dim;
			m_nodes[tIndex].maxBound = m_vb[midIndex].pos + t;
			m_nodes[tIndex].minBound = m_vb[midIndex].pos - t;
			m_nodes[tIndex].left = buildLayer(indices, l, mid, (dim + 1) % 3, t);
			if (m_nodes[tIndex].left) {
				m_nodes[tIndex].maxBound = ElementMax(m_nodes[m_nodes[tIndex].left].maxBound, m_nodes[tIndex].maxBound);
				m_nodes[tIndex].minBound = ElementMin(m_nodes[m_nodes[tIndex].left].minBound, m_nodes[tIndex].minBound);
			}
			m_nodes[tIndex].right = buildLayer(indices, mid + 1, r, (dim + 1) % 3, t);
			if (m_nodes[tIndex].right) {
				m_nodes[tIndex].maxBound = ElementMax(m_nodes[m_nodes[tIndex].right].maxBound, m_nodes[tIndex].maxBound);
				m_nodes[tIndex].minBound = ElementMin(m_nodes[m_nodes[tIndex].right].minBound, m_nodes[tIndex].minBound);
			}
			return tIndex;
		}

		void buildTree(std::vector<Vertex>& vB, int vpNum, float t)
		{
			m_vb = vB.data();
			m_size = vpNum;
			std::vector<int> indices(vpNum);
			for (int i = 0; i < vpNum; ++i)
			{
				indices[i] = i;
			}
			m_nodes.resize(vpNum);
			m_root = buildLayer(indices.data(), 0, m_size, 0, t);
		}

		void query(int node, const Vector3& pos, std::vector<int>& v_hit, float t) const {
			if (pos.x > m_nodes[node].maxBound.x || pos.x < m_nodes[node].minBound.x ||
				pos.y > m_nodes[node].maxBound.y || pos.y < m_nodes[node].minBound.y ||
				pos.z > m_nodes[node].maxBound.z || pos.z < m_nodes[node].minBound.z)
			{
				return;
			}
			int vIndex = m_nodes[node].index;

			if ((m_vb[vIndex].pos - pos).SquareLength() <= t * t) {
				v_hit.push_back(vIndex);
			}

			if (m_nodes[node].left)
				query(m_nodes[node].left, pos, v_hit, t);

			if (m_nodes[node].right)
				query(m_nodes[node].right, pos, v_hit, t);
		}

		void clear() {
			m_offset = 0;
			m_root = 0;
		}
	};


	class Face 
	{
	public:
		int indices[3];
		Face()
		{
			indices[0] = indices[1] = indices[2] = 0;
		}

		Face(int v0, int v1, int v2) {
			indices[0] = v0;
			indices[1] = v1;
			indices[2] = v2;
		}

		Face(const Face& face) {
			indices[0] = face.indices[0];
			indices[1] = face.indices[1];
			indices[2] = face.indices[2];
		}

		Face(const int _indices[]) {
			indices[0] = _indices[0];
			indices[1] = _indices[1];
			indices[2] = _indices[2];
		}

		Face& operator=(const Face& face) {
			if (this == &face)
				return *this;
			indices[0] = face.indices[0];
			indices[1] = face.indices[1];
			indices[2] = face.indices[2];
			return *this;
		}

		Vector3 norm(const std::vector<Vertex>& vertices) const {
			Vector3 v0 = vertices[indices[0]].pos;
			Vector3 v1 = vertices[indices[1]].pos;
			Vector3 v2 = vertices[indices[2]].pos;

			return (v1 - v0).Cross(v2 - v0).SafeUnit();
		}

		friend bool operator==(const Face& face1, const Face& face2)
		{
			bool b0 = (face1.indices[0] == face2.indices[0]) && (face1.indices[1] == face2.indices[1]) && (face1.indices[2] == face2.indices[2]);
			bool b1 = (face1.indices[0] == face2.indices[0]) && (face1.indices[1] == face2.indices[2]) && (face1.indices[2] == face2.indices[1]);
			bool b2 = (face1.indices[0] == face2.indices[1]) && (face1.indices[1] == face2.indices[0]) && (face1.indices[2] == face2.indices[2]);
			bool b3 = (face1.indices[0] == face2.indices[1]) && (face1.indices[1] == face2.indices[2]) && (face1.indices[2] == face2.indices[0]);
			bool b4 = (face1.indices[0] == face2.indices[2]) && (face1.indices[1] == face2.indices[0]) && (face1.indices[2] == face2.indices[1]);
			bool b5 = (face1.indices[0] == face2.indices[2]) && (face1.indices[1] == face2.indices[1]) && (face1.indices[2] == face2.indices[0]);
			return b0 || b1 || b2 || b3 || b4 || b5;
		}
	};

	class FaceMap
	{
	private:

		struct FaceHash
		{
			std::size_t operator()(const Face& face) const
			{
				const size_t h = 0xd8163841; // here arbitrarialy chosen primes
				size_t a1 = face.indices[0] + face.indices[1] + face.indices[2];
				size_t a2 = face.indices[0] * face.indices[1] + face.indices[1] * face.indices[2] + face.indices[2] * face.indices[0];
				size_t a3 = face.indices[0] * face.indices[1] * face.indices[2];
				return (a1 * h + a2) * h + a3;
			}
		};

		std::unordered_map<Face, Face, FaceHash> m_table;

	public:
		void insert(const Face& face)
		{
			m_table[face] = face;
		}

		bool get(const Face& face, Face& realFace)
		{
			auto it = m_table.find(face);
			if (it == m_table.end())
			{
				return false;
			}
			realFace = it->second;
			return true;
		}

		bool remove(const Face& face)
		{
			auto it = m_table.find(face);
			if (it == m_table.end())
			{
				return false;
			}
			m_table.erase(it);
			return true;
		}
	};

	class MeshQEMOptimizer {
	public:
		std::vector<Vertex> m_vertices;
		std::vector<Face> m_faces;
		std::vector<Edge> m_edges;
		FaceMap m_faceMap;
		PriorityPool<Edge> m_heap;
		Kdtree m_tree;
		int m_f_offset;
		int m_edgeCount;
		int m_faceCount;
		int m_vertexCount;
		int m_vertexCountNew;
		std::vector<bool> m_inNewMesh;
		std::vector<bool> m_inEdge;

	public:
		MeshQEMOptimizer()
		{
			m_f_offset = 0;
			m_edgeCount = 0;
			m_faceCount = 0;
			m_vertexCount = 0;
			m_vertexCountNew = 0;
		}

		~MeshQEMOptimizer()
		{
		}

		void GetNewVertices(std::vector<Vector3>& _vertices, std::vector<int>& _indices)
		{
			_vertices.clear();
			_indices.clear();

			int vNum = 0;

			for (int index = 0; index < m_vertexCount; ++index)
			{
				if (!m_inNewMesh[index])
				{
					continue;
				}
				m_vertices[index].newIndex = vNum;
				_vertices.push_back(m_vertices[index].pos);
				++vNum;
			}

			std::vector<bool> inFace(m_vertexCount, false);

			assert(vNum == m_vertexCountNew);


			for (int index = 0; index < m_vertexCount; ++index)
			{
				if (!m_inNewMesh[index])
				{
					continue;
				}
				for (size_t i = 0; i < m_vertices[index].m_neighbors.size(); ++i)
				{
					int neiIndex1 = m_vertices[index].m_neighbors[i];
					for (size_t j = i + 1; j < m_vertices[index].m_neighbors.size(); ++j)
					{
						int neiIndex2 = m_vertices[index].m_neighbors[j];
						if (!inFace[neiIndex1] && !inFace[neiIndex2])
						{
							if (m_vertices[neiIndex1].IsNeighbor(neiIndex2))
							{
								Face realFace;
								int b = m_faceMap.get(Face(index, neiIndex1, neiIndex2), realFace);
								if (b)
								{
									_indices.push_back(m_vertices[realFace.indices[0]].newIndex);
									_indices.push_back(m_vertices[realFace.indices[1]].newIndex);
									_indices.push_back(m_vertices[realFace.indices[2]].newIndex);
								}
							}
						}
					}
				}
				inFace[index] = true;
			}

			return;
		}

		int AddVertex(const Vector3& p) {
			int index = m_vertexCount;
			m_vertices.emplace_back(p);
			m_inNewMesh.push_back(true);
			m_inEdge.push_back(false);
			++m_vertexCount;
			++m_vertexCountNew;
			return index;
		}

		void RemoveVertex(int index) {
			if (m_inNewMesh[index] != true)
			{
				assert(false);
			}
			m_inNewMesh[index] = false;
			--m_vertexCountNew;
		}

		void AddFace(const Face& f)
		{
			for (int i = 0; i < 3; ++i)
			{
				for (int j = i + 1; j < 3; ++j)
				{
					if (!m_vertices[f.indices[i]].IsNeighbor(f.indices[j]))
					{
						m_vertices[f.indices[i]].AddNeighbor(f.indices[j]);
						m_vertices[f.indices[j]].AddNeighbor(f.indices[i]);
					}
				}
			}
			m_faceMap.insert(f);
			m_faces.push_back(f);
			++m_f_offset;
			++m_faceCount;
		}

		int AddEdge(int v1, int v2)
		{
			int pair_index = m_edgeCount;
			Edge e;
			e.v[0] = v1;
			e.v[1] = v2;
			e.index = pair_index;
			m_edges.push_back(e);
			m_vertices[v1].AddEdge(pair_index);
			m_vertices[v2].AddEdge(pair_index);
			++m_edgeCount;
			return pair_index;
		}

		void ComputeQEM()
		{
			for (int index = 0; index < m_vertexCount; ++index)
			{
				m_vertices[index].ComputeQEM(m_vertices);
			}
		}

		void ComputeValidPairs(float t)
		{
			m_tree.buildTree(m_vertices, m_vertexCount, t);
			for (int index = 0; index < m_vertexCount; ++index)
			{
				for (size_t i = 0; i < m_vertices[index].m_neighbors.size(); ++i)
				{
					int neighborIndex = m_vertices[index].m_neighbors[i];
					if (!m_inEdge[neighborIndex])
					{
						int pairIndex = AddEdge(index, neighborIndex);
						m_edges[pairIndex].UpdateOptimalPos(m_vertices);
						m_edges[pairIndex].updateCost(m_vertices);
					}
				}
				std::vector<int> v_hit;
				m_tree.query(m_tree.m_root, m_vertices[index].pos, v_hit, t);
				for (size_t k = 0; k < v_hit.size(); ++k)
				{
					if ((v_hit[k] != index) && !m_inEdge[v_hit[k]] && !m_vertices[index].HasEdge(Edge(index, v_hit[k]), m_edges))
					{
						int pairIndex = AddEdge(index, v_hit[k]);
						m_edges[pairIndex].UpdateOptimalPos(m_vertices);
						m_edges[pairIndex].updateCost(m_vertices);
					}
				}
				m_inEdge[index] = true;
			}
		}

		void BuildHeap(std::vector<Edge>& pairs, int n)
		{
			for (int i = 0; i < n; ++i)
			{
				m_heap.push(&pairs[i]);
			}
		}

		bool Simplify(float rate, float t)
		{
			if (m_vertexCount <= 4 || m_faceCount <= 4)
			{
				return false;
			}

			ComputeQEM();
			ComputeValidPairs(t);
			BuildHeap(m_edges, m_edgeCount);

			int newCount = m_faceCount;
			int iter = 0;

			while (newCount > (int)(m_faceCount * rate))
			{
				if (m_heap.empty())
				{
					break;
				}
				Edge* minPair = m_heap.top();
				m_heap.pop();
				bool succ = Update(*minPair);
				if (succ)
				{
					newCount -= 2;
				}
				iter++;
			}

			return true;
		}

		bool Update(const Edge& pair)
		{
			Vector3 newPos = pair.OptimalPos();
			for (size_t i = 0; i < m_vertices[pair.v[0]].m_neighbors.size(); ++i)
			{
				for (size_t j = i + 1; j < m_vertices[pair.v[0]].m_neighbors.size(); ++j)
				{
					int neiIndex1 = m_vertices[pair.v[0]].m_neighbors[i];
					int neiIndex2 = m_vertices[pair.v[0]].m_neighbors[j];
					Face realFace;
					int succ = m_faceMap.get(Face(pair.v[0], neiIndex1, neiIndex2), realFace);
					if (succ)
					{
						Vector3 originNorm = realFace.norm(m_vertices);
						Vector3 p0 = m_vertices[realFace.indices[0]].pos;
						Vector3 p1 = m_vertices[realFace.indices[1]].pos;
						Vector3 p2 = m_vertices[realFace.indices[2]].pos;
						if (realFace.indices[0] == pair.v[0]) p0 = newPos;
						else if (realFace.indices[1] == pair.v[0]) p1 = newPos;
						else p2 = newPos;
						Vector3 newNorm = (p1 - p0).Cross(p2 - p0).SafeUnit();
						if (originNorm.Dot(newNorm) < INVERSE_LIMIT)
						{
							m_vertices[pair.v[0]].RemoveEdge(pair.index);
							m_vertices[pair.v[1]].RemoveEdge(pair.index);
							return false;
						}
					}
				}
			}

			int newIndex = pair.v[0];
			Vector3 originPos = m_vertices[newIndex].pos;
			m_vertices[newIndex].pos = newPos;

			std::vector<Face> realFaceV;
			std::vector<Face> newFaceV;
			for (size_t i = 0; i < m_vertices[pair.v[1]].m_neighbors.size(); ++i)
			{
				for (size_t j = i + 1; j < m_vertices[pair.v[1]].m_neighbors.size(); ++j)
				{
					int neiIndex1 = m_vertices[pair.v[1]].m_neighbors[i];
					int neiIndex2 = m_vertices[pair.v[1]].m_neighbors[j];
					Face realFace;
					int succ = m_faceMap.get(Face(pair.v[1], neiIndex1, neiIndex2), realFace);
					if (succ)
					{
						Face newFace = realFace;
						if (realFace.indices[0] == pair.v[1]) newFace.indices[0] = pair.v[0];
						else if (realFace.indices[1] == pair.v[1]) newFace.indices[1] = pair.v[0];
						else if (realFace.indices[2] == pair.v[1]) newFace.indices[2] = pair.v[0];
						else assert(0 == 1);
						Vector3 n0 = realFace.norm(m_vertices);
						Vector3 n = newFace.norm(m_vertices);
						if (n.Dot(n0) > INVERSE_LIMIT)
						{
							realFaceV.push_back(realFace);
							newFaceV.push_back(newFace);
						}
						else
						{
							m_vertices[pair.v[0]].pos = originPos;
							m_vertices[pair.v[0]].RemoveEdge(pair.index);
							m_vertices[pair.v[1]].RemoveEdge(pair.index);
							return false;
						}
					}
				}
			}
			for (size_t i = 0; i < realFaceV.size(); ++i)
			{
				bool bb = m_faceMap.remove(realFaceV[i]);
				assert(bb);
				(void)bb;
				m_faceMap.insert(newFaceV[i]);
			}

			for (size_t i = 0; i < m_vertices[pair.v[1]].m_neighbors.size(); ++i)
			{
				int neighborIndex = m_vertices[pair.v[1]].m_neighbors[i];
				if (neighborIndex != pair.v[0])
				{
					if (!m_vertices[newIndex].IsNeighbor(neighborIndex))
					{
						m_vertices[newIndex].AddNeighbor(neighborIndex);
						m_vertices[neighborIndex].AddNeighbor(newIndex);
					}
					m_vertices[neighborIndex].RemoveNeighbor(pair.v[1]);
				}
			}
			m_vertices[newIndex].RemoveNeighbor(pair.v[1]);

			//add v[1] pairs to new vertex(v[0])
			for (size_t i = 0; i < m_vertices[pair.v[1]].m_pairs.size(); ++i)
			{
				int pairIndex = m_vertices[pair.v[1]].m_pairs[i];
				if (m_edges[pairIndex].v[0] == pair.v[1])
				{
					if (m_edges[pairIndex].v[1] == pair.v[0])
					{
						//pair between v[0] and v[1]
						assert(pairIndex == pair.index);
						m_vertices[newIndex].RemoveEdge(pairIndex);
						continue;
					}
					else
					{
						m_edges[pairIndex].v[0] = newIndex;
					}
				}
				else
				{
					assert(m_edges[pairIndex].v[1] == pair.v[1]);
					if (m_edges[pairIndex].v[0] == pair.v[0])
					{
						//pair between v[0] and v[1]
						assert(pairIndex == pair.index);
						m_vertices[newIndex].RemoveEdge(pairIndex);
						continue;
					}
					else {
						m_edges[pairIndex].v[1] = newIndex;
					}
				}

				if (m_vertices[newIndex].HasEdge(pairIndex, m_edges))
				{
					m_heap.remove(&m_edges[pairIndex]);
					if (m_edges[pairIndex].v[0] == pair.v[0])
					{
						m_vertices[m_edges[pairIndex].v[1]].RemoveEdge(pairIndex);
					}
					else
					{
						m_vertices[m_edges[pairIndex].v[0]].RemoveEdge(pairIndex);
					}
				}
				else
				{
					m_vertices[newIndex].AddEdge(pairIndex);
				}
			}

			//update cost & optimal pos
			m_vertices[newIndex].Q += m_vertices[pair.v[1]].Q;
			for (size_t i = 0; i < m_vertices[newIndex].m_pairs.size(); ++i)
			{
				int pairIndex = m_vertices[newIndex].m_pairs[i];
				m_edges[pairIndex].UpdateOptimalPos(m_vertices);
				m_edges[pairIndex].updateCost(m_vertices);
				m_heap.update(&m_edges[pairIndex]);
			}
			RemoveVertex(pair.v[1]);
			return true;
		}
	};

	bool SimplifyMesh(const Vector3* pv, const void* pi, int nv, int nt, bool is16bit, float rate, std::vector<Vector3>& new_v, std::vector<int>& new_i)
	{
		MeshQEMOptimizer s;

		for (int i = 0; i < nv; ++i)
		{
			s.AddVertex(pv[i]);
		}

		const uint16_t* pi16 = is16bit ? static_cast<const uint16_t*>(pi) : nullptr;
		const uint32_t* pi32 = is16bit ? nullptr : static_cast<const uint32_t*>(pi);

		for (int i = 0; i < nt; ++i)
		{
			int indices[3];
			if (pi16)
			{
				indices[0] = (int)pi16[i * 3 + 0];
				indices[1] = (int)pi16[i * 3 + 1];
				indices[2] = (int)pi16[i * 3 + 2];
			}
			else
			{
				indices[0] = (int)pi32[i * 3 + 0];
				indices[1] = (int)pi32[i * 3 + 1];
				indices[2] = (int)pi32[i * 3 + 2];
			}

			Face face(indices);
			s.AddFace(face);
		}

		if (!s.Simplify(rate, 0.001f))
		{
			return false;
		}

		s.GetNewVertices(new_v, new_i);
		return true;

	}

}