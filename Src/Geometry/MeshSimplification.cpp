
#include "../Maths/Vector3.h"
#include "../Maths/Vector4.h"
#include "../Maths/Matrix4.h"
#include "../CollisionPrimitive/StaticMesh.h"
#include "MeshSimplification.h"

#include <assert.h>
#include <stdio.h>
#include <string>

namespace Riemann
{
	const int MAX_VERTEX = 5000;
	const int MAX_PAIR = 50000;
	const int MAX_HEAP_NODE = 50000;
	const int MAX_FACE = 3 * MAX_VERTEX;
	const int P = 131313;
	const int EVAL_COUNT = 100;
	const float INVERSE_LIMIT = -0.1f;

	inline Vector3 ElementMax(const Vector3& v1, const Vector3& v2) { return Vector3(std::max(v1.x, v2.x), std::max(v1.y, v2.y), std::max(v1.z, v2.z)); }
	inline Vector3 ElementMin(const Vector3& v1, const Vector3& v2) { return Vector3(std::min(v1.x, v2.x), std::min(v1.y, v2.y), std::min(v1.z, v2.z)); }

	Matrix4 dp_matrix(const Vector4& v)
	{
		Matrix4 dp;
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j)
				dp[i][j] = v[i] * v[j];
		return dp;
	}

	class Pair;
	class Vertex {
	public:
		Vector3 p;
		Matrix4 Q;
		int newIndex;

		std::vector<int> m_neighbors;
		std::vector<int> m_pairs;

		Vertex(float x = 0, float y = 0, float z = 0) : p(x, y, z)
		{
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

		void ComputeQEM(const std::vector<Vertex>& vertices)
		{
			for (size_t i = 0; i < m_neighbors.size(); ++i)
			for (size_t j = i + 1; j < m_neighbors.size(); ++j)
			{
				if (vertices[m_neighbors[i]].IsNeighbor(m_neighbors[j]))
				{
					Vector3 norm = (vertices[m_neighbors[i]].p - p).Cross(vertices[m_neighbors[j]].p - p).SafeUnit();
					float w = -(p.Dot(norm));
					Vector4 v4(norm, w);
					Q += dp_matrix(v4);
				}
			}
		}
		void SetPos(const Vector3& pos)
		{
			p = pos;
		}

		bool HasPair(int index, const std::vector<Pair>& pairs) const;
		bool HasPair(const Pair& pair, const std::vector<Pair>& pairs) const;

		void AddPair(int index)
		{
			m_pairs.push_back(index);
		}

		void RemovePair(int index)
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
				return vB[vp1].p.x < vB[vp2].p.x;
			else if (D == 1)
				return vB[vp1].p.y < vB[vp2].p.y;
			else
				return vB[vp1].p.z < vB[vp2].p.z;
		}
	};

	struct Node
	{
		int index;
		int left;
		int right;
		Vector3 minBound;
		Vector3 maxBound;
		int dim;
		Node() {
			index = 0;
			left = 0;
			right = 0;
			minBound = Vector3::Zero();
			maxBound = Vector3::Zero();
			dim = 0;
		}
	};

	class Pair
	{
	private:
		Vector3 optPos;
	public:
		float cost;
		float cost1;
		int heapIndex;
		int index;
		int v[2];

		Pair() {
			cost = 0.0;
			optPos = Vector3::Zero();
			heapIndex = 0;
			index = 0;
			v[0] = v[1] = 0;
		}

		Pair(int v0, int v1) {
			cost = 0.0;
			optPos = Vector3::Zero();
			heapIndex = 0;
			index = 0;
			v[0] = v0;
			v[1] = v1;
		}

		Vector3 OptimalPos() const {
			return optPos;
		}

		void UpdateOptimalPos(const std::vector<Vertex>& vertices)
		{
			optPos = (vertices[v[0]].p + vertices[v[1]].p) / 2; //if no solution, choose the middle
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
			Vector4 y(optPos, 1.0);
			Matrix4 A = vertices[v[0]].Q + vertices[v[1]].Q;
			Vector4 Ay = A * y;
			cost = y.Dot(Ay);
			cost1 = (vertices[v[0]].p - vertices[v[1]].p).SquareLength();
		}

		friend bool operator< (const Pair& p1, const Pair& p2)
		{
			return p1.cost < p2.cost;
		}
	};

	bool Vertex::HasPair(int index, const std::vector<Pair>& pairs) const
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

	bool Vertex::HasPair(const Pair& pair, const std::vector<Pair>& pairs) const
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


	struct HeapNode
	{
		int index;
		float cost;
		float cost1;
		bool exist;
		friend bool operator< (const HeapNode& n1, const HeapNode& n2) {
			return (n1.cost < n2.cost) || ((fabsf(n1.cost - n2.cost) < 1e-5) && (n1.cost1 < n2.cost1));
		}
		HeapNode() : index(0), exist(0) {}
	};

	class Heap
	{
	private:
		Pair* m_pairs;
		HeapNode* m_data;
		int m_size;
		int m_num_pairs;

		int Down(int k)
		{
			int i = k, j = k;
			while (true) {
				if (((i << 1) + 1 < m_size) && (m_data[(i << 1) + 1] < m_data[j])) {
					j = (i << 1) + 1;
				}
				if (((i << 1) + 2 < m_size) && (m_data[(i << 1) + 2] < m_data[j])) {
					j = (i << 1) + 2;
				}
				if (i == j) {
					return i;
				}
				else {
					HeapNode temp = m_data[i]; m_data[i] = m_data[j]; m_data[j] = temp;
					if (m_data[i].exist) {
						assert(m_pairs[m_data[i].index].heapIndex == j);
						m_pairs[m_data[i].index].heapIndex = i;
					}
					if (m_data[j].exist) {
						if (m_pairs[m_data[j].index].heapIndex != i)
						{
							// TODO, log
						}
						assert(m_pairs[m_data[j].index].heapIndex == i);
						m_pairs[m_data[j].index].heapIndex = j;
					}
					i = j;
				}
			}
			return i;
		}

		int Up(int k) {
			int i = k;
			while (i > 0) {
				int j = (i - 1) >> 1;
				if (m_data[i] < m_data[j]) {
					HeapNode temp = m_data[i]; m_data[i] = m_data[j]; m_data[j] = temp;
					if (m_data[i].exist) {
						assert(m_pairs[m_data[i].index].heapIndex == j);
						m_pairs[m_data[i].index].heapIndex = i;
					}
					if (m_data[j].exist) {
						assert(m_pairs[m_data[j].index].heapIndex == i);
						m_pairs[m_data[j].index].heapIndex = j;
					}
					i = j;
				}
				else {
					return i;
				}
			}
			return i;
		}

	public:
		Heap()
		{
			m_pairs = nullptr;
			m_size = 0;
			m_num_pairs = 0;
			m_data = new HeapNode[MAX_HEAP_NODE];
		}

		~Heap()
		{
			delete[] m_data;
		}

		void Build(std::vector<Pair>& pairs, int n)
		{
			m_pairs = pairs.data();
			m_size = n;
			m_num_pairs = n;
			for (int i = 0; i < n; ++i) {
				m_data[i].cost = pairs[i].cost;
				m_data[i].cost1 = pairs[i].cost1;
				m_data[i].index = i;
				m_data[i].exist = true;
			}
			for (int i = 0; i < n; ++i) {
				pairs[m_data[i].index].heapIndex = i;
			}
			for (int i = (n - 2) >> 1; i >= 0; --i) {
				Down(i);
			}
		}

		int Add(Pair& pair)
		{
			assert(0 < m_size);
			assert(m_size < MAX_HEAP_NODE);
			m_data[m_size].cost = pair.cost;
			m_data[m_size].cost1 = pair.cost1;
			m_data[m_size].index = pair.index;
			m_data[m_size].exist = true;
			pair.heapIndex = m_size;
			++m_size;
			int temp = Up(m_size - 1);

			return temp;
		}

		void Remove()
		{
			assert(m_size - 1 < MAX_HEAP_NODE);
			if (m_data[0].exist) {
				--m_num_pairs;
			}
			m_data[0] = m_data[--m_size];
			if (m_data[0].exist) {
				assert(m_pairs[m_data[0].index].heapIndex == m_size);
				m_pairs[m_data[0].index].heapIndex = 0;
			}
			Down(0);
		}

		void Remove(const Pair& pair)
		{
			assert(m_data[pair.heapIndex].exist == true);
			m_data[pair.heapIndex].exist = false;
			--m_num_pairs;
		}

		void Update(Pair& pair)
		{
			m_data[pair.heapIndex].exist = false;
			Add(pair);
		}

		int Top()
		{
			while (0 < m_size && !m_data[0].exist) {
				Remove();
			}
			if (0 < m_size) {
				return m_data[0].index;
			}
			return -1;
		}
	};


	class Kdtree {
	private:
		Node* m_nodes;
		Vertex* m_vb;
		int m_offset;
		int m_size;
	public:
		int m_root;

		Kdtree()
		{
			m_offset = 1; //0 for null
			m_root = 0;
			m_nodes = new Node[MAX_VERTEX];
			m_vb = nullptr;
			m_size = -1;
		}

		~Kdtree()
		{
			delete[] m_nodes;
		}

		int buildLayer(int* indices, int l, int r, int dim, float t) {
			if (l >= r) return 0;
			int mid = (l + r) >> 1;
			std::nth_element(indices + l, indices + mid, indices + r, VertexCmp(dim, m_vb));
			int tIndex = m_offset;
			++m_offset;
			int midIndex = indices[mid];
			m_nodes[tIndex].index = midIndex;
			m_nodes[tIndex].dim = dim;
			m_nodes[tIndex].maxBound = m_vb[midIndex].p + t;
			m_nodes[tIndex].minBound = m_vb[midIndex].p - t;
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

		void buildTree(std::vector<Vertex>& vB, int vpNum, float t) {
			m_vb = vB.data();
			m_size = vpNum;
			int* indices = new int[MAX_VERTEX];
			for (int i = 0; i < vpNum; ++i) {
				indices[i] = i;
			}
			m_root = buildLayer(indices, 0, m_size, 0, t);
			delete[] indices;
		}

		void query(int node, const Vector3& pos, std::vector<int>& v_hit, float t) const {
			if (pos.x > m_nodes[node].maxBound.x || pos.x < m_nodes[node].minBound.x ||
				pos.y > m_nodes[node].maxBound.y || pos.y < m_nodes[node].minBound.y ||
				pos.z > m_nodes[node].maxBound.z || pos.z < m_nodes[node].minBound.z)
			{
				return;
			}
			int vIndex = m_nodes[node].index;

			if ((m_vb[vIndex].p - pos).SquareLength() <= t * t) {
				v_hit.push_back(vIndex);
			}

			if (m_nodes[node].left)
				query(m_nodes[node].left, pos, v_hit, t);

			if (m_nodes[node].right)
				query(m_nodes[node].right, pos, v_hit, t);
		}

		void clear() {
			m_offset = 1;
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
			Vector3 v0 = vertices[indices[0]].p;
			Vector3 v1 = vertices[indices[1]].p;
			Vector3 v2 = vertices[indices[2]].p;

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

		bool inFace(const Vector3& p, const std::vector<Vertex>& vertices) const {
			//using areas of trangles
			Vector3 v0 = vertices[indices[0]].p;
			Vector3 v1 = vertices[indices[1]].p;
			Vector3 v2 = vertices[indices[2]].p;

			float A0 = (v1 - v0).Cross(v2 - v0).Length();
			float S0 = (v1 - p).Cross(v2 - p).Length();
			float S1 = (v2 - p).Cross(v0 - p).Length();
			float S2 = (v0 - p).Cross(v1 - p).Length();
			if (S0 + S1 + S2 > A0) {
				return false;
			}
			else {
				return true;
			}
		}

		float distance(const Vector3& p, const std::vector<Vertex>& vertices) const
		{
			Vector3 n = norm(vertices);
			float dist_p2f = n.Dot(vertices[indices[0]].p - p);
			Vector3 p_on_f = p + dist_p2f * n;
			if (inFace(p_on_f, vertices))
			{
				return fabsf(dist_p2f);
			}
			else
			{
				float d = FLT_MAX;
				for (int i = 0; i < 3; ++i)
				{
					float new_d = (vertices[indices[i]].p - p).Length();
					if (new_d < d) {
						d = new_d;
					}
				}
				return d;
			}
		}
	};

	class Map
	{
	private:
		std::vector<Face> table;
		std::vector<bool> fill;
		std::vector<bool> lazy;

	public:
		Map()
		{
			table.resize(2 * MAX_FACE);
			memset((void*)table.data(), 0, MAX_FACE * sizeof(Face));
			fill.resize(2 * MAX_FACE, false);
			lazy.resize(2 * MAX_FACE, false);
		}

		unsigned int hash(const Face& face)
		{
			unsigned long long int a1 = face.indices[0] + face.indices[1] + face.indices[2];
			unsigned long long int a2 = face.indices[0] * face.indices[1] + face.indices[1] * face.indices[2] + face.indices[2] * face.indices[0];
			unsigned long long int a3 = face.indices[0] * face.indices[1] * face.indices[2];
			unsigned long long int temp = (a1 * P + a2) * P + a3;
			return (unsigned int)(temp % MAX_FACE);
		}

		void insert(const Face& face)
		{
			unsigned int hashCode = hash(face);
			unsigned int step = 0, position = hashCode;
			while (fill[position]) {
				step++;
				position = (hashCode + step * step) % MAX_FACE;
			}
			table[position] = face;
			fill[position] = true;
		}

		bool get(const Face& face, Face& realFace)
		{
			unsigned int hashCode = hash(face);
			unsigned int step = 0, position = hashCode;
			while ((fill[position] && (!(table[position] == face))) || (!fill[position] && lazy[position]))
			{
				step++;
				position = (hashCode + step * step) % MAX_FACE;
			}
			if (fill[position]) {
				realFace = table[position];
				return true;
			}
			return false;
		}

		bool remove(const Face& face)
		{
			unsigned int hashCode = hash(face);
			unsigned int step = 0, position = hashCode;
			while ((fill[position] && (!(table[position] == face))) || (!fill[position] && lazy[position]))
			{
				step++;
				position = (hashCode + step * step) % MAX_FACE;
			}
			if (fill[position]) {
				fill[position] = false;
				lazy[position] = true;
				return true;
			}
			return false;
		}
	};

	class MeshQEMOptimizer {
	public:
		std::vector<Vertex> m_vertices;
		std::vector<Vector3> m_old_vertices;
		std::vector<Face> m_old_face;
		std::vector<Pair> m_pairs;
		Map m_faceMap;
		Heap m_heap;
		Kdtree m_tree;
		int m_fOffset;
		int m_vOffset;
		int m_pOffset;
		int m_faceCount;
		int m_vertexCount;
		std::vector<bool> m_valid;
		std::vector<bool> m_inPair;
		std::vector<bool> m_inFace;

	public:
		MeshQEMOptimizer()
		{
			m_vertices.resize(MAX_VERTEX);
			m_pairs.resize(MAX_PAIR);
			m_old_vertices.resize(MAX_VERTEX);
			m_old_face.resize(MAX_FACE);
			m_vOffset = 1;
			m_pOffset = 0;
			m_fOffset = 0;
			m_faceCount = 0;
			m_vertexCount = 0;
			m_valid.resize(MAX_VERTEX, false);
			m_inPair.resize(MAX_PAIR, false);
			m_inFace.resize(MAX_FACE, false);
		}

		~MeshQEMOptimizer()
		{
		}

		void GetNewVertices(std::vector<Vector3>& _vertices, std::vector<int>& _indices)
		{
			_vertices.clear();
			_indices.clear();

			int vNum = 0;

			for (int index = 1; index < m_vOffset; ++index)
			{
				if (!m_valid[index])
				{
					continue;
				}
				m_vertices[index].newIndex = vNum;
				_vertices.emplace_back(m_vertices[index].p.x, m_vertices[index].p.y, m_vertices[index].p.z);
				++vNum;
			}

			assert(vNum == m_vertexCount);
			for (int index = 1; index < m_vOffset; ++index)
			{
				if (!m_valid[index])
				{
					continue;
				}
				for (size_t i = 0; i < m_vertices[index].m_neighbors.size(); ++i)
				{
					int neiIndex1 = m_vertices[index].m_neighbors[i];
					for (size_t j = i + 1; j < m_vertices[index].m_neighbors.size(); ++j)
					{
						int neiIndex2 = m_vertices[index].m_neighbors[j];
						if (!m_inFace[neiIndex1] && !m_inFace[neiIndex2])
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
				m_inFace[index] = true;
			}
		}

		int AddVertex(const Vector3& p) {
			int index = m_vOffset;
			m_vertices[m_vOffset].SetPos(p);
			m_old_vertices[m_vOffset] = p;
			m_valid[m_vOffset] = true;
			++m_vOffset;
			++m_vertexCount;
			assert(m_vOffset < MAX_VERTEX);
			return index;
		}

		void RemoveVertex(int index) {
			if (m_valid[index] != true)
			{
				// TODO, log		
			}
			assert(m_valid[index] == true);
			m_valid[index] = false;
			--m_vertexCount;
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
			m_old_face[m_fOffset] = f;
			++m_fOffset;
			++m_faceCount;
		}

		int AddPair(int v1, int v2)
		{
			int pairIndex = m_pOffset;
			m_pairs[m_pOffset].v[0] = v1;
			m_pairs[m_pOffset].v[1] = v2;
			m_pairs[m_pOffset].index = pairIndex;
			m_vertices[v1].AddPair(pairIndex);
			m_vertices[v2].AddPair(pairIndex);
			++m_pOffset;
			assert(m_pOffset < MAX_PAIR);
			return pairIndex;
		}

		void ComputeQEM()
		{
			for (int index = 1; index < m_vOffset; ++index)
			{
				m_vertices[index].ComputeQEM(m_vertices);
			}
		}

		void ComputeValidPairs(float t)
		{
			m_tree.buildTree(m_vertices, m_vOffset, t);
			for (int index = 1; index < m_vOffset; ++index)
			{
				for (size_t i = 0; i < m_vertices[index].m_neighbors.size(); ++i)
				{
					int neighborIndex = m_vertices[index].m_neighbors[i];
					if (!m_inPair[neighborIndex])
					{
						int pairIndex = AddPair(index, neighborIndex);
						m_pairs[pairIndex].UpdateOptimalPos(m_vertices);
						m_pairs[pairIndex].updateCost(m_vertices);
					}
				}
				std::vector<int> v_hit;
				m_tree.query(m_tree.m_root, m_vertices[index].p, v_hit, t);
				for (size_t k = 0; k < v_hit.size(); ++k)
				{
					if ((v_hit[k] != index) && !m_inPair[v_hit[k]] && !m_vertices[index].HasPair(Pair(index, v_hit[k]), m_pairs))
					{
						int pairIndex = AddPair(index, v_hit[k]);
						m_pairs[pairIndex].UpdateOptimalPos(m_vertices);
						m_pairs[pairIndex].updateCost(m_vertices);
					}
				}
				m_inPair[index] = true;
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
			m_heap.Build(m_pairs, m_pOffset);

			int newCount = m_faceCount;
			int iter = 0;
			Face temp;
			while ((float)newCount > (float)m_faceCount * rate)
			{
				int pairIndex = m_heap.Top();
				if (pairIndex < 0)
				{
					break;
				}
				Pair minPair = m_pairs[m_heap.Top()];
				m_heap.Remove();
				bool succ = Update(minPair);
				if (succ)
				{
					newCount -= 2;
				}
				iter++;
			}

			return true;
		}

		bool Update(const Pair& pair)
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
						Vector3 p0 = m_vertices[realFace.indices[0]].p;
						Vector3 p1 = m_vertices[realFace.indices[1]].p;
						Vector3 p2 = m_vertices[realFace.indices[2]].p;
						if (realFace.indices[0] == pair.v[0]) p0 = newPos;
						else if (realFace.indices[1] == pair.v[0]) p1 = newPos;
						else p2 = newPos;
						Vector3 newNorm = (p1 - p0).Cross(p2 - p0).SafeUnit();
						if (originNorm.Dot(newNorm) < INVERSE_LIMIT)
						{
							m_vertices[pair.v[0]].RemovePair(pair.index);
							m_vertices[pair.v[1]].RemovePair(pair.index);
							return false;
						}
					}
				}
			}

			int newIndex = pair.v[0];
			Vector3 originPos = m_vertices[newIndex].p;
			m_vertices[newIndex].SetPos(newPos);

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
							m_vertices[pair.v[0]].SetPos(originPos);
							m_vertices[pair.v[0]].RemovePair(pair.index);
							m_vertices[pair.v[1]].RemovePair(pair.index);
							return false;
						}
					}
				}
			}
			for (size_t i = 0; i < realFaceV.size(); ++i)
			{
				int bb = m_faceMap.remove(realFaceV[i]);
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
				if (m_pairs[pairIndex].v[0] == pair.v[1])
				{
					if (m_pairs[pairIndex].v[1] == pair.v[0])
					{
						//pair between v[0] and v[1]
						assert(pairIndex == pair.index);
						m_vertices[newIndex].RemovePair(pairIndex);
						continue;
					}
					else
					{
						m_pairs[pairIndex].v[0] = newIndex;
					}
				}
				else
				{
					assert(m_pairs[pairIndex].v[1] == pair.v[1]);
					if (m_pairs[pairIndex].v[0] == pair.v[0])
					{
						//pair between v[0] and v[1]
						assert(pairIndex == pair.index);
						m_vertices[newIndex].RemovePair(pairIndex);
						continue;
					}
					else {
						m_pairs[pairIndex].v[1] = newIndex;
					}
				}

				if (m_vertices[newIndex].HasPair(pairIndex, m_pairs))
				{
					m_heap.Remove(m_pairs[pairIndex]);
					if (m_pairs[pairIndex].v[0] == pair.v[0])
					{
						m_vertices[m_pairs[pairIndex].v[1]].RemovePair(pairIndex);
					}
					else
					{
						m_vertices[m_pairs[pairIndex].v[0]].RemovePair(pairIndex);
					}
				}
				else
				{
					m_vertices[newIndex].AddPair(pairIndex);
				}
			}

			//update cost & optimal pos
			m_vertices[newIndex].Q += m_vertices[pair.v[1]].Q;
			for (size_t i = 0; i < m_vertices[newIndex].m_pairs.size(); ++i)
			{
				int pairIndex = m_vertices[newIndex].m_pairs[i];
				m_pairs[pairIndex].UpdateOptimalPos(m_vertices);
				m_pairs[pairIndex].updateCost(m_vertices);
				m_heap.Update(m_pairs[pairIndex]);
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
				indices[0] = (int)pi16[i * 3 + 0] + 1;
				indices[1] = (int)pi16[i * 3 + 1] + 1;
				indices[2] = (int)pi16[i * 3 + 2] + 1;
			}
			else
			{
				indices[0] = (int)pi32[i * 3 + 0] + 1;
				indices[1] = (int)pi32[i * 3 + 1] + 1;
				indices[2] = (int)pi32[i * 3 + 2] + 1;
			}

			Face face(indices);
			s.AddFace(face);
		}

		if (!s.Simplify(rate, 0.0f))
		{
			return false;
		}

		s.GetNewVertices(new_v, new_i);
		return true;

	}

}