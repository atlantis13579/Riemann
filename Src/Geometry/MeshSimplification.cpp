
#include "../Maths/Vector3.h"
#include "../Maths/Vector4.h"
#include "../Maths/Matrix4.h"
#include "../Core/PriorityQueue.h"
#include "../Core/LinearSet.h"
#include "../CollisionPrimitive/StaticMesh.h"
#include "MeshSimplification.h"

#include <assert.h>
#include <unordered_map>
#include <string>

// Surface Simplification Using Quadric Error Metrics
// Michael Garland and Paul S. Heckbert

namespace Riemann
{
	const float INVERSE_LIMIT = -0.1f;

	class Edge;
	class Vertex {
	public:
		Vector3 pos;
		Matrix4 Q;
		int newIndex;

		std::vector<int> m_neighbors;
		std::vector<int> m_edges;

		Vertex()
		{
			pos = Vector3::Zero();
			Q = Matrix4::Zero();
			newIndex = 0;
		}

		Vertex(const Vector3 &p)
		{
			pos = p;
			Q = Matrix4::Zero();
			newIndex = 0;
		}

		bool IsNeighbor(int index) const
		{
			for (size_t i = 0; i < m_neighbors.size(); ++i)
			{
				if (m_neighbors[i] == index)
				{
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
				if (m_neighbors[i] == index)
				{
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
			m_edges.push_back(index);
		}

		void RemoveEdge(int index)
		{
			for (size_t i = 0; i < m_edges.size(); ++i)
			{
				if (m_edges[i] == index)
				{
					m_edges[i] = m_edges.back();
					m_edges.pop_back();
					return;
				}
			}
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
		int num_faces;
		int v[2];

		Edge()
		{
			cost = cost1 = 0.0f;
			optPos = Vector3::Zero();
			index = 0;
			num_faces = 0;
			v[0] = v[1] = 0;
		}

		Edge(int v0, int v1)
		{
			cost = cost1 = 0.0f;
			optPos = Vector3::Zero();
			index = 0;
			num_faces = 0;
			v[0] = v0;
			v[1] = v1;
		}

		Vector3 OptimalPos() const
		{
			return optPos;
		}

		bool IsBoundaryEdge() const
		{
			return num_faces <= 1;
		}

		bool IsManifoldEdge() const
		{
			return num_faces == 2;
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

		static bool InInSegment(const Vector3& v, const Vector3& v0, const Vector3& v1)
		{
			const float d0 = (v - v0).Length();
			const float d1 = (v - v1).Length();
			const float d = (v0 - v1).Length();
			return fabsf(d0 + d1 - d) < 1e-5f;
		}

		void ComputeOptimalPos(const std::vector<Vertex>& vertices)
		{
			const Vertex &v0 = vertices[v[0]];
			const Vertex &v1 = vertices[v[1]];

			Matrix4 A = v0.Q + v1.Q;
			A(3, 0) = 0.0f;
			A(3, 1) = 0.0f;
			A(3, 2) = 0.0f;
			A(3, 3) = 1.0f;

			// Solve A * X = Vector4(0, 0, 0, 1)
			if (A.Invertible())
			{
				Vector4 X = A.Inverse() * Vector4(0.0f, 0.0f, 0.0f, 1.0f);
				optPos.x = X.x;
				optPos.y = X.y;
				optPos.z = X.z;

				// numerial error ?
				if (InInSegment(optPos, v0.pos, v1.pos))
				{
					return;
				}
			}
			
			// fallback
			optPos = (v0.pos + v1.pos) * 0.5f;
		}

		void ComputeCost(const std::vector<Vertex>& vertices)
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
		for (size_t i = 0; i < this->m_edges.size(); ++i)
		{
			if (((pairs[index].v[0] == pairs[this->m_edges[i]].v[0]) && (pairs[index].v[1] == pairs[this->m_edges[i]].v[1]))
				|| ((pairs[index].v[0] == pairs[this->m_edges[i]].v[1]) && (pairs[index].v[1] == pairs[this->m_edges[i]].v[0])))
			{
				return true;
			}
		}
		return false;
	}

	bool Vertex::HasEdge(const Edge& pair, const std::vector<Edge>& pairs) const
	{
		for (size_t i = 0; i < this->m_edges.size(); ++i)
		{
			if (((pair.v[0] == pairs[this->m_edges[i]].v[0]) && (pair.v[1] == pairs[this->m_edges[i]].v[1]))
				|| ((pair.v[0] == pairs[this->m_edges[i]].v[1]) && (pair.v[1] == pairs[this->m_edges[i]].v[0])))
			{
				return true;
			}
		}
		return false;
	}

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
		int m_edgeCount;
		int m_vertexCount;
		int m_vertexCountNew;
		std::vector<bool> m_inNewMesh;

	public:
		MeshQEMOptimizer()
		{
			m_edgeCount = 0;
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
					int index_i = f.indices[i];
					int index_j = f.indices[j];
					if (!m_vertices[index_i].IsNeighbor(index_j))
					{
						m_vertices[index_i].AddNeighbor(index_j);
						assert(!m_vertices[index_j].IsNeighbor(index_i));
						m_vertices[index_j].AddNeighbor(index_i);
					}
				}
			}
			m_faceMap.insert(f);
			m_faces.push_back(f);
		}

		int AddEdge(int v1, int v2)
		{
			int index = m_edgeCount;
			Edge e;
			e.v[0] = v1;
			e.v[1] = v2;
			e.index = index;
			m_edges.push_back(e);
			m_vertices[v1].AddEdge(index);
			m_vertices[v2].AddEdge(index);
			++m_edgeCount;
			return index;
		}

		void ComputeQEM()
		{
			for (int index = 0; index < m_vertexCount; ++index)
			{
				m_vertices[index].ComputeQEM(m_vertices);
			}
		}

		void BuildEdges()
		{
			std::vector<bool> processed(m_vertexCount, false);
			for (int i = 0; i < m_vertexCount; ++i)
			{
				for (size_t j = 0; j < m_vertices[i].m_neighbors.size(); ++j)
				{
					int neighborIndex = m_vertices[i].m_neighbors[j];
					if (!processed[neighborIndex])
					{
						int index = AddEdge(i, neighborIndex);
						m_edges[index].ComputeOptimalPos(m_vertices);
						m_edges[index].ComputeCost(m_vertices);
					}
				}

				processed[i] = true;
			}
		}

		void BuildFaceEdges()
		{
			std::vector<int> face_edges;

			for (const Face& face : m_faces)
			{
				LinearSet<int> set, set2;
				set.insert(face.indices[0]);
				set.insert(face.indices[1]);
				set.insert(face.indices[2]);

				for (int vid : face.indices)
				{
					const Vertex &v = m_vertices[vid];
					for (int eid : v.m_edges)
					{
						const Edge &edge = m_edges[eid];
						if (set.contains(edge.v[0]) && set.contains(edge.v[1]))
						{
							set2.insert(eid);
						}
					}
				}

				assert(set2.size() == 3);

				for (int eid : set2)
				{
					face_edges.push_back(eid);
				}

				continue;
			}

			for (Edge edge : m_edges)
			{
				edge.num_faces = 0;
			}

			for (int eid : face_edges)
			{
				m_edges[eid].num_faces += 1;
			}

			return;
		}

		void BuildHeap(std::vector<Edge>& pairs, int n)
		{
			for (int i = 0; i < n; ++i)
			{
				m_heap.push(&pairs[i]);
			}
		}

		bool Simplify(const SimplificationConfig &cfg)
		{
			int faceCount = (int)m_faces.size();
			if (m_vertexCount <= 4 || faceCount <= 4)
			{
				return false;
			}

			ComputeQEM();
			BuildEdges();
			BuildFaceEdges();
			BuildHeap(m_edges, m_edgeCount);

			int newCount = faceCount;
			int expectCount = cfg.faces > 3 ? cfg.faces : (int)(faceCount * cfg.rate);
			int iter = 0;

			while (newCount > expectCount)
			{
				if (m_heap.empty())
				{
					break;
				}

				iter++;
				Edge* min_edge = m_heap.top();
				m_heap.pop();

				if (!min_edge->IsManifoldEdge())
				{
					continue;
				}

				bool succ = EdgeCollapse(*min_edge);
				if (succ)
				{
					newCount -= 2;
				}
			}

			return true;
		}

		bool EdgeCollapse(const Edge& e)
		{
			Vector3 newPos = e.OptimalPos();
			for (size_t i = 0; i < m_vertices[e.v[0]].m_neighbors.size(); ++i)
			{
				for (size_t j = i + 1; j < m_vertices[e.v[0]].m_neighbors.size(); ++j)
				{
					int neiIndex1 = m_vertices[e.v[0]].m_neighbors[i];
					int neiIndex2 = m_vertices[e.v[0]].m_neighbors[j];
					Face realFace;
					int succ = m_faceMap.get(Face(e.v[0], neiIndex1, neiIndex2), realFace);
					if (succ)
					{
						Vector3 originNorm = realFace.norm(m_vertices);
						Vector3 p0 = m_vertices[realFace.indices[0]].pos;
						Vector3 p1 = m_vertices[realFace.indices[1]].pos;
						Vector3 p2 = m_vertices[realFace.indices[2]].pos;
						if (realFace.indices[0] == e.v[0])
							p0 = newPos;
						else if (realFace.indices[1] == e.v[0])
							p1 = newPos;
						else
							p2 = newPos;
						Vector3 newNorm = (p1 - p0).Cross(p2 - p0).SafeUnit();
						if (originNorm.Dot(newNorm) < INVERSE_LIMIT)
						{
							m_vertices[e.v[0]].RemoveEdge(e.index);
							m_vertices[e.v[1]].RemoveEdge(e.index);
							return false;
						}
					}
				}
			}

			int newIndex = e.v[0];
			Vector3 originPos = m_vertices[newIndex].pos;
			m_vertices[newIndex].pos = newPos;

			std::vector<Face> realFaceV;
			std::vector<Face> newFaceV;
			for (size_t i = 0; i < m_vertices[e.v[1]].m_neighbors.size(); ++i)
			{
				for (size_t j = i + 1; j < m_vertices[e.v[1]].m_neighbors.size(); ++j)
				{
					int neiIndex1 = m_vertices[e.v[1]].m_neighbors[i];
					int neiIndex2 = m_vertices[e.v[1]].m_neighbors[j];
					Face realFace;
					int succ = m_faceMap.get(Face(e.v[1], neiIndex1, neiIndex2), realFace);
					if (succ)
					{
						Face newFace = realFace;
						if (realFace.indices[0] == e.v[1])
							newFace.indices[0] = e.v[0];
						else if (realFace.indices[1] == e.v[1])
							newFace.indices[1] = e.v[0];
						else if (realFace.indices[2] == e.v[1])
							newFace.indices[2] = e.v[0];
						else
							assert(false);
						
						Vector3 n0 = realFace.norm(m_vertices);
						Vector3 n = newFace.norm(m_vertices);
						if (n.Dot(n0) > INVERSE_LIMIT)
						{
							realFaceV.push_back(realFace);
							newFaceV.push_back(newFace);
						}
						else
						{
							m_vertices[e.v[0]].pos = originPos;
							m_vertices[e.v[0]].RemoveEdge(e.index);
							m_vertices[e.v[1]].RemoveEdge(e.index);
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

			for (size_t i = 0; i < m_vertices[e.v[1]].m_neighbors.size(); ++i)
			{
				int neighborIndex = m_vertices[e.v[1]].m_neighbors[i];
				if (neighborIndex != e.v[0])
				{
					if (!m_vertices[newIndex].IsNeighbor(neighborIndex))
					{
						m_vertices[newIndex].AddNeighbor(neighborIndex);
						m_vertices[neighborIndex].AddNeighbor(newIndex);
					}
					m_vertices[neighborIndex].RemoveNeighbor(e.v[1]);
				}
			}
			m_vertices[newIndex].RemoveNeighbor(e.v[1]);

			//add v[1] pairs to new vertex(v[0])
			for (size_t i = 0; i < m_vertices[e.v[1]].m_edges.size(); ++i)
			{
				int pairIndex = m_vertices[e.v[1]].m_edges[i];
				if (m_edges[pairIndex].v[0] == e.v[1])
				{
					if (m_edges[pairIndex].v[1] == e.v[0])
					{
						//pair between v[0] and v[1]
						assert(pairIndex == e.index);
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
					assert(m_edges[pairIndex].v[1] == e.v[1]);
					if (m_edges[pairIndex].v[0] == e.v[0])
					{
						//pair between v[0] and v[1]
						assert(pairIndex == e.index);
						m_vertices[newIndex].RemoveEdge(pairIndex);
						continue;
					}
					else
					{
						m_edges[pairIndex].v[1] = newIndex;
					}
				}

				if (m_vertices[newIndex].HasEdge(pairIndex, m_edges))
				{
					m_heap.remove(&m_edges[pairIndex]);
					if (m_edges[pairIndex].v[0] == e.v[0])
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
			m_vertices[newIndex].Q += m_vertices[e.v[1]].Q;
			for (size_t i = 0; i < m_vertices[newIndex].m_edges.size(); ++i)
			{
				int pairIndex = m_vertices[newIndex].m_edges[i];
				m_edges[pairIndex].ComputeOptimalPos(m_vertices);
				m_edges[pairIndex].ComputeCost(m_vertices);
				m_heap.update(&m_edges[pairIndex]);
			}
			RemoveVertex(e.v[1]);
			return true;
		}
	};

	bool SimplifyMesh(const Vector3* pv, const void* pi, int nv, int nt, bool is16bit, const SimplificationConfig& cfg, std::vector<Vector3>& new_v, std::vector<int>& new_i)
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

			assert(indices[0] != indices[1] && indices[0] != indices[1] && indices[1] != indices[2]);

			Face face(indices);
			s.AddFace(face);
		}

		if (!s.Simplify(cfg))
		{
			return false;
		}

		s.GetNewVertices(new_v, new_i);
		return true;

	}

}