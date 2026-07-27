
#include "../Maths/Vector3.h"
#include "../Maths/Vector4.h"
#include "../Maths/Matrix4.h"
#include "../Core/PriorityQueue.h"
#include "../Core/LinearSet.h"
#include "../CollisionPrimitive/StaticMesh.h"
#include "MeshSimplification.h"

#include <assert.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <string>


namespace Riemann
{
	// Surface Simplification Using Quadric Error Metrics, Michael Garland and Paul S. Heckbert
	class GarlandQEM
	{
	private:
		static constexpr float INVERSE_LIMIT = -0.1f;

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

		template <typename EdgeType>
		bool HasEdge(int index, const std::vector<EdgeType>& pairs) const
		{
			for (size_t i = 0; i < m_edges.size(); ++i)
			{
				if (((pairs[index].v[0] == pairs[m_edges[i]].v[0]) && (pairs[index].v[1] == pairs[m_edges[i]].v[1]))
					|| ((pairs[index].v[0] == pairs[m_edges[i]].v[1]) && (pairs[index].v[1] == pairs[m_edges[i]].v[0])))
				{
					return true;
				}
			}
			return false;
		}

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

	public:
		static bool Simplify(const Vector3* pv, const void* pi, int nv, int nt, bool is16bit, const SimplificationConfig& cfg, std::vector<Vector3>& new_v, std::vector<int>& new_i)
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
	};

	bool SimplifyMeshGarlandQEM(const Vector3* pv, const void* pi, int nv, int nt, bool is16bit, const SimplificationConfig& cfg, std::vector<Vector3>& new_v, std::vector<int>& new_i)
	{
		return GarlandQEM::Simplify(pv, pi, nv, nt, is16bit, cfg, new_v, new_i);
	}

	// New Quadric Metric for Simplifying Meshes with Appearance Attributes
	// Efficient Minimization of New Quadric Metric
	class HoppeQEM
	{
	private:
		static constexpr double REDUCTION_EPSILON = 1.0e-12;
		static constexpr double REDUCTION_EDGE_WEIGHT = 512.0;
		static constexpr double REDUCTION_DEGREE_PENALTY = 100.0;
		static constexpr double REDUCTION_INVERSION_PENALTY = 1000000.0;
		static constexpr int REDUCTION_DEGREE_LIMIT = 24;

		struct ReductionVector
		{
			double x;
			double y;
			double z;

			ReductionVector()
				: x(0.0), y(0.0), z(0.0)
			{
			}

			ReductionVector(double vx, double vy, double vz)
				: x(vx), y(vy), z(vz)
			{
			}

			explicit ReductionVector(const Vector3& v)
				: x(v.x), y(v.y), z(v.z)
			{
			}

			ReductionVector operator+(const ReductionVector& v) const
			{
				return ReductionVector(x + v.x, y + v.y, z + v.z);
			}

			ReductionVector operator-(const ReductionVector& v) const
			{
				return ReductionVector(x - v.x, y - v.y, z - v.z);
			}

			ReductionVector operator-() const
			{
				return ReductionVector(-x, -y, -z);
			}

			ReductionVector operator*(double scale) const
			{
				return ReductionVector(x * scale, y * scale, z * scale);
			}

			ReductionVector& operator+=(const ReductionVector& v)
			{
				x += v.x;
				y += v.y;
				z += v.z;
				return *this;
			}

			double Dot(const ReductionVector& v) const
			{
				return x * v.x + y * v.y + z * v.z;
			}

			ReductionVector Cross(const ReductionVector& v) const
			{
				return ReductionVector(
					y * v.z - z * v.y,
					z * v.x - x * v.z,
					x * v.y - y * v.x);
			}

			double SquareLength() const
			{
				return Dot(*this);
			}

			bool IsFinite() const
			{
				return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
			}

			Vector3 ToVector3() const
			{
				return Vector3((float)x, (float)y, (float)z);
			}
		};

		struct ReductionQuadric
		{
			double xx = 0.0;
			double yy = 0.0;
			double zz = 0.0;
			double xy = 0.0;
			double xz = 0.0;
			double yz = 0.0;
			ReductionVector linear;
			double constant = 0.0;
			double area = 0.0;
			ReductionVector volumeNormal;
			double volumeDistance = 0.0;

			static ReductionQuadric FromTriangle(const Vector3& v0, const Vector3& v1, const Vector3& v2)
			{
				const ReductionVector p0(v0);
				const ReductionVector p1(v1);
				const ReductionVector p2(v2);
				const ReductionVector edge01 = p1 - p0;
				const ReductionVector edge02 = p2 - p0;
				ReductionVector normal = edge02.Cross(edge01);
				const double length = std::sqrt(normal.SquareLength());

				ReductionQuadric quadric;
				quadric.volumeNormal = normal;
				quadric.volumeDistance = -normal.Dot(p0);
				quadric.area = length * 0.5;
				if (quadric.area < REDUCTION_EPSILON)
				{
					quadric.volumeNormal = ReductionVector();
					quadric.volumeDistance = 0.0;
					quadric.area = 0.0;
					return quadric;
				}

				normal = normal * (1.0 / length);
				const double distance = -normal.Dot(p0);
				quadric.xx = quadric.area * normal.x * normal.x;
				quadric.yy = quadric.area * normal.y * normal.y;
				quadric.zz = quadric.area * normal.z * normal.z;
				quadric.xy = quadric.area * normal.x * normal.y;
				quadric.xz = quadric.area * normal.x * normal.z;
				quadric.yz = quadric.area * normal.y * normal.z;
				quadric.linear = normal * (quadric.area * distance);
				quadric.constant = quadric.area * distance * distance;
				return quadric;
			}

			static ReductionQuadric FromLine(const Vector3& v0, const Vector3& v1, double weight)
			{
				const ReductionVector p0(v0);
				ReductionVector axis = ReductionVector(v1) - p0;
				const double length = std::sqrt(axis.SquareLength());
				ReductionQuadric quadric;
				if (length < REDUCTION_EPSILON)
				{
					return quadric;
				}

				axis = axis * (1.0 / length);
				const double scale = weight * length;
				quadric.xx = scale * (1.0 - axis.x * axis.x);
				quadric.yy = scale * (1.0 - axis.y * axis.y);
				quadric.zz = scale * (1.0 - axis.z * axis.z);
				quadric.xy = -scale * axis.x * axis.y;
				quadric.xz = -scale * axis.x * axis.z;
				quadric.yz = -scale * axis.y * axis.z;
				quadric.linear = -quadric.Multiply(p0);
				quadric.constant = p0.Dot(quadric.Multiply(p0));
				return quadric;
			}

			ReductionVector Multiply(const ReductionVector& v) const
			{
				return ReductionVector(
					xx * v.x + xy * v.y + xz * v.z,
					xy * v.x + yy * v.y + yz * v.z,
					xz * v.x + yz * v.y + zz * v.z);
			}

			double Evaluate(const Vector3& position) const
			{
				const ReductionVector p(position);
				double error = p.Dot(Multiply(p)) + 2.0 * linear.Dot(p) + constant;
				if (!std::isfinite(error) || error < 0.0)
				{
					error = 0.0;
				}
				return error;
			}

			ReductionQuadric& operator+=(const ReductionQuadric& quadric)
			{
				xx += quadric.xx;
				yy += quadric.yy;
				zz += quadric.zz;
				xy += quadric.xy;
				xz += quadric.xz;
				yz += quadric.yz;
				linear += quadric.linear;
				constant += quadric.constant;
				area += quadric.area;
				volumeNormal += quadric.volumeNormal;
				volumeDistance += quadric.volumeDistance;
				return *this;
			}
		};

		static bool SolveReductionSystem(double* matrix, double* right, int dimension, double* result)
		{
			for (int column = 0; column < dimension; ++column)
			{
				int pivot = column;
				double pivotMagnitude = std::fabs(matrix[column * dimension + column]);
				for (int row = column + 1; row < dimension; ++row)
				{
					const double magnitude = std::fabs(matrix[row * dimension + column]);
					if (magnitude > pivotMagnitude)
					{
						pivot = row;
						pivotMagnitude = magnitude;
					}
				}
				if (pivotMagnitude < REDUCTION_EPSILON)
				{
					return false;
				}

				if (pivot != column)
				{
					for (int i = column; i < dimension; ++i)
					{
						std::swap(matrix[column * dimension + i], matrix[pivot * dimension + i]);
					}
					std::swap(right[column], right[pivot]);
				}

				const double inversePivot = 1.0 / matrix[column * dimension + column];
				for (int row = column + 1; row < dimension; ++row)
				{
					const double factor = matrix[row * dimension + column] * inversePivot;
					if (factor == 0.0)
					{
						continue;
					}
					matrix[row * dimension + column] = 0.0;
					for (int i = column + 1; i < dimension; ++i)
					{
						matrix[row * dimension + i] -= factor * matrix[column * dimension + i];
					}
					right[row] -= factor * right[column];
				}
			}

			for (int row = dimension - 1; row >= 0; --row)
			{
				double value = right[row];
				for (int column = row + 1; column < dimension; ++column)
				{
					value -= matrix[row * dimension + column] * result[column];
				}
				result[row] = value / matrix[row * dimension + row];
				if (!std::isfinite(result[row]))
				{
					return false;
				}
			}
			return true;
		}

		static bool OptimizeReductionPosition(const ReductionQuadric& quadric, Vector3* position)
		{
			if (position == nullptr || quadric.area < REDUCTION_EPSILON)
			{
				return false;
			}
			double matrix[9] = {
				quadric.xx, quadric.xy, quadric.xz,
				quadric.xy, quadric.yy, quadric.yz,
				quadric.xz, quadric.yz, quadric.zz
			};
			double right[3] = { -quadric.linear.x, -quadric.linear.y, -quadric.linear.z };
			double result[3] = {};
			if (!SolveReductionSystem(matrix, right, 3, result))
			{
				return false;
			}
			*position = Vector3((float)result[0], (float)result[1], (float)result[2]);
			return true;
		}

		static bool OptimizeReductionVolume(const ReductionQuadric& quadric, Vector3* position)
		{
			if (position == nullptr || quadric.area < REDUCTION_EPSILON ||
				quadric.volumeNormal.SquareLength() < REDUCTION_EPSILON)
			{
				return false;
			}

			const ReductionVector& normal = quadric.volumeNormal;
			double matrix[16] = {
				quadric.xx, quadric.xy, quadric.xz, normal.x,
				quadric.xy, quadric.yy, quadric.yz, normal.y,
				quadric.xz, quadric.yz, quadric.zz, normal.z,
				normal.x, normal.y, normal.z, 0.0
			};
			double right[4] = {
				-quadric.linear.x,
				-quadric.linear.y,
				-quadric.linear.z,
				-quadric.volumeDistance
			};
			double result[4] = {};
			if (!SolveReductionSystem(matrix, right, 4, result))
			{
				return false;
			}
			*position = Vector3((float)result[0], (float)result[1], (float)result[2]);
			return true;
		}

		static bool OptimizeReductionEdge(const ReductionQuadric& quadric, const Vector3& v0, const Vector3& v1, Vector3* position)
		{
			if (position == nullptr || quadric.area < REDUCTION_EPSILON)
			{
				return false;
			}

			const ReductionVector p0(v0);
			const ReductionVector p1(v1);
			const ReductionVector m0 = quadric.Multiply(p0);
			const ReductionVector m1 = quadric.Multiply(p1);
			const ReductionVector m01 = m1 - m0;
			const double m01Sqr = m01.SquareLength();
			if (m01Sqr < 1.0e-16)
			{
				return false;
			}

			const ReductionVector right = -quadric.linear;
			const ReductionVector residual = right - m0;
			double t = m01.Dot(residual) / m01Sqr;
			const double normalSqr = quadric.volumeNormal.SquareLength();
			if (normalSqr > REDUCTION_EPSILON)
			{
				const double normal0 = quadric.volumeNormal.Dot(p0);
				const double normal01 = quadric.volumeNormal.Dot(p1) - normal0;
				const double ataXX = m01Sqr + normal01 * normal01;
				const double ataXY = m01.Dot(quadric.volumeNormal);
				const double determinant = ataXX * normalSqr - ataXY * ataXY;
				if (std::fabs(determinant) > 1.0e-16)
				{
					const double atb0 = m01.Dot(residual) -
						(quadric.volumeDistance + normal0) * normal01;
					const double atb1 = quadric.volumeNormal.Dot(residual);
					t = (normalSqr * atb0 - ataXY * atb1) / determinant;
				}
			}

			t = std::max(0.0, std::min(t, 1.0));
			*position = (p0 * (1.0 - t) + p1 * t).ToVector3();
			return true;
		}

		struct ReductionPositionKey
		{
			uint32_t x;
			uint32_t y;
			uint32_t z;

			static uint32_t FloatBits(float value)
			{
				if (value == 0.0f)
				{
					return 0;
				}
				uint32_t bits = 0;
				std::memcpy(&bits, &value, sizeof(bits));
				return bits;
			}

			explicit ReductionPositionKey(const Vector3& position)
				: x(FloatBits(position.x)), y(FloatBits(position.y)), z(FloatBits(position.z))
			{
			}

			bool operator==(const ReductionPositionKey& key) const
			{
				return x == key.x && y == key.y && z == key.z;
			}
		};

		struct ReductionPositionKeyHash
		{
			size_t operator()(const ReductionPositionKey& key) const
			{
				size_t hash = (size_t)key.x;
				hash ^= (size_t)key.y + 0x9e3779b9u + (hash << 6) + (hash >> 2);
				hash ^= (size_t)key.z + 0x9e3779b9u + (hash << 6) + (hash >> 2);
				return hash;
			}
		};

		struct ReductionFaceKey
		{
			int v[3];

			ReductionFaceKey(int v0, int v1, int v2)
			{
				const int rotations[3][3] = {
					{ v0, v1, v2 },
					{ v1, v2, v0 },
					{ v2, v0, v1 }
				};
				int best = 0;
				for (int i = 1; i < 3; ++i)
				{
					if (rotations[i][0] < rotations[best][0] ||
						(rotations[i][0] == rotations[best][0] && rotations[i][1] < rotations[best][1]) ||
						(rotations[i][0] == rotations[best][0] && rotations[i][1] == rotations[best][1] && rotations[i][2] < rotations[best][2]))
					{
						best = i;
					}
				}
				v[0] = rotations[best][0];
				v[1] = rotations[best][1];
				v[2] = rotations[best][2];
			}

			bool operator==(const ReductionFaceKey& key) const
			{
				return v[0] == key.v[0] && v[1] == key.v[1] && v[2] == key.v[2];
			}
		};

		struct ReductionFaceKeyHash
		{
			size_t operator()(const ReductionFaceKey& key) const
			{
				size_t hash = (size_t)key.v[0];
				hash ^= (size_t)key.v[1] + 0x9e3779b9u + (hash << 6) + (hash >> 2);
				hash ^= (size_t)key.v[2] + 0x9e3779b9u + (hash << 6) + (hash >> 2);
				return hash;
			}
		};

		struct ReductionVertex
		{
			Vector3 position = Vector3::Zero();
			std::unordered_set<int> faces;
			uint32_t revision = 1;
			bool active = false;
		};

		struct ReductionFace
		{
			int v[3] = { 0, 0, 0 };
			ReductionQuadric quadric;
			bool active = true;
		};

		struct ReductionCandidate
		{
			double error = std::numeric_limits<double>::max();
			int v0 = -1;
			int v1 = -1;
			uint32_t revision0 = 0;
			uint32_t revision1 = 0;
			Vector3 position = Vector3::Zero();

			bool operator<(const ReductionCandidate& candidate) const
			{
				if (error != candidate.error)
				{
					return error > candidate.error;
				}
				if (v0 != candidate.v0)
				{
					return v0 > candidate.v0;
				}
				return v1 > candidate.v1;
			}
		};

		class MeshReductionOptimizer
		{
		public:
			bool Initialize(const Vector3* vertices, const void* indices, int vertexCount, int triangleCount, bool indices16Bit)
			{
				if (vertices == nullptr || indices == nullptr || vertexCount <= 0 || triangleCount <= 0)
				{
					return false;
				}

				std::vector<int> vertexRemap((size_t)vertexCount, -1);
				std::unordered_map<ReductionPositionKey, int, ReductionPositionKeyHash> weldedVertices;
				m_vertices.reserve((size_t)vertexCount);
				for (int i = 0; i < vertexCount; ++i)
				{
					if (!std::isfinite(vertices[i].x) || !std::isfinite(vertices[i].y) || !std::isfinite(vertices[i].z))
					{
						return false;
					}
					const ReductionPositionKey key(vertices[i]);
					auto found = weldedVertices.find(key);
					if (found != weldedVertices.end())
					{
						vertexRemap[(size_t)i] = found->second;
						continue;
					}

					const int newIndex = (int)m_vertices.size();
					ReductionVertex vertex;
					vertex.position = vertices[i];
					m_vertices.push_back(vertex);
					weldedVertices.insert(std::make_pair(key, newIndex));
					vertexRemap[(size_t)i] = newIndex;
				}

				const uint16_t* indices16 = indices16Bit ? static_cast<const uint16_t*>(indices) : nullptr;
				const uint32_t* indices32 = indices16Bit ? nullptr : static_cast<const uint32_t*>(indices);
				m_faces.reserve((size_t)triangleCount);
				for (int triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex)
				{
					int sourceIndices[3];
					for (int corner = 0; corner < 3; ++corner)
					{
						const uint32_t sourceIndex = indices16 ? (uint32_t)indices16[triangleIndex * 3 + corner]
							: indices32[triangleIndex * 3 + corner];
						if (sourceIndex >= (uint32_t)vertexCount)
						{
							return false;
						}
						sourceIndices[corner] = vertexRemap[(size_t)sourceIndex];
					}

					if (sourceIndices[0] == sourceIndices[1] || sourceIndices[1] == sourceIndices[2] || sourceIndices[2] == sourceIndices[0])
					{
						continue;
					}
					const ReductionFaceKey faceKey(sourceIndices[0], sourceIndices[1], sourceIndices[2]);
					if (m_faceMap.find(faceKey) != m_faceMap.end())
					{
						continue;
					}

					ReductionFace face;
					face.v[0] = sourceIndices[0];
					face.v[1] = sourceIndices[1];
					face.v[2] = sourceIndices[2];
					face.quadric = CalculateFaceQuadric(face);
					const int faceIndex = (int)m_faces.size();
					m_faces.push_back(face);
					m_faceMap.insert(std::make_pair(faceKey, faceIndex));
					AttachFaceTopology(faceIndex);
				}

				m_remainingFaces = (int)m_faces.size();
				if (m_remainingFaces == 0)
				{
					return false;
				}

				std::unordered_set<uint64_t> edges;
				GatherAllEdges(&edges);
				for (uint64_t edge : edges)
				{
					PushCandidate((int)(edge >> 32), (int)(edge & 0xffffffffu));
				}
				return true;
			}

			bool Simplify(const SimplificationConfig& config, std::vector<Vector3>* vertices, std::vector<int>* indices)
			{
				if (vertices == nullptr || indices == nullptr || m_remainingFaces <= 0)
				{
					return false;
				}

				const double rate = std::max(0.0, std::min((double)config.rate, 1.0));
				int targetFaces = (int)std::ceil((double)m_remainingFaces * rate);
				if (config.faces > 0)
				{
					targetFaces = std::min(targetFaces, config.faces);
				}
				targetFaces = std::max(2, std::min(targetFaces, m_remainingFaces));

				while (m_remainingFaces > targetFaces && !m_candidates.empty())
				{
					const ReductionCandidate candidate = m_candidates.top();
					m_candidates.pop();
					if (!IsCandidateCurrent(candidate))
					{
						continue;
					}
					Collapse(candidate);
				}

				// This matches the reduction interface's final fallback when legal collapses are exhausted.
				for (ReductionFace& face : m_faces)
				{
					if (m_remainingFaces <= targetFaces)
					{
						break;
					}
					if (face.active)
					{
						face.active = false;
						--m_remainingFaces;
					}
				}

				return Compact(vertices, indices);
			}

		private:
			static uint64_t DirectedEdgeKey(int v0, int v1)
			{
				return ((uint64_t)(uint32_t)v0 << 32) | (uint32_t)v1;
			}

			static uint64_t UndirectedEdgeKey(int v0, int v1)
			{
				if (v1 < v0)
				{
					std::swap(v0, v1);
				}
				return DirectedEdgeKey(v0, v1);
			}

			ReductionQuadric CalculateFaceQuadric(const ReductionFace& face) const
			{
				return ReductionQuadric::FromTriangle(
					m_vertices[(size_t)face.v[0]].position,
					m_vertices[(size_t)face.v[1]].position,
					m_vertices[(size_t)face.v[2]].position);
			}

			bool FaceIsDegenerate(const ReductionFace& face) const
			{
				if (face.v[0] == face.v[1] || face.v[1] == face.v[2] || face.v[2] == face.v[0])
				{
					return true;
				}
				const Vector3& v0 = m_vertices[(size_t)face.v[0]].position;
				const Vector3& v1 = m_vertices[(size_t)face.v[1]].position;
				const Vector3& v2 = m_vertices[(size_t)face.v[2]].position;
				return (v0.x == v1.x && v0.y == v1.y && v0.z == v1.z) ||
					(v1.x == v2.x && v1.y == v2.y && v1.z == v2.z) ||
					(v2.x == v0.x && v2.y == v0.y && v2.z == v0.z);
			}

			void ChangeDirectedEdgeCount(int v0, int v1, int delta)
			{
				const uint64_t key = DirectedEdgeKey(v0, v1);
				auto found = m_directedEdgeCounts.find(key);
				if (found == m_directedEdgeCounts.end())
				{
					if (delta > 0)
					{
						m_directedEdgeCounts.insert(std::make_pair(key, delta));
					}
					return;
				}
				found->second += delta;
				if (found->second <= 0)
				{
					m_directedEdgeCounts.erase(found);
				}
			}

			int DirectedEdgeCount(int v0, int v1) const
			{
				auto found = m_directedEdgeCounts.find(DirectedEdgeKey(v0, v1));
				return found == m_directedEdgeCounts.end() ? 0 : found->second;
			}

			void AttachFaceTopology(int faceIndex)
			{
				ReductionFace& face = m_faces[(size_t)faceIndex];
				for (int corner = 0; corner < 3; ++corner)
				{
					ReductionVertex& vertex = m_vertices[(size_t)face.v[corner]];
					vertex.faces.insert(faceIndex);
					vertex.active = true;
					ChangeDirectedEdgeCount(face.v[corner], face.v[(corner + 1) % 3], 1);
				}
			}

			void DetachFaceTopology(int faceIndex)
			{
				ReductionFace& face = m_faces[(size_t)faceIndex];
				for (int corner = 0; corner < 3; ++corner)
				{
					m_vertices[(size_t)face.v[corner]].faces.erase(faceIndex);
					ChangeDirectedEdgeCount(face.v[corner], face.v[(corner + 1) % 3], -1);
				}
				const ReductionFaceKey key(face.v[0], face.v[1], face.v[2]);
				auto found = m_faceMap.find(key);
				if (found != m_faceMap.end() && found->second == faceIndex)
				{
					m_faceMap.erase(found);
				}
			}

			void GatherAllEdges(std::unordered_set<uint64_t>* edges) const
			{
				if (edges == nullptr)
				{
					return;
				}
				for (const ReductionFace& face : m_faces)
				{
					if (!face.active)
					{
						continue;
					}
					for (int corner = 0; corner < 3; ++corner)
					{
						edges->insert(UndirectedEdgeKey(face.v[corner], face.v[(corner + 1) % 3]));
					}
				}
			}

			bool IsEdge(int v0, int v1) const
			{
				if (v0 < 0 || v1 < 0 || v0 == v1 || v0 >= (int)m_vertices.size() || v1 >= (int)m_vertices.size())
				{
					return false;
				}
				const ReductionVertex& vertex0 = m_vertices[(size_t)v0];
				const ReductionVertex& vertex1 = m_vertices[(size_t)v1];
				if (!vertex0.active || !vertex1.active)
				{
					return false;
				}
				const std::unordered_set<int>& faces = vertex0.faces.size() < vertex1.faces.size() ? vertex0.faces : vertex1.faces;
				for (int faceIndex : faces)
				{
					const ReductionFace& face = m_faces[(size_t)faceIndex];
					if (!face.active)
					{
						continue;
					}
					bool hasV0 = false;
					bool hasV1 = false;
					for (int corner = 0; corner < 3; ++corner)
					{
						hasV0 = hasV0 || face.v[corner] == v0;
						hasV1 = hasV1 || face.v[corner] == v1;
					}
					if (hasV0 && hasV1)
					{
						return true;
					}
				}
				return false;
			}

			void GatherAdjacentFaces(int v0, int v1, std::vector<int>* faces) const
			{
				faces->clear();
				std::unordered_set<int> uniqueFaces;
				for (int faceIndex : m_vertices[(size_t)v0].faces)
				{
					if (m_faces[(size_t)faceIndex].active && uniqueFaces.insert(faceIndex).second)
					{
						faces->push_back(faceIndex);
					}
				}
				for (int faceIndex : m_vertices[(size_t)v1].faces)
				{
					if (m_faces[(size_t)faceIndex].active && uniqueFaces.insert(faceIndex).second)
					{
						faces->push_back(faceIndex);
					}
				}
			}

			bool TriangleWillInvert(const ReductionFace& face, int v0, int v1, const Vector3& newPosition) const
			{
				int movedCorner = -1;
				int movedCount = 0;
				for (int corner = 0; corner < 3; ++corner)
				{
					if (face.v[corner] == v0 || face.v[corner] == v1)
					{
						movedCorner = corner;
						++movedCount;
					}
				}
				if (movedCount != 1)
				{
					return false;
				}

				const Vector3& oldPosition = m_vertices[(size_t)face.v[movedCorner]].position;
				const Vector3& position1 = m_vertices[(size_t)face.v[(movedCorner + 1) % 3]].position;
				const Vector3& position2 = m_vertices[(size_t)face.v[(movedCorner + 2) % 3]].position;
				const Vector3 edge21 = position2 - position1;
				const Vector3 oldEdge = oldPosition - position1;
				const Vector3 newEdge = newPosition - position1;
				return oldEdge.Cross(edge21).Dot(newEdge.Cross(edge21)) < 0.0f;
			}

			bool PositionIsValid(const Vector3& position, int v0, int v1, const std::vector<int>& adjacentFaces) const
			{
				Vector3 boundsMin(std::numeric_limits<float>::max());
				Vector3 boundsMax(-std::numeric_limits<float>::max());
				for (int faceIndex : adjacentFaces)
				{
					const ReductionFace& face = m_faces[(size_t)faceIndex];
					for (int corner = 0; corner < 3; ++corner)
					{
						const Vector3& vertex = m_vertices[(size_t)face.v[corner]].position;
						boundsMin.x = std::min(boundsMin.x, vertex.x);
						boundsMin.y = std::min(boundsMin.y, vertex.y);
						boundsMin.z = std::min(boundsMin.z, vertex.z);
						boundsMax.x = std::max(boundsMax.x, vertex.x);
						boundsMax.y = std::max(boundsMax.y, vertex.y);
						boundsMax.z = std::max(boundsMax.z, vertex.z);
					}
				}

				double distanceSqr = 0.0;
				for (int axis = 0; axis < 3; ++axis)
				{
					const float value = position[axis];
					if (value < boundsMin[axis])
					{
						const double distance = boundsMin[axis] - value;
						distanceSqr += distance * distance;
					}
					else if (value > boundsMax[axis])
					{
						const double distance = value - boundsMax[axis];
						distanceSqr += distance * distance;
					}
				}
				const Vector3 boundsSize = boundsMax - boundsMin;
				if (distanceSqr > (double)boundsSize.SquareLength() * 4.0)
				{
					return false;
				}

				for (int faceIndex : adjacentFaces)
				{
					if (TriangleWillInvert(m_faces[(size_t)faceIndex], v0, v1, position))
					{
						return false;
					}
				}
				return true;
			}

			ReductionCandidate EvaluateCandidate(int v0, int v1) const
			{
				ReductionCandidate candidate;
				if (v1 < v0)
				{
					std::swap(v0, v1);
				}
				candidate.v0 = v0;
				candidate.v1 = v1;
				candidate.revision0 = m_vertices[(size_t)v0].revision;
				candidate.revision1 = m_vertices[(size_t)v1].revision;
				if (!IsEdge(v0, v1))
				{
					return candidate;
				}

				std::vector<int> adjacentFaces;
				GatherAdjacentFaces(v0, v1, &adjacentFaces);
				if (adjacentFaces.empty() || adjacentFaces.size() == (size_t)m_remainingFaces)
				{
					return candidate;
				}

				ReductionQuadric quadric;
				for (int faceIndex : adjacentFaces)
				{
					const ReductionFace& face = m_faces[(size_t)faceIndex];
					quadric += face.quadric;
					for (int corner = 0; corner < 3; ++corner)
					{
						const int edgeV0 = face.v[corner];
						const int edgeV1 = face.v[(corner + 1) % 3];
						const bool touchesMerge = edgeV0 == v0 || edgeV0 == v1 || edgeV1 == v0 || edgeV1 == v1;
						if (touchesMerge && DirectedEdgeCount(edgeV1, edgeV0) == 0)
						{
							quadric += ReductionQuadric::FromLine(
								m_vertices[(size_t)edgeV0].position,
								m_vertices[(size_t)edgeV1].position,
								REDUCTION_EDGE_WEIGHT);
						}
					}
				}

				double penalty = 0.0;
				const int degree = (int)m_vertices[(size_t)v0].faces.size() + (int)m_vertices[(size_t)v1].faces.size();
				if (degree > REDUCTION_DEGREE_LIMIT)
				{
					penalty += REDUCTION_DEGREE_PENALTY * (degree - REDUCTION_DEGREE_LIMIT);
				}

				Vector3 newPosition;
				bool positionValid = OptimizeReductionVolume(quadric, &newPosition) &&
					PositionIsValid(newPosition, v0, v1, adjacentFaces);
				if (!positionValid)
				{
					positionValid = OptimizeReductionPosition(quadric, &newPosition) &&
						PositionIsValid(newPosition, v0, v1, adjacentFaces);
				}
				if (!positionValid)
				{
					positionValid = OptimizeReductionEdge(
						quadric,
						m_vertices[(size_t)v0].position,
						m_vertices[(size_t)v1].position,
						&newPosition) && PositionIsValid(newPosition, v0, v1, adjacentFaces);
				}
				if (!positionValid)
				{
					newPosition = (m_vertices[(size_t)v0].position + m_vertices[(size_t)v1].position) * 0.5f;
					if (!PositionIsValid(newPosition, v0, v1, adjacentFaces))
					{
						penalty += REDUCTION_INVERSION_PENALTY;
					}
				}

				candidate.position = newPosition;
				candidate.error = quadric.Evaluate(newPosition) + penalty;
				return candidate;
			}

			void PushCandidate(int v0, int v1)
			{
				if (!IsEdge(v0, v1))
				{
					return;
				}
				ReductionCandidate candidate = EvaluateCandidate(v0, v1);
				if (std::isfinite(candidate.error) && candidate.error < std::numeric_limits<double>::max())
				{
					m_candidates.push(candidate);
				}
			}

			bool IsCandidateCurrent(const ReductionCandidate& candidate) const
			{
				return candidate.v0 >= 0 && candidate.v1 >= 0 &&
					m_vertices[(size_t)candidate.v0].active && m_vertices[(size_t)candidate.v1].active &&
					m_vertices[(size_t)candidate.v0].revision == candidate.revision0 &&
					m_vertices[(size_t)candidate.v1].revision == candidate.revision1 &&
					IsEdge(candidate.v0, candidate.v1);
			}

			void Collapse(const ReductionCandidate& candidate)
			{
				const int keepVertex = candidate.v0;
				const int removeVertex = candidate.v1;
				std::vector<int> adjacentFaces;
				GatherAdjacentFaces(keepVertex, removeVertex, &adjacentFaces);
				if (adjacentFaces.empty())
				{
					return;
				}

				std::unordered_set<int> affectedVertices;
				for (int faceIndex : adjacentFaces)
				{
					ReductionFace& face = m_faces[(size_t)faceIndex];
					for (int corner = 0; corner < 3; ++corner)
					{
						affectedVertices.insert(face.v[corner]);
					}
					DetachFaceTopology(faceIndex);
				}

				m_vertices[(size_t)keepVertex].position = candidate.position;
				m_vertices[(size_t)removeVertex].active = false;
				for (int faceIndex : adjacentFaces)
				{
					ReductionFace& face = m_faces[(size_t)faceIndex];
					for (int corner = 0; corner < 3; ++corner)
					{
						if (face.v[corner] == removeVertex)
						{
							face.v[corner] = keepVertex;
						}
					}

					if (FaceIsDegenerate(face))
					{
						face.active = false;
						--m_remainingFaces;
						continue;
					}

					const ReductionFaceKey key(face.v[0], face.v[1], face.v[2]);
					if (m_faceMap.find(key) != m_faceMap.end())
					{
						face.active = false;
						--m_remainingFaces;
						continue;
					}

					face.quadric = CalculateFaceQuadric(face);
					m_faceMap.insert(std::make_pair(key, faceIndex));
					AttachFaceTopology(faceIndex);
					for (int corner = 0; corner < 3; ++corner)
					{
						affectedVertices.insert(face.v[corner]);
					}
				}

				affectedVertices.insert(keepVertex);
				affectedVertices.insert(removeVertex);
				for (int vertexIndex : affectedVertices)
				{
					ReductionVertex& vertex = m_vertices[(size_t)vertexIndex];
					vertex.active = !vertex.faces.empty();
					++vertex.revision;
				}

				std::unordered_set<uint64_t> edges;
				for (int vertexIndex : affectedVertices)
				{
					const ReductionVertex& vertex = m_vertices[(size_t)vertexIndex];
					for (int faceIndex : vertex.faces)
					{
						const ReductionFace& face = m_faces[(size_t)faceIndex];
						if (!face.active)
						{
							continue;
						}
						for (int corner = 0; corner < 3; ++corner)
						{
							edges.insert(UndirectedEdgeKey(face.v[corner], face.v[(corner + 1) % 3]));
						}
					}
				}
				for (uint64_t edge : edges)
				{
					PushCandidate((int)(edge >> 32), (int)(edge & 0xffffffffu));
				}
			}

			bool Compact(std::vector<Vector3>* vertices, std::vector<int>* indices) const
			{
				vertices->clear();
				indices->clear();
				std::vector<int> vertexRemap(m_vertices.size(), -1);
				indices->reserve((size_t)m_remainingFaces * 3);
				for (const ReductionFace& face : m_faces)
				{
					if (!face.active)
					{
						continue;
					}
					for (int corner = 0; corner < 3; ++corner)
					{
						const int oldIndex = face.v[corner];
						int& newIndex = vertexRemap[(size_t)oldIndex];
						if (newIndex < 0)
						{
							newIndex = (int)vertices->size();
							vertices->push_back(m_vertices[(size_t)oldIndex].position);
						}
						indices->push_back(newIndex);
					}
				}
				return !vertices->empty() && indices->size() >= 6;
			}

		private:
			std::vector<ReductionVertex> m_vertices;
			std::vector<ReductionFace> m_faces;
			std::unordered_map<ReductionFaceKey, int, ReductionFaceKeyHash> m_faceMap;
			std::unordered_map<uint64_t, int> m_directedEdgeCounts;
			std::priority_queue<ReductionCandidate> m_candidates;
			int m_remainingFaces = 0;
		};

	public:
		static bool Simplify(const Vector3* pv, const void* pi, int nv, int nt, bool is16bit, const SimplificationConfig& cfg, std::vector<Vector3>& new_v, std::vector<int>& new_i)
		{
			MeshReductionOptimizer optimizer;
			if (!optimizer.Initialize(pv, pi, nv, nt, is16bit))
			{
				return false;
			}
			return optimizer.Simplify(cfg, &new_v, &new_i);
		}
	};

	bool SimplifyMeshHoppeQEM(const Vector3* pv, const void* pi, int nv, int nt, bool is16bit, const SimplificationConfig& cfg, std::vector<Vector3>& new_v, std::vector<int>& new_i)
	{
		return HoppeQEM::Simplify(pv, pi, nv, nt, is16bit, cfg, new_v, new_i);
	}
}
