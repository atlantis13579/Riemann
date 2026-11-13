#include "PBDBackend.h"
#include "PBDBackendMath.h"

#include <algorithm>
#include <cmath>
#include <limits>

#if defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#define RIEMANN_PBD_SSE2 1
#include <emmintrin.h>
#endif

namespace Riemann
{
	namespace
	{
		bool Finite(const PBDBackendVector& Value)
		{
			return std::isfinite(Value.x) && std::isfinite(Value.y) && std::isfinite(Value.z) && std::isfinite(Value.w);
		}

#if defined(RIEMANN_PBD_SSE2)
		// SSE2 evaluates the XYZ Jacobian, inertia and quaternion vector operations.
		// Scalar row dependencies are preserved; no padded fourth lane enters a dot product.
		struct SIMD
		{
			using Vector = PBDBackendVector;
			static Vector Store(__m128 Value)
			{
				Vector Result;
				_mm_storeu_ps(&Result.x, Value);
				Result.w = 0.0f;
				return Result;
			}
			static Vector Add(const Vector& A, const Vector& B) { return Store(_mm_add_ps(_mm_loadu_ps(&A.x), _mm_loadu_ps(&B.x))); }
			static Vector Sub(const Vector& A, const Vector& B) { return Store(_mm_sub_ps(_mm_loadu_ps(&A.x), _mm_loadu_ps(&B.x))); }
			static Vector Mul(const Vector& A, float B) { return Store(_mm_mul_ps(_mm_loadu_ps(&A.x), _mm_set1_ps(B))); }
			static float Dot(const Vector& A, const Vector& B)
			{
				const __m128 Product = _mm_mul_ps(_mm_loadu_ps(&A.x), _mm_loadu_ps(&B.x));
				const __m128 XY = _mm_add_ss(Product, _mm_shuffle_ps(Product, Product, _MM_SHUFFLE(1, 1, 1, 1)));
				return _mm_cvtss_f32(_mm_add_ss(XY, _mm_shuffle_ps(Product, Product, _MM_SHUFFLE(2, 2, 2, 2))));
			}
			static Vector Cross(const Vector& A, const Vector& B)
			{
				const __m128 VA = _mm_loadu_ps(&A.x), VB = _mm_loadu_ps(&B.x);
				const __m128 AYZX = _mm_shuffle_ps(VA, VA, _MM_SHUFFLE(3, 0, 2, 1));
				const __m128 AZXY = _mm_shuffle_ps(VA, VA, _MM_SHUFFLE(3, 1, 0, 2));
				const __m128 BYZX = _mm_shuffle_ps(VB, VB, _MM_SHUFFLE(3, 0, 2, 1));
				const __m128 BZXY = _mm_shuffle_ps(VB, VB, _MM_SHUFFLE(3, 1, 0, 2));
				return Store(_mm_sub_ps(_mm_mul_ps(AYZX, BZXY), _mm_mul_ps(AZXY, BYZX)));
			}
		};
#endif

		bool ValidParameters(const PBDBackendParameters& Parameters, PBDBackendStage Stage)
		{
			return std::isfinite(Parameters.DeltaTime) && Parameters.DeltaTime > 0.0f
				&& std::isfinite(Parameters.Alpha) && Parameters.Alpha >= 0.0f
				&& std::isfinite(Parameters.PenetrationSlop) && Parameters.PenetrationSlop >= 0.0f
				&& std::isfinite(Parameters.MaxPositionCorrection) && Parameters.MaxPositionCorrection > 0.0f
				&& std::isfinite(Parameters.MaxAngularCorrection) && Parameters.MaxAngularCorrection > 0.0f
				&& static_cast<uint32_t>(Stage) <= static_cast<uint32_t>(PBDBackendStage::Velocity);
		}
	}

	bool ValidatePBDBackendOutput(const PBDBackendBody* Bodies, uint32_t BodyCount,
		const PBDBackendContact* Contacts, uint32_t ContactCount)
	{
		if ((!Bodies && BodyCount) || (!Contacts && ContactCount)) return false;
		for (uint32_t Index = 0; Index < BodyCount; ++Index)
		{
			const PBDBackendBody& Body = Bodies[Index];
			if (!Finite(Body.Position) || !Finite(Body.Rotation) || !Finite(Body.LinearVelocity)
				|| !Finite(Body.AngularVelocity) || !Finite(Body.InverseInertia0)
				|| !Finite(Body.InverseInertia1) || !Finite(Body.InverseInertia2)
				|| Body.Position.w < 0.0f || Body.LinearVelocity.w < 0.0f
				|| Body.Flags.x > 1 || Body.Flags.y > 1 || Body.Flags.z > 1
				|| (!Body.Flags.x && (Body.Position.w != 0.0f || Body.Flags.z != 0))) return false;
			const float Norm = PBDBackendMath::Scalar::Dot(Body.Rotation, Body.Rotation) + Body.Rotation.w * Body.Rotation.w;
			if (!std::isfinite(Norm) || std::abs(Norm - 1.0f) > 0.01f) return false;
		}
		for (uint32_t Index = 0; Index < ContactCount; ++Index)
		{
			const PBDBackendContact& Contact = Contacts[Index];
			if (Contact.Bodies.x >= BodyCount || Contact.Bodies.y >= BodyCount || Contact.Bodies.x == Contact.Bodies.y
				|| !Finite(Contact.AnchorA) || !Finite(Contact.AnchorB) || !Finite(Contact.Normal)
				|| !Finite(Contact.Tangent) || !Finite(Contact.Binormal) || !Finite(Contact.Lambda)
				|| !Finite(Contact.Material) || Contact.Lambda.x < 0.0f || Contact.Material.x < 0.0f
				|| Contact.Material.y < 0.0f || Contact.Material.z < 0.0f) return false;
		}
		return true;
	}

	bool IsPBDBackendAvailable(SolverBackend Backend)
	{
		switch (Backend)
		{
		case SolverBackend::CPU: return true;
		case SolverBackend::SIMD:
#if defined(RIEMANN_PBD_SSE2)
			return true;
#else
			return false;
#endif
		case SolverBackend::ComputeShader: return IsPBDComputeShaderBackendAvailable();
		case SolverBackend::CUDA:
#if defined(RIEMANN_ENABLE_CUDA)
			return IsPBDCUDAAvailable();
#else
			return false;
#endif
		default: return false;
		}
	}

	PBDBackend::~PBDBackend() { ReleaseContext(); }

	void PBDBackend::ReleaseContext()
	{
		if (!m_Context) return;
		if (m_Statistics.Backend == SolverBackend::ComputeShader) DestroyPBDComputeShaderContext(m_Context);
#if defined(RIEMANN_ENABLE_CUDA)
		else if (m_Statistics.Backend == SolverBackend::CUDA) DestroyPBDCUDAContext(m_Context);
#endif
		m_Context = nullptr;
	}

	bool PBDBackend::Configure(SolverBackend Backend, bool AllowFallback)
	{
		ReleaseContext();
		m_Statistics = PBDBackendStatistics();
		m_Statistics.RequestedBackend = m_Statistics.Backend = Backend;
		m_AllowFallback = AllowFallback;
		m_Ready = false;
		if (static_cast<uint32_t>(Backend) > static_cast<uint32_t>(SolverBackend::CUDA))
		{
			m_Statistics.Message = "Invalid PBD solver backend";
			return false;
		}
		bool Available = IsPBDBackendAvailable(Backend);
		if (Available && Backend == SolverBackend::ComputeShader)
			Available = (m_Context = CreatePBDComputeShaderContext()) != nullptr;
#if defined(RIEMANN_ENABLE_CUDA)
		if (Available && Backend == SolverBackend::CUDA)
			Available = (m_Context = CreatePBDCUDAContext()) != nullptr;
#endif
		if (!Available)
		{
			m_Statistics.Message = "Requested PBD solver backend is unavailable";
			if (!AllowFallback) return false;
			m_Statistics.Backend = SolverBackend::CPU;
			m_Statistics.UsedFallback = true;
			m_Statistics.Message += "; using CPU";
		}
		m_Ready = true;
		return true;
	}

	bool PBDBackend::Execute(std::vector<PBDBackendBody>& Bodies, std::vector<PBDBackendContact>& Contacts,
		const PBDBackendParameters& Parameters, PBDBackendStage Stage)
	{
		if (!m_Ready) return false;
		if (Bodies.size() > UINT32_MAX || Contacts.size() > (size_t)std::numeric_limits<int>::max() || !ValidParameters(Parameters, Stage)
			|| !ValidatePBDBackendOutput(Bodies.data(), (uint32_t)Bodies.size(), Contacts.data(), (uint32_t)Contacts.size()))
		{
			m_Statistics.Message = "Invalid PBD contact batch";
			return false;
		}
		if (Contacts.empty()) return true;

		// Dependency levels preserve every original read/write dependency. Read-only
		// ground and kinematic bodies do not serialize otherwise independent contacts.
		std::vector<int> LastLayer(Bodies.size(), -1);
		std::vector<uint32_t> Layers(Contacts.size());
		uint32_t LayerCount = 0;
		for (size_t Index = 0; Index < Contacts.size(); ++Index)
		{
			const PBDBackendContact& Contact = Contacts[Index];
			const uint32_t A = Contact.Bodies.x, B = Contact.Bodies.y;
			const uint32_t Layer = (uint32_t)std::max(Bodies[A].Flags.x ? LastLayer[A] + 1 : 0,
				Bodies[B].Flags.x ? LastLayer[B] + 1 : 0);
			Layers[Index] = Layer;
			LayerCount = std::max(LayerCount, Layer + 1);
			if (Bodies[A].Flags.x) LastLayer[A] = (int)Layer;
			if (Bodies[B].Flags.x) LastLayer[B] = (int)Layer;
		}
		m_LayerOffsets.assign((size_t)LayerCount + 1, 0);
		for (uint32_t Layer : Layers) ++m_LayerOffsets[(size_t)Layer + 1];
		for (uint32_t Layer = 0; Layer < LayerCount; ++Layer) m_LayerOffsets[Layer + 1] += m_LayerOffsets[Layer];
		std::vector<uint32_t> Cursor = m_LayerOffsets;
		m_RowIndices.resize(Contacts.size());
		for (uint32_t Index = 0; Index < Contacts.size(); ++Index) m_RowIndices[Cursor[Layers[Index]]++] = Index;

		m_WorkBodies = Bodies;
		m_WorkContacts = Contacts;
		bool Success = true;
		if (m_Statistics.Backend == SolverBackend::ComputeShader)
			Success = ExecutePBDComputeShader(m_Context, m_WorkBodies.data(), (uint32_t)m_WorkBodies.size(),
				m_WorkContacts.data(), (uint32_t)m_WorkContacts.size(), m_RowIndices.data(), m_LayerOffsets.data(), LayerCount, Parameters, Stage);
#if defined(RIEMANN_ENABLE_CUDA)
		else if (m_Statistics.Backend == SolverBackend::CUDA)
			Success = ExecutePBDCUDA(m_Context, m_WorkBodies.data(), (uint32_t)m_WorkBodies.size(),
				m_WorkContacts.data(), (uint32_t)m_WorkContacts.size(), m_RowIndices.data(), m_LayerOffsets.data(), LayerCount, Parameters, Stage);
#endif
		if (!Success)
		{
			m_Statistics.Message = "PBD device execution failed";
			ReleaseContext();
			if (!m_AllowFallback) { m_Ready = false; return false; }
			m_Statistics.Backend = SolverBackend::CPU;
			m_Statistics.UsedFallback = true;
			m_Statistics.Message += "; using CPU";
			m_WorkBodies = Bodies;
			m_WorkContacts = Contacts;
		}
		if (m_Statistics.Backend == SolverBackend::CPU || m_Statistics.Backend == SolverBackend::SIMD)
		{
			for (uint32_t Index : m_RowIndices)
			{
#if defined(RIEMANN_PBD_SSE2)
				if (m_Statistics.Backend == SolverBackend::SIMD)
					PBDBackendMath::SolveRow<SIMD>(m_WorkBodies.data(), m_WorkContacts[Index], Parameters, Stage);
				else
#endif
					PBDBackendMath::SolveRow<PBDBackendMath::Scalar>(m_WorkBodies.data(), m_WorkContacts[Index], Parameters, Stage);
			}
		}
		if (!ValidatePBDBackendOutput(m_WorkBodies.data(), (uint32_t)m_WorkBodies.size(), m_WorkContacts.data(), (uint32_t)m_WorkContacts.size()))
		{
			m_Statistics.Message = "PBD contact solve produced invalid output";
			return false;
		}
		Bodies.swap(m_WorkBodies);
		Contacts.swap(m_WorkContacts);
		if (!m_Statistics.UsedFallback) m_Statistics.Message.clear();
		if (Stage == PBDBackendStage::Position) m_Statistics.PositionDispatches += LayerCount;
		else if (Stage == PBDBackendStage::Reconstruct) m_Statistics.ReconstructionDispatches += LayerCount;
		else m_Statistics.VelocityDispatches += LayerCount;
		m_Statistics.ContactRows += Contacts.size();
		for (uint32_t Layer = 0; Layer < LayerCount; ++Layer)
			m_Statistics.MaxBatchSize = std::max(m_Statistics.MaxBatchSize, m_LayerOffsets[Layer + 1] - m_LayerOffsets[Layer]);
		return true;
	}
}
