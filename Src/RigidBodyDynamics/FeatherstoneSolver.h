#pragma once

#include <array>
#include <vector>
#include "../Maths/Transform.h"
#include "../LinearSystem/DenseMatrix.h"

namespace Riemann
{
	using RagdollMatrix = Maths::LinearAlgebra::TDenseMatrix<double>;
	struct SpatialVector
	{
		static constexpr int Dimension = 6;
		std::array<double, Dimension> Value{};
		SpatialVector() = default;
		SpatialVector(const Vector3& Angular, const Vector3& Linear);
		Vector3 GetAngular() const;
		Vector3 GetLinear() const;
		double& operator[](int Index) { return Value[(size_t)Index]; }
		double operator[](int Index) const { return Value[(size_t)Index]; }
		SpatialVector operator+(const SpatialVector& Other) const;
		SpatialVector operator-(const SpatialVector& Other) const;
		SpatialVector operator*(double Scale) const;
		SpatialVector& operator+=(const SpatialVector& Other);
		double Dot(const SpatialVector& Other) const;
		bool IsFinite() const;
	};
	struct SpatialMatrix
	{
		std::array<double, SpatialVector::Dimension * SpatialVector::Dimension> Value{};
		double& operator()(int Row, int Column) { return Value[(size_t)Row * SpatialVector::Dimension + Column]; }
		double operator()(int Row, int Column) const { return Value[(size_t)Row * SpatialVector::Dimension + Column]; }
		SpatialVector operator*(const SpatialVector& Vector) const;
		SpatialMatrix operator*(const SpatialMatrix& Other) const;
		SpatialMatrix Transpose() const;
		SpatialMatrix& operator+=(const SpatialMatrix& Other);
	};
	enum class RagdollReducedJointType { Fixed, Revolute, Prismatic, Spherical, Floating };
	struct RagdollLinkSettings
	{
		int Parent = -1;
		RagdollReducedJointType Joint = RagdollReducedJointType::Fixed;
		Transform ParentToJoint;
		Vector3 Axis = Vector3::UnitZ();
		Vector3 ConeAxisParent = Vector3::UnitZ(), ConeAxisChild = Vector3::UnitZ();
		double ConeLimit = 3.14159265358979323846;
		Vector3 CenterOfMass = Vector3::Zero();
		float Mass = 1;
		Matrix3 Inertia = Matrix3(1, 1, 1);
		double Armature = 0;
		double Damping = 0;
	};
	struct RagdollConfiguration
	{
		double Position = 0;
		Quaternion Rotation = Quaternion::One();
		Vector3 Translation = Vector3::Zero();
	};
	struct FeatherstoneSettings
	{
		double RelativePivotTolerance = 1.0e-10;
		float SymmetryTolerance = 1.0e-6f;
	};
	class FeatherstoneSolver
	{
	public:
		FeatherstoneSolver(const std::vector<RagdollLinkSettings>& Links, const FeatherstoneSettings& Settings = FeatherstoneSettings());
		size_t GetLinkCount() const { return m_Links.size(); }
		int GetDofCount() const { return (int)m_Velocity.size(); }
		int GetLinkDofCount(size_t Index) const;
		int GetLinkDofOffset(size_t Index) const { return m_Offsets.at(Index); }
		const RagdollLinkSettings& GetLinkSettings(size_t Index) const { return m_Links.at(Index); }
		const std::vector<RagdollConfiguration>& GetConfiguration() const { return m_Configuration; }
		const std::vector<double>& GetVelocity() const { return m_Velocity; }
		void SetConfiguration(const std::vector<RagdollConfiguration>& Configuration);
		void SetVelocity(const std::vector<double>& Velocity);
		Transform GetLinkTransform(size_t Index) const { return m_Cache.at(Index).World; }
		Vector3 GetCenterOfMass(size_t Index) const;
		SpatialVector GetLinkVelocity(size_t Index) const;
		std::vector<SpatialVector> ComputeLinkVelocities(const std::vector<double>& Velocity) const;
		RagdollMatrix ComputeLinkJacobian(size_t Index, const Vector3& LinkPoint = Vector3::Zero()) const;
		RagdollMatrix ComputeDenseJacobian(bool AtCenterOfMass = true) const;
		RagdollMatrix ComputeMassMatrix() const;
		std::vector<double> ComputeGeneralizedForce(const std::vector<SpatialVector>& WorldWrenches) const;
		std::vector<double> ComputeBiasForce(const Vector3& Gravity = Vector3::Zero()) const;
		std::vector<double> ComputeAcceleration(const std::vector<double>& Torque, const Vector3& Gravity,
			const std::vector<SpatialVector>& ExternalWrenches = {});
		std::vector<double> ComputeImpulseResponse(const std::vector<SpatialVector>& LinkImpulses,
			const std::vector<double>& JointImpulses = {});
		std::vector<double> InverseDynamics(const std::vector<double>& Acceleration, const Vector3& Gravity,
			const std::vector<SpatialVector>& ExternalWrenches = {}) const;
		void IntegrateVelocity(const std::vector<double>& Acceleration, double Dt);
		void IntegrateConfiguration(double Dt);
		double GetKineticEnergy() const;
		double GetPotentialEnergy(const Vector3& Gravity) const;
		SpatialVector GetMomentum() const;
		size_t GetFactorizationCount() const { return m_FactorizationCount; }
	private:
		struct LinkCache
		{
			Transform World;
			SpatialMatrix MotionTransform, Inertia, ReducedRagdollInertia, ReducedInertia, JointInverseInertia;
			std::array<SpatialVector, SpatialVector::Dimension> Subspace{}, InertiaSubspace{};
			SpatialVector Velocity, BiasAcceleration;
		};
		void UpdateKinematics();
		void UpdateVelocities();
		void FillLinkJacobian(size_t Index, const Vector3& LinkPoint, RagdollMatrix& Matrix, int RowOffset) const;
		void Factorize();
		std::vector<double> Solve(const std::vector<double>& GeneralizedForce, const Vector3& Gravity,
			const std::vector<SpatialVector>& ExternalWrenches, bool IncludeBias);
		void ValidateVector(const std::vector<double>& Value) const;
		void ValidateWrenches(const std::vector<SpatialVector>& Value) const;
		std::vector<RagdollLinkSettings> m_Links;
		FeatherstoneSettings m_Settings;
		std::vector<RagdollConfiguration> m_Configuration;
		std::vector<double> m_Velocity;
		std::vector<int> m_Offsets;
		std::vector<LinkCache> m_Cache;
		std::vector<SpatialVector> m_WorkForce, m_WorkAcceleration, m_WorkJoint;
		bool m_FactorizationValid = false;
		size_t m_FactorizationCount = 0;
	};
}
