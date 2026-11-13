#pragma once

#include <cstdint>
#include <memory>
#include "../Maths/Transform.h"

namespace Riemann
{
	struct PhysicsFieldNode;
	class PhysicsFieldSystem;
	class PhysicsWorld;
	class RigidBody;
	enum class FieldFalloff { Linear, Smooth, Constant };
	class ScalarField
	{
	public:
		ScalarField();
		static ScalarField Constant(double Value);
		static ScalarField Radial(const Vector3& Center, double InnerRadius, double OuterRadius, FieldFalloff Falloff = FieldFalloff::Smooth);
		static ScalarField Box(const Transform& Pose, const Vector3& InnerExtent, const Vector3& OuterExtent, FieldFalloff Falloff = FieldFalloff::Smooth);
		static ScalarField Wave(double Offset, double Amplitude, double FrequencyHz, double Phase = 0);
		static ScalarField Add(const ScalarField& A, const ScalarField& B);
		static ScalarField Multiply(const ScalarField& A, const ScalarField& B);
		static ScalarField Minimum(const ScalarField& A, const ScalarField& B);
		static ScalarField Maximum(const ScalarField& A, const ScalarField& B);
		static ScalarField Clamp(const ScalarField& A, double Lower = 0, double Upper = 1);
		double Evaluate(const Vector3& Position, double Time = 0) const;
	private:
		friend class VectorField;
		friend class PhysicsFieldSystem;
		explicit ScalarField(std::shared_ptr<const PhysicsFieldNode> Node);
		std::shared_ptr<const PhysicsFieldNode> m_Node;
	};
	class VectorField
	{
	public:
		VectorField();
		static VectorField Constant(const Vector3& Value);
		static VectorField Radial(const Vector3& Center);
		static VectorField Add(const VectorField& A, const VectorField& B);
		static VectorField Scale(const VectorField& A, const ScalarField& Scale);
		static VectorField Cross(const VectorField& A, const VectorField& B);
		static VectorField Normalize(const VectorField& A);
		Vector3 Evaluate(const Vector3& Position, double Time = 0) const;
	private:
		friend class PhysicsFieldSystem;
		explicit VectorField(std::shared_ptr<const PhysicsFieldNode> Node);
		std::shared_ptr<const PhysicsFieldNode> m_Node;
	};
	enum class PhysicsFieldTarget { Force, Acceleration, Torque, Impulse, LinearVelocity, AngularVelocity, Activation, Strain, Material };
	enum class PhysicsFieldActivation { Wake, Sleep, Freeze, Unfreeze };
	struct PhysicsFieldCommand
	{
		PhysicsFieldTarget Target = PhysicsFieldTarget::Force;
		bool Persistent = false;
		uint32_t LayerMask = UINT32_MAX;
		double MinimumMask = 0;
		ScalarField Mask = ScalarField::Constant(1);
		ScalarField Scalar;
		VectorField Vector;
		PhysicsFieldActivation Activation = PhysicsFieldActivation::Wake;
		ScalarField StaticFriction = ScalarField::Constant(0.5);
		ScalarField DynamicFriction = ScalarField::Constant(0.3);
		ScalarField Restitution;
	};
	struct PhysicsFieldStats
	{
		uint64_t Samples = 0, EvaluatedNodes = 0, BodyApplications = 0, BrokenBonds = 0;
		double Time = 0;
	};
	class PhysicsFieldSystem
	{
	public:
		explicit PhysicsFieldSystem(PhysicsWorld& World);
		~PhysicsFieldSystem();
		PhysicsFieldSystem(const PhysicsFieldSystem&) = delete;
		PhysicsFieldSystem& operator=(const PhysicsFieldSystem&) = delete;
		uint64_t Add(const PhysicsFieldCommand& Command);
		bool Remove(uint64_t Id);
		void Clear();
		size_t GetCount() const;
		const PhysicsFieldStats& GetStats() const;
	private:
		friend class PhysicsWorld;
		void BeginFrame();
		void ApplyBoundary();
		void ApplyForces();
		void FinishSubstep(float Dt);
		void EndFrame();
		void Reset();
		struct Storage;
		std::unique_ptr<Storage> m_Data;
	};
}
