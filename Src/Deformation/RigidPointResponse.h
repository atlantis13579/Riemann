#pragma once

#include <cstddef>
#include "../Maths/Quaternion.h"

namespace Riemann
{
	struct ConstraintMassMatrix
	{
		using DVector = Maths::TVector3<double>;
		double Values[3][3] = {};
		double* operator[](size_t Row) { return Values[Row]; }
		const double* operator[](size_t Row) const { return Values[Row]; }
		DVector operator*(const DVector& Vector) const;
		double MaximumMagnitude() const;
	};
	struct RigidResponseSettings
	{
		double RelativeTolerance = 1e-12;
		double InertiaTolerance = 1e-6;
		double QuaternionTolerance = 1e-4;
		double MaximumRotation = 0.7853981633974483;
		int MaximumDiagonalIterations = 32;
		bool IsValid() const;
	};
	struct RigidConstraintBody
	{
		Vector3* Center = nullptr;
		Quaternion* Rotation = nullptr;
		double InverseMass = 0;
		Matrix3 LocalInverseInertia = Matrix3::Zero();
	};
	class RigidPointResponse
	{
	public:
		using DVector = ConstraintMassMatrix::DVector;
		struct Evaluation
		{
			DVector Point = DVector(0), Arm = DVector(0);
			ConstraintMassMatrix InverseInertia, InversePointMass;
		};
		struct Correction
		{
			Vector3 Center = Vector3::Zero();
			Quaternion Rotation = Quaternion::One();
			DVector AngularMultiplier = DVector(0);
		};
		RigidPointResponse() = default;
		RigidPointResponse(const RigidConstraintBody& Body, const Vector3& LocalArm,
			const RigidResponseSettings& Settings = RigidResponseSettings());
		Evaluation Evaluate() const;
		double StepScale(const Evaluation& State, const DVector& PositionImpulse) const;
		Correction Prepare(const Evaluation& State, const DVector& PositionImpulse) const;
		void Commit(const Correction& Next) const;
		DVector SolveMass(const ConstraintMassMatrix& Matrix, const DVector& Right) const;
		const Vector3* GetCenterAddress() const { return m_Body.Center; }
	private:
		RigidConstraintBody m_Body;
		Vector3 m_LocalArm = Vector3::Zero();
		ConstraintMassMatrix m_LocalInverseInertia;
		RigidResponseSettings m_Settings;
	};
}
