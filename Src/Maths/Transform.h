#pragma once

#include "Vector3.h"
#include "Vector4.h"
#include "Matrix3.h"
#include "Matrix4.h"
#include "Quaternion.h"

namespace Maths
{
	struct Pose
	{
		Pose()
		{
			pos = Vector3::Zero();
			quat = Quaternion::One();
		}

		explicit Pose(const Vector3& _pos, const Quaternion& _quat = Quaternion::One())
		{
			pos = _pos;
			quat = _quat;
		}

		Pose(const Pose& _pose)
		{
			pos = _pose.pos;
			quat = _pose.quat;
		}

		Vector3		pos;
		Quaternion	quat;
	};

	class Transform
	{
	public:
		Transform()
		{
			LoadIdentity();
		}

		void				LoadIdentity()
		{
			m_bWorldMatrixDirty = true;
			m_bInvWorldMatrixDirty = true;
			m_Translation = Vector3::Zero();
			m_Rotation = Quaternion::One();
			m_Scale = Vector3::One();
		}

		const Vector3& GetTranslation() const
		{
			return m_Translation;
		}

		Matrix3				GetRotationMatrix() const
		{
			return m_Rotation.ToRotationMatrix3();
		}

		const Quaternion& GetRotation() const
		{
			return m_Rotation;
		}

		const Vector3& GetScale() const
		{
			return m_Scale;
		}

		void				SetTranslation(const Vector3& trans)
		{
			m_Translation = trans;
			m_bWorldMatrixDirty = true;
			m_bInvWorldMatrixDirty = true;
		}

		void				SetRotation(const Quaternion& rotation)
		{
			m_Rotation = rotation;
			m_bWorldMatrixDirty = true;
			m_bInvWorldMatrixDirty = true;
		}

		void				SetScale(const Vector3& scale)
		{
			m_Scale = scale;
			m_bWorldMatrixDirty = true;
			m_bInvWorldMatrixDirty = true;
		}

		const Matrix4& GetWorldMatrix()
		{
			if (m_bWorldMatrixDirty)
			{
				TRSToWorldMatrix(m_WorldMatrix, m_Translation, m_Rotation, m_Scale);
				m_bWorldMatrixDirty = false;
			}
			return m_WorldMatrix;
		}

		const Matrix4& GetInverseWorldMatrix()
		{
			if (m_bInvWorldMatrixDirty)
			{
				TRSToInverseWorldMatrix(m_InvWorldMatrix, m_Translation, m_Rotation, m_Scale);
				m_bInvWorldMatrixDirty = false;
			}
			return m_InvWorldMatrix;
		}

		Vector3			LocalToWorld(const Vector3& Point)
		{
			const Matrix4& mat = GetWorldMatrix();
			return mat * Point;
		}

		Vector3			LocalToWorldEx(const Vector3& Point) const
		{
			return m_Rotation * Point + m_Translation;
		}

		Vector3			LocalToWorldDirection(const Vector3& Direction) const
		{
			return m_Rotation * Direction;
		}

		Vector3			WorldToLocal(const Vector3& Point)
		{
			const Matrix4& mat = GetInverseWorldMatrix();
			return mat * Point;
		}

		Vector3			WorldToLocalEx(const Vector3& Point) const
		{
			return m_Rotation.Conjugate() * (Point - m_Translation);
		}

		Vector3			WorldToLocalDirection(const Vector3& Direction) const
		{
			return m_Rotation.Conjugate() * Direction;
		}

		static Matrix4		BuildViewMatrix_RHCoordinateSystem(const Vector3& Eye, const Vector3& LookAt, const Vector3& Up)
		{
			Vector3 zAxis = (Eye - LookAt).Unit();
			Vector3 xAxis = CrossProduct(Up, zAxis).Unit();
			Vector3 yAxis = CrossProduct(zAxis, xAxis).Unit();

			float w1 = -1.0f * DotProduct(xAxis, Eye);
			float w2 = -1.0f * DotProduct(yAxis, Eye);
			float w3 = -1.0f * DotProduct(zAxis, Eye);

			return Matrix4(
				xAxis.x, xAxis.y, xAxis.z, w1,
				yAxis.x, yAxis.y, yAxis.z, w2,
				zAxis.x, zAxis.y, zAxis.z, w3,
				0.0f, 0.0f, 0.0f, 1.0f
			);
		}

		static Matrix4		BuildViewMatrix_LHCoordinateSystem(const Vector3& Eye, const Vector3& LookAt, const Vector3& Up)
		{
			Vector3 zAxis = (LookAt - Eye).Unit();
			Vector3 xAxis = CrossProduct(Up, zAxis).Unit();
			Vector3 yAxis = CrossProduct(zAxis, xAxis).Unit();

			float w1 = -1 * DotProduct(xAxis, Eye);
			float w2 = -1 * DotProduct(yAxis, Eye);
			float w3 = -1 * DotProduct(zAxis, Eye);

			return Matrix4(
				xAxis.x, xAxis.y, xAxis.z, w1,
				yAxis.x, yAxis.y, yAxis.z, w2,
				zAxis.x, zAxis.y, zAxis.z, w3,
				0.0f, 0.0f, 0.0f, 1.0f
			);
		}

		static Matrix4		BuildPerspectiveMatrix_LHCoordinateSystem(float Fov, float Aspect, float zNear, float zFar) {
			auto yscale = 1.0f / tanf(Fov * 0.5f);
			auto xscale = yscale / Aspect;

			return Matrix4(
				xscale, 0.0f, 0.0f, 0.0f,
				0.0f, yscale, 0.0f, 0.0f,
				0.0f, 0.0f, zFar / (zFar - zNear), (-zNear * zFar) / (zFar - zNear),
				0.0f, 0.0f, 1.0f, 0.0f
			);
		}

		static Matrix4		BuildPerspectiveMatrix_RHCoordinateSystem(float Fov, float Aspect, float zNear, float zFar) {
			auto yscale = 1.0f / tanf(Fov * 0.5f);
			auto xscale = yscale / Aspect;

			return Matrix4(
				xscale, 0.0f, 0.0f, 0.0f,
				0.0f, yscale, 0.0f, 0.0f,
				0.0f, 0.0f, zFar / (zNear - zFar), (zNear * zFar) / (zNear - zFar),
				0.0f, 0.0f, -1.0f, 0.0f
			);
		}

		static Matrix4		BuildOrthogonalMatrix_LHCoordinateSystem(float Width, float Height, float zNear, float zFar) {
			return Matrix4(
				2.0f / Width, 0.0f, 0.0f, 0.0f,
				0.0f, 2.0f / Height, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f / (zFar - zNear), -zNear / (zFar - zNear),
				0.0f, 0.0f, 0.0f, 1.0f
			);
		}

		static Matrix4		BuildOrthogonalMatrix_RHCoordinateSystem(float Width, float Height, float zNear, float zFar) {
			return Matrix4(
				2.0f / Width, 0.0f, 0.0f, 0.0f,
				0.0f, 2.0f / Height, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f / (zNear - zFar), zNear / (zNear - zFar),
				0.0f, 0.0f, 0.0f, 1.0f
			);
		}

		static Matrix4		BuildTranslationMatrix(const Vector3& Trans)
		{
			return Matrix4(
				1.0f, 0.0f, 0.0f, Trans.x,
				0.0f, 1.0f, 0.0f, Trans.y,
				0.0f, 0.0f, 1.0f, Trans.z,
				0.0f, 0.0f, 0.0f, 1.0f
			);
		}

		static Matrix4		BuildRotationMatrix_X(float theta)
		{
			float c = cosf(theta), s = sinf(theta);
			return Matrix4(
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, c, -s, 0.0f,
				0.0f, s, c, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			);
		}

		static Matrix4		BuildRotationMatrix_Y(float theta)
		{
			float c = cosf(theta), s = sinf(theta);
			return Matrix4(
				c, 0.0f, s, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				-s, 0.0f, c, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			);
		}

		static Matrix4		BuildRotationMatrix_Z(const float theta)
		{
			float c = cosf(theta), s = sinf(theta);
			return Matrix4(
				c, -s, 0.0f, 0.0f,
				s, c, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			);
		}

		static Matrix4		BuildRotationMatrix_Euler(const float yaw, const float pitch, const float roll) {
			float cYaw = cosf(yaw);
			float cPitch = cosf(pitch);
			float cRoll = cosf(roll);
			float sYaw = sinf(yaw);
			float sPitch = sinf(pitch);
			float sRoll = sinf(roll);

			return Matrix4(
				(cRoll * cYaw) + (sRoll * sPitch * sYaw), (-sRoll * cYaw) + (cRoll * sPitch * sYaw), (cPitch * sYaw), 0.0f,
				(sRoll * cPitch), (cRoll * cPitch), -sPitch, 0.0f,
				(cRoll * -sYaw) + (sRoll * sPitch * cYaw), (sRoll * sYaw) + (cRoll * sPitch * cYaw), (cPitch * cYaw), 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			);
		}

		static Matrix4		BuildRotationMatrix_Quat(const Quaternion& Rot)
		{
			return Rot.ToRotationMatrix4();
		}


		static Matrix4		BuildMatrixRotation_Axis(const Vector3& axis, const float radio) {
			float c = cosf(radio), s = sinf(radio), v = 1.0f - c;

			return Matrix4(
				c + axis.x * axis.x * v, axis.x * axis.y * v - axis.z * s, axis.x * axis.z * v + axis.y * s, 0.0f,
				axis.x * axis.y * v + axis.z * s, c + axis.y * axis.y * v, axis.y * axis.z * v - axis.x * s, 0.0f,
				axis.x * axis.z * v - axis.y * s, axis.y * axis.z * v + axis.x * s, c + axis.z * axis.z * v, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			);
		}

		static Matrix4		BuildScaleMatrix(const Vector3& Scale)
		{
			return Matrix4(
				Scale.x, 0.0f, 0.0f, 0.0f,
				0.0f, Scale.y, 0.0f, 0.0f,
				0.0f, 0.0f, Scale.z, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			);
		}

		static void			TRSToWorldMatrix(Matrix4& World, const Vector3& Translation, const Quaternion& Rotation, const Vector3& Scale)
		{
			Matrix4 matTrans = BuildTranslationMatrix(Translation);
			Matrix4 matScale = BuildScaleMatrix(Scale);
			Matrix4 matRot = Rotation.ToRotationMatrix4();

			World = matTrans * matRot * matScale;			// make sure Translation Matrix go first.
		}

		static void			TRToWorldMatrix(Matrix4& World, const Vector3& Translation, const Quaternion& Rotation)
		{
			World = Rotation.ToRotationMatrix4();
			World[0][3] += Translation.x;
			World[1][3] += Translation.y;
			World[2][3] += Translation.z;
		}

		static void			TRSToInverseWorldMatrix(Matrix4& InvWorld, const Vector3& Translation, const Quaternion& Rotation, const Vector3& Scale)
		{
			Matrix4 matTrans = BuildTranslationMatrix(-Translation);
			Matrix4 matScale = BuildScaleMatrix(Vector3::One() / Scale);
			Matrix4 matRot = Rotation.ToRotationMatrix4().Transpose();

			InvWorld = matScale * matRot * matTrans;		// make sure Scale Matrix go first.
		}

		static void			TRToInverseWorldMatrix(Matrix4& InvWorld, const Vector3& Translation, const Quaternion& Rotation)
		{
			InvWorld = Rotation.ToRotationMatrix4().Transpose();
			InvWorld[0][3] -= Translation.x;
			InvWorld[1][3] -= Translation.y;
			InvWorld[2][3] -= Translation.z;
		}

		static void			WorldMatrixToTR(const Matrix4& World, Vector3& Translation, Quaternion& Rotation)
		{
			Translation = Vector3(World[0][3], World[1][3], World[2][3]);
			Matrix3 mat3(World[0][0], World[0][1], World[0][2], World[1][0], World[1][1], World[1][2], World[2][0], World[2][1], World[2][2]);
			Rotation.FromRotationMatrix3(mat3);
		}

		// For only non-composite scale transform
		static void			WorldMatrixToTRS(const Matrix4& World, Vector3& Translation, Quaternion& Rotation, Vector3& Scale)
		{
			Translation = Vector3(World[0][3], World[1][3], World[2][3]);
			float sx = Vector3(World[0][0], World[1][0], World[2][0]).Length();
			float sy = Vector3(World[0][1], World[1][1], World[2][1]).Length();
			float sz = Vector3(World[0][2], World[1][2], World[2][2]).Length();
			Matrix3 matRot(World[0][0] / sx, World[0][1] / sy, World[0][2] / sz,
				World[1][0] / sx, World[1][1] / sy, World[1][2] / sz,
				World[2][0] / sx, World[2][1] / sy, World[2][2] / sz);
			Rotation.FromRotationMatrix3(matRot);
			Scale = Vector3(sx, sy, sz);
			return;
		}

		static Vector3		ForwardVector(const Quaternion& quat)
		{
			return quat * Vector3::UnitZ();
		}

		static Vector3		UpVector(const Quaternion& quat)
		{
			return quat * Vector3::UnitY();
		}

	private:
		Vector3		m_Translation;
		Quaternion	m_Rotation;
		Vector3		m_Scale;

		bool		m_bWorldMatrixDirty;
		bool		m_bInvWorldMatrixDirty;
		Matrix4		m_WorldMatrix;
		Matrix4		m_InvWorldMatrix;
	};
}

using Pose = Maths::Pose;
using Transform = Maths::Transform;