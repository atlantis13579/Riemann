#pragma once

#include "Vector3.h"
#include "Vector4.h"
#include "Matrix3.h"
#include "Matrix4.h"
#include "Quaternion.h"

namespace Maths
{
	struct Transform
	{
		Vector3		pos;
		Quaternion	quat;

		Transform()
		{
			pos = Vector3::Zero();
			quat = Quaternion::One();
		}

		explicit Transform(const Vector3& _pos)
		{
			pos = _pos;
			quat = Quaternion::One();
		}

		explicit Transform(const Quaternion& _quat)
		{
			pos = Vector3::Zero();
			quat = _quat;
		}

		Transform(const Vector3& _pos, const Quaternion& _quat)
		{
			pos = _pos;
			quat = _quat;
		}

		Transform(const Transform& rhs)
		{
			pos = rhs.pos;
			quat = rhs.quat;
		}

		inline Transform& operator=(const Transform& rhs)
		{
			pos = rhs.pos;
			quat = rhs.quat;
			return *this;
		}

		inline Transform operator+(const Transform& rhs) const
		{
			return Transform(pos + rhs.pos, quat * rhs.quat);
		}

		inline Transform operator-(const Transform& rhs) const
		{
			return Transform(pos - rhs.pos, quat * rhs.quat.Conjugate());
		}

		inline Transform operator*(const Transform& rhs) const
		{
			return Transform(pos + quat * rhs.pos, quat * rhs.quat);
		}

		inline Vector3 operator*(const Vector3& v) const
		{
			return pos + quat * v;
		}

		inline void	operator+= (const Transform& rhs)
		{
			*this = (*this) + rhs;
		}

		inline void	operator-= (const Transform& rhs)
		{
			*this = (*this) - rhs;
		}

		inline void	operator*= (const Transform& rhs)
		{
			*this = (*this) * rhs;
		}

		inline Transform TransformForward(const Transform& rhs) const
		{
			return (*this) * rhs;
		}

		inline Transform TransformInv(const Transform& rhs) const
		{
			Quaternion qinv = quat.Conjugate();
			return Transform(qinv * (rhs.pos - pos), qinv * rhs.quat);
		}

		inline Vector3	TransformPos(const Vector3& Point) const
		{
			return LocalToWorld(Point);
		}

		inline Vector3	TransformDirection(const Vector3& Direction) const
		{
			return LocalToWorldDirection(Direction);
		}

		inline Vector3	TransformPosInv(const Vector3& Point) const
		{
			return WorldToLocal(Point);
		}

		inline Vector3	TransformDirectionInv(const Vector3& Direction) const
		{
			return WorldToLocalDirection(Direction);
		}

		inline Vector3	LocalToWorld(const Vector3& Point) const
		{
			return quat * Point + pos;
		}

		inline Vector3	LocalToWorldDirection(const Vector3& Direction) const
		{
			return quat * Direction;
		}

		inline Vector3	WorldToLocal(const Vector3& Point) const
		{
			return quat.Conjugate() * (Point - pos);
		}

		inline Vector3	WorldToLocalDirection(const Vector3& Direction) const
		{
			return quat.Conjugate() * Direction;
		}

		Matrix3			GetRotationMatrix() const
		{
			return quat.ToRotationMatrix3();
		}

		inline Vector3	GetForwardVector() const
		{
			return quat * Vector3::UnitZ();
		}

		inline Vector3	GetUpVector() const
		{
			return quat * Vector3::UnitY();
		}

		inline Vector3	GetLeftVector() const
		{
			return quat * Vector3::UnitX();
		}

		inline Vector3	GetRightVector() const
		{
			return quat * -Vector3::UnitX();
		}


		static Transform	Identity()
		{
			return Transform(Vector3::Zero(), Quaternion::One());
		}
	};

	// Transformation between object 1's local space and object 2's local space
	struct Transform2
	{
		Transform	transform1;
		Transform	transform2;

		Transform2(const Transform* t1, const Transform* t2)
		{
			transform1.quat = t1->quat;
			transform2.quat = t2->quat;
			transform1.pos = t1->pos;
			transform2.pos = t2->pos;
		}

		Vector3		Local1ToLocal2(const Vector3& Point) const
		{
			return transform2.quat.Conjugate() * (transform1.quat * Point + transform1.pos - transform2.pos);
		}

		Vector3		Local1ToLocal2Direction(const Vector3& Direction) const
		{
			Quaternion quat = transform1.quat * transform2.quat.Conjugate();
			return quat * Direction;
		}

		Matrix3		Local1ToLocal2RotationMatrix() const
		{
			Quaternion quat = transform1.quat * transform2.quat.Conjugate();
			return quat.ToRotationMatrix3();
		}

		Vector3		Local2ToLocal1(const Vector3& Point) const
		{
			return transform1.quat.Conjugate() * (transform2.quat * Point + transform2.pos - transform1.pos);
		}

		Vector3		Local2ToLocal1Direction(const Vector3& Direction) const
		{
			Quaternion quat = transform2.quat * transform1.quat.Conjugate();
			return quat * Direction;
		}

		Matrix3		Local2ToLocal1RotationMatrix() const
		{
			Quaternion quat = transform2.quat * transform1.quat.Conjugate();
			return quat.ToRotationMatrix3();
		}

		Vector3		Local1ToWorld(const Vector3& Point) const
		{
			return transform1.quat * Point + transform1.pos;
		}

		Vector3		Local1ToWorldDirection(const Vector3& Direction) const
		{
			return transform1.quat * Direction;
		}

		Matrix3		Local1ToWorldRotationMatrix() const
		{
			return transform1.quat.ToRotationMatrix3();
		}

		Vector3		Local2ToWorld(const Vector3& Point) const
		{
			return transform2.quat * Point + transform2.pos;
		}

		Vector3		Local2ToWorldDirection(const Vector3& Direction) const
		{
			return transform2.quat * Direction;
		}

		Matrix3		Local2ToWorldRotationMatrix() const
		{
			return transform2.quat.ToRotationMatrix3();
		}
	};

	class Transform3
	{
	public:
		Transform3()
		{
			LoadIdentity();
		}

		void				LoadIdentity()
		{
			world_matrix_is_dirty = true;
			inv_world_matrix_is_dirty = true;
			pos = Vector3::Zero();
			quat = Quaternion::One();
			scale = Vector3::One();
		}

		void				LoadIdentityEx()
		{
			LoadIdentity();

			GetWorldMatrix();
			GetInverseWorldMatrix();
		}

		const Vector3& GetTranslation() const
		{
			return pos;
		}

		Matrix3			GetRotationMatrix() const
		{
			return quat.ToRotationMatrix3();
		}

		const Quaternion& GetRotation() const
		{
			return quat;
		}

		const Vector3& GetScale() const
		{
			return scale;
		}

		void				SetTranslation(const Vector3& trans)
		{
			pos = trans;
			world_matrix_is_dirty = true;
			inv_world_matrix_is_dirty = true;
		}

		void				SetRotation(const Quaternion& rotation)
		{
			quat = rotation;
			world_matrix_is_dirty = true;
			inv_world_matrix_is_dirty = true;
		}

		void				SetScale(const Vector3& _scale)
		{
			scale = _scale;
			world_matrix_is_dirty = true;
			inv_world_matrix_is_dirty = true;
		}

		const Matrix4& GetWorldMatrix()
		{
			if (world_matrix_is_dirty)
			{
				TRSToWorldMatrix(world_matrix, pos, quat, scale);
				world_matrix_is_dirty = false;
			}
			return world_matrix;
		}

		const Matrix4& GetInverseWorldMatrix()
		{
			if (inv_world_matrix_is_dirty)
			{
				TRSToInverseWorldMatrix(inv_world_matrix, pos, quat, scale);
				inv_world_matrix_is_dirty = false;
			}
			return inv_world_matrix;
		}

		Vector3			LocalToWorld(const Vector3& Point)
		{
			const Matrix4& mat = GetWorldMatrix();
			Vector4 v = mat * Vector4(Point.x, Point.y, Point.z, 1.0f);
			return Vector3(v.x, v.y, v.z);
		}

		Vector3			TransformScale(const Vector3& Point) const
		{
			return quat.Conjugate() * (scale * (quat * Point));
		}

		Vector3			TransformScaleInv(const Vector3& Point) const
		{
			return quat * (Vector3(1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z) * (quat.Conjugate() * Point));
		}

		Vector3			LocalToWorldEx(const Vector3& Point) const
		{
			Vector3 scaled_point = TransformScale(Point);
			return quat * scaled_point + pos;
		}

		Vector3			LocalToWorldDirection(const Vector3& Direction) const
		{
			return quat * Direction;
		}

		Vector3			WorldToLocal(const Vector3& Point)
		{
			const Matrix4& mat = GetInverseWorldMatrix();
			Vector4 v = mat * Vector4(Point.x, Point.y, Point.z, 1.0f);
			return Vector3(v.x, v.y, v.z);
		}

		Vector3			WorldToLocalEx(const Vector3& Point) const
		{
			Vector3 scaled_point = TransformScaleInv(Point);
			return quat.Conjugate() * (scaled_point - pos);
		}

		Vector3			WorldToLocalDirection(const Vector3& Direction) const
		{
			return quat.Conjugate() * Direction;
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

		static Vector3		LeftVector(const Quaternion& quat)
		{
			return quat * Vector3::UnitX();
		}

		static Vector3		RightVector(const Quaternion& quat)
		{
			return quat * -Vector3::UnitX();
		}

		static Transform3	Identity()
		{
			Transform3 trans;
			trans.LoadIdentity();
			return trans;
		}

	private:
		Vector3		pos;
		Quaternion	quat;
		Vector3		scale;

		bool		world_matrix_is_dirty;
		bool		inv_world_matrix_is_dirty;
		Matrix4		world_matrix;
		Matrix4		inv_world_matrix;
	};
}

using Transform = Maths::Transform;
using Transform2 = Maths::Transform2;
using Transform3 = Maths::Transform3;