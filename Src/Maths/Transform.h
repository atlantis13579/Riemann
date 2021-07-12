#pragma once

#include "Vector3d.h"
#include "Vector4d.h"
#include "Matrix3d.h"
#include "Matrix4d.h"
#include "Quaternion.h"

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
		m_Translation = Vector3d::Zero();
		m_Rotation = Quaternion::UnitW();
		m_Scale = Vector3d::One();
	}

	const Vector3d& GetTranslation() const
	{
		return m_Translation;
	}

	Matrix3d GetRotationMatrix() const
	{
		return m_Rotation.ToRotationMatrix();
	}

	const Quaternion& GetRotation() const
	{
		return m_Rotation;
	}

	const Vector3d& GetScale() const
	{
		return m_Scale;
	}

	void		SetTranslation(const Vector3d& trans)
	{
		m_Translation = trans;
		m_bWorldMatrixDirty = true;
		m_bInvWorldMatrixDirty = true;
	}

	void		SetRotation(const Quaternion rotation)
	{
		m_Rotation = rotation;
		m_bWorldMatrixDirty = true;
		m_bInvWorldMatrixDirty = true;
	}

	void		SetScale(const Vector3d& scale)
	{
		m_Scale = scale;
		m_bWorldMatrixDirty = true;
		m_bInvWorldMatrixDirty = true;
	}

	const Matrix4d& GetWorldMatrix()
	{
		if (m_bWorldMatrixDirty)
		{
			GetWorldMatrix(m_WorldMatrix, m_Translation, m_Rotation, m_Scale);
			m_bWorldMatrixDirty = false;
		}
		return m_WorldMatrix;
	}

	const Matrix4d& GetInverseWorldMatrix()
	{
		if (m_bInvWorldMatrixDirty)
		{
			GetInverseWorldMatrix(m_InvWorldMatrix, m_Translation, m_Rotation, m_Scale);
			m_bInvWorldMatrixDirty = false;
		}
		return m_InvWorldMatrix;
	}

	static Vector3d TransformPosition(const Matrix4d& mat, const Vector3d& Point)
	{
		Vector4d hSpace = Vector4d(Point.x, Point.y, Point.z, 1.0f);
		Vector4d t = hSpace * mat;
		return t.xyz();
	}

	Vector3d LocalToWorld(const Vector3d& Point)
	{
		const Matrix4d& mat = GetWorldMatrix();
		return TransformPosition(mat, Point);
	}

	Vector3d WorldToLocal(const Vector3d& Point)
	{
		const Matrix4d& mat = GetInverseWorldMatrix();
		return TransformPosition(mat, Point);
	}

	static Matrix4d BuildViewMatrix_RHCoordinateSystem(const Vector3d& Eye, const Vector3d& LookAt, const Vector3d& Up)
	{
		Vector3d zAxis = (Eye - LookAt).Unit();
		Vector3d xAxis = CrossProduct(Up, zAxis).Unit();
		Vector3d yAxis = CrossProduct(zAxis, xAxis).Unit();

		float w1 = -1 * DotProduct(xAxis, Eye);
		float w2 = -1 * DotProduct(yAxis, Eye);
		float w3 = -1 * DotProduct(zAxis, Eye);

		return Matrix4d(
			xAxis.x,	xAxis.y,	xAxis.z,	w1,
			yAxis.x,	yAxis.y,	yAxis.z,	w2,
			zAxis.x,	zAxis.y,	zAxis.z,	w3,
			0.0f,		0.0f,		0.0f,		1.0f
		);
	}

	static Matrix4d BuildViewMatrix_LHCoordinateSystem(const Vector3d& Eye, const Vector3d& LookAt, const Vector3d& Up)
	{
		Vector3d zAxis = (LookAt - Eye).Unit();
		Vector3d xAxis = CrossProduct(Up, zAxis).Unit();
		Vector3d yAxis = CrossProduct(zAxis, xAxis).Unit();

		float w1 = -1 * DotProduct(xAxis, Eye);
		float w2 = -1 * DotProduct(yAxis, Eye);
		float w3 = -1 * DotProduct(zAxis, Eye);

		return Matrix4d(
			xAxis.x,	xAxis.y,	xAxis.z,	w1,
			yAxis.x,	yAxis.y,	yAxis.z,	w2,
			zAxis.x,	zAxis.y,	zAxis.z,	w3,
			0.0f,		0.0f,		0.0f,		1.0f
		);
	}

	static Matrix4d BuildPerspectiveMatrix_LHCoordinateSystem(float Fov, float Aspect, float zNear, float zFar) {
		auto yscale = 1.0f / tanf(Fov * 0.5f);
		auto xscale = yscale / Aspect;

		return Matrix4d(
			xscale,		0.0f,		0.0f,					0.0f,
			0.0f,		yscale,		0.0f,					0.0f,
			0.0f,		0.0f,		zFar / (zFar - zNear),	(-zNear * zFar) / (zFar - zNear),
			0.0f,		0.0f,		1.0f,					0.0f
		);
	}

	static Matrix4d BuildPerspectiveMatrix_RHCoordinateSystem(float Fov, float Aspect, float zNear, float zFar) {
		auto yscale = 1.0f / tanf(Fov * 0.5f);
		auto xscale = yscale / Aspect;

		return Matrix4d(
			xscale,		0.0f,		0.0f,					0.0f,
			0.0f,		yscale,		0.0f,					0.0f,
			0.0f,		0.0f,		zFar / (zNear - zFar),	(zNear * zFar) / (zNear - zFar),
			0.0f,		0.0f,		-1.0f,					0.0f
		);
	}

	static Matrix4d BuildOrthogonalMatrix_LHCoordinateSystem(float Width, float Height, float zNear, float zFar) {
		return Matrix4d(
			2.0f / Width,	0.0f,				0.0f,					0.0f,
			0.0f,			2.0f / Height,		0.0f,					0.0f,
			0.0f,			0.0f,				1.0f / (zFar - zNear),	-zNear / (zFar - zNear),
			0.0f,			0.0f,				0.0f,					1.0f
		);
	}

	static Matrix4d BuildOrthogonalMatrix_RHCoordinateSystem(float Width, float Height, float zNear, float zFar) {
		return Matrix4d(
			2.0f / Width,	0.0f,				0.0f,					0.0f,
			0.0f,			2.0f / Height,		0.0f,					0.0f,
			0.0f,			0.0f,				1.0f / (zNear - zFar),	zNear / (zNear - zFar),
			0.0f,			0.0f,				0.0f,					1.0f
		);
	}

	static Matrix4d		BuildTranslationMatrix(const Vector3d& Trans)
	{
		return Matrix4d(
			1.0f,	0.0f,	0.0f,	Trans.x,
			0.0f,	1.0f,	0.0f,	Trans.y,
			0.0f,	0.0f,	1.0f,	Trans.z,
			0.0f,	0.0f,	0.0f,	1.0f
		);
	}

	static Matrix4d		BuildRotationMatrix_X(float theta)
	{
		float c = cosf(theta), s = sinf(theta);
		return Matrix4d(
			1.0f,	0.0f,	0.0f,	0.0f,
			0.0f,	c,		-s,		0.0f,
			0.0f,	s,		c,		0.0f,
			0.0f,	0.0f,	0.0f,	1.0f
		);
	}

	static Matrix4d		BuildRotationMatrix_Y(float theta)
	{
		float c = cosf(theta), s = sinf(theta);
		return Matrix4d(
			c,		0.0f,	s,		0.0f,
			0.0f,	1.0f,	0.0f,	0.0f,
			-s,		0.0f,	c,		0.0f,
			0.0f,	0.0f,	0.0f,	1.0f
		);
	}

	static Matrix4d		BuildRotationMatrix_Z(const float theta)
	{
		float c = cosf(theta), s = sinf(theta);
		return Matrix4d(
			c,		-s,		0.0f,	0.0f,
			s,		c,		0.0f,	0.0f,
			0.0f,	0.0f,	1.0f,	0.0f,
			0.0f,	0.0f,	0.0f,	1.0f
		);
	}

	static Matrix4d		BuildRotationMatrix_Euler(const float yaw, const float pitch, const float roll) {
		float cYaw = cosf(yaw);
		float cPitch = cosf(pitch);
		float cRoll = cosf(roll);
		float sYaw = sinf(yaw);
		float sPitch = sinf(pitch);
		float sRoll = sinf(roll);

		return Matrix4d(
			(cRoll * cYaw) + (sRoll * sPitch * sYaw), (-sRoll * cYaw) + (cRoll * sPitch * sYaw), (cPitch * sYaw), 0.0f,
			(sRoll * cPitch), (cRoll * cPitch), -sPitch, 0.0f,
			(cRoll * -sYaw) + (sRoll * sPitch * cYaw), (sRoll * sYaw) + (cRoll * sPitch * cYaw), (cPitch * cYaw), 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		);
	}

	static Matrix4d		BuildRotationMatrix_Quat(const Quaternion& Rot)
	{
		return Rot.ToRotationMatrix4d();
	}


	static Matrix4d		BuildMatrixRotation_Axis(const Vector3d& axis, const float radio) {
		float c = cosf(radio), s = sinf(radio), v = 1.0f - c;

		return Matrix4d(
			c + axis.x * axis.x * v,			axis.x * axis.y * v - axis.z * s,	axis.x * axis.z * v + axis.y * s,	0.0f,
			axis.x * axis.y * v + axis.z * s,	c + axis.y * axis.y * v,			axis.y * axis.z * v - axis.x * s,	0.0f,
			axis.x * axis.z * v - axis.y * s,   axis.y * axis.z * v + axis.x * s,	c + axis.z * axis.z * v,			0.0f,
			0.0f,								0.0f,								0.0f,								1.0f
		);
	}

	static Matrix4d		BuildScaleMatrix(const Vector3d& Scale)
	{
		return Matrix4d(
			Scale.x,	0.0f,		0.0f,		0.0f,
			0.0f,		Scale.y,	0.0f,		0.0f,
			0.0f,		0.0f,		Scale.z,	0.0f,
			0.0f,		0.0f,		0.0f,		1.0f
		);
	}

	static void			GetWorldMatrix(Matrix4d& World, const Vector3d& Translation, const Quaternion& Rotation, const Vector3d&	Scale)
	{
		Matrix4d matTrans = BuildTranslationMatrix(Translation);
		Matrix4d matScale = BuildScaleMatrix(Scale);
		Matrix4d matRot = Rotation.ToRotationMatrix4d();

		World = matTrans * matRot * matScale;			// make sure Translation Matrix go first.
	}

	static void			GetInverseWorldMatrix(Matrix4d& InvWorld, const Vector3d& Translation, const Quaternion& Rotation, const Vector3d& Scale)
	{
		Matrix4d matTrans = BuildTranslationMatrix(-Translation);
		Matrix4d matScale = BuildScaleMatrix(Vector3d::One() / Scale);
		Matrix4d matRot = Rotation.ToRotationMatrix4d().Transpose();

		InvWorld = matScale * matRot * matTrans;		// make sure Scale Matrix go first.
	}

private:
	Vector3d	m_Translation;
	Quaternion	m_Rotation;
	Vector3d	m_Scale;

	bool		m_bWorldMatrixDirty;
	bool		m_bInvWorldMatrixDirty;
	Matrix4d	m_WorldMatrix;
	Matrix4d	m_InvWorldMatrix;
};