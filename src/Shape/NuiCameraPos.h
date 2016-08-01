#pragma once

#include <Eigen/Geometry>
#include "Foundation/NuiFileIOUtilities.h"

typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
typedef Eigen::Matrix<float, 4, 4> Matrix4frm;
typedef Eigen::Vector3f Vector3f;

struct NuiCameraIntrinsics
{
	float m_fx;
	float m_fy;
	float m_cx;
	float m_cy;

	NuiCameraIntrinsics()
		: m_cx(0.0f)
		, m_cy(0.0f)
		, m_fx(0.0f)
		, m_fy(0.0f)
	{
	}
	NuiCameraIntrinsics(float fx, float fy, float cx, float cy)
		: m_fx(fx)
		, m_fy(fy)
		, m_cx(cx)
		, m_cy(cy)
	{
	}
};

class NuiCameraPos
{
public:
	NuiCameraPos()
	{
		m_rotation.setIdentity();
		m_translation = Vector3f::Zero();
	}
	~NuiCameraPos(){}

	void setRotation(const Matrix3frm& rot)
	{
		m_rotation = rot;
	}
	const Matrix3frm& getRotation() const
	{
		return m_rotation;
	}

	void setTranslation(const Vector3f& tran)
	{
		m_translation = tran;
	}
	const Vector3f& getTranslation() const
	{
		return m_translation;
	}

	Matrix4frm getTransform() const
	{
		Matrix4frm transform;
		transform.setIdentity();
		transform.topLeftCorner(3, 3) = m_rotation;
		transform.block<1, 3>(3, 0) = m_translation;
		return transform;
	}

	Eigen::Affine3f getAffine() const
	{
		Eigen::Affine3f aff;
		aff.linear () = m_rotation;
		aff.translation () = m_translation;
		return aff;
	}

	void setIntrinsics(const NuiCameraIntrinsics& intri)
	{
		m_intrinsics = intri;
	}
	const NuiCameraIntrinsics& getIntrinsics() const
	{
		return m_intrinsics;
	}

	bool save(const std::string& fileName)
	{
		return NuiFileIOUtilities::writeCamera(fileName, m_intrinsics.m_fx, m_intrinsics.m_fy, m_intrinsics.m_cx, m_intrinsics.m_cy);
	}

	bool load(const std::string& fileName)
	{
		return NuiFileIOUtilities::readCamera(fileName, &m_intrinsics.m_fx, &m_intrinsics.m_fy, &m_intrinsics.m_cx, &m_intrinsics.m_cy);
	}

private:
	Matrix3frm				m_rotation;
	Vector3f				m_translation;
	NuiCameraIntrinsics		m_intrinsics;
};
