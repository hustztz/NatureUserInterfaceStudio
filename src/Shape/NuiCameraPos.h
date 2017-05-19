#pragma once

#include <Eigen/Geometry>
#include "NuiCameraParams.h"

typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
typedef Eigen::Matrix<float, 4, 4> Matrix4frm;
typedef Eigen::Vector3f Vector3f;

class NuiCameraPos
{
public:
	NuiCameraPos()
	{
		m_rotation.setIdentity();
		m_localTranslation = Vector3f::Zero();
		m_offsetTranslation = Vector3f::Zero();
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

	void setLocalTranslation(const Vector3f& tran)
	{
		m_localTranslation = tran;
	}
	const Vector3f& getLocalTranslation() const
	{
		return m_localTranslation;
	}

	void setOffsetTranslation(const Vector3f& tran)
	{
		m_offsetTranslation = tran;
	}
	Vector3f		getGlobalTranslation() const { return m_localTranslation + m_offsetTranslation; }

	Matrix4frm getTransform() const
	{
		Matrix4frm transform;
		transform.setIdentity();
		transform.topLeftCorner(3, 3) = m_rotation;
		transform.block<1, 3>(3, 0) = getGlobalTranslation();
		return transform;
	}

	Eigen::Affine3f getAffine() const
	{
		Eigen::Affine3f aff;
		aff.linear () = m_rotation;
		aff.translation () = getGlobalTranslation();
		return aff;
	}

	void setIntrinsics(const NuiCameraIntrinsics& intri)
	{
		m_params.m_intrinsics = intri;
	}
	const NuiCameraIntrinsics& getIntrinsics() const
	{
		return m_params.m_intrinsics;
	}

	void setSensorDepthRange(float min, float max)
	{
		m_params.m_sensorDepthMin = min;
		m_params.m_sensorDepthMax = max;
	}
	float getSensorDepthMin() const
	{
		return m_params.m_sensorDepthMin;
	}
	float getSensorDepthMax() const
	{
		return m_params.m_sensorDepthMax;
	}

private:
	Matrix3frm				m_rotation;
	Vector3f				m_localTranslation;
	Vector3f				m_offsetTranslation;
	NuiCameraParams			m_params;
};
