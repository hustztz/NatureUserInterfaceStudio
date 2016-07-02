#pragma once

#include <Eigen/Geometry>
#include "OpenCLUtilities/NuiOpenCLUtil.h"

typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
typedef Eigen::Vector3f Vector3f;

class NuiKinfuTransform
{
public:
	NuiKinfuTransform();
	~NuiKinfuTransform();

	void setTransform(const Matrix3frm& rot, const Vector3f& tran);
	
	const Matrix3frm& getRotation() const {	return m_rotation;	}
	const Vector3f& getTranslation() const { return m_translation;	}
	cl_mem	getTransformCL() const { return m_rigidTransformCL; }

private:
	Matrix3frm				m_rotation;
	Vector3f				m_translation;
	cl_mem					m_rigidTransformCL;
};