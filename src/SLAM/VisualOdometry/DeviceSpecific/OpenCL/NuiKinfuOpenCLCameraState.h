#pragma once

#include "../NuiKinfuCameraState.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

class NuiKinfuOpenCLCameraState : public NuiKinfuCameraState
{
public:
	NuiKinfuOpenCLCameraState();
	virtual ~NuiKinfuOpenCLCameraState();

	virtual void UpdateCameraTransform(const Matrix3frm& rot, const Vector3f& tran) override;
	virtual void UpdateCameraParams(const NuiCameraParams& cameraParams, UINT nWidth, UINT nHeight) override;
	virtual void UpdateCameraTransform(const Matrix3frm& rot, const Vector3f& tran, const Vector3f& offsetTran) override;
	
	cl_mem	GetCameraTransformBuffer() const { return m_rigidTransformCL; }
	cl_mem	GetCameraParamsBuffer() const { return m_cameraParamsCL; }
protected:
	void	WriteToBuffer(const Matrix3frm& rot, const Vector3f& tran);
private:
	cl_mem			m_cameraParamsCL;
	cl_mem			m_rigidTransformCL;
};