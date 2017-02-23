#pragma once

#include "../../NuiKinfuCameraState.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

class NuiKinfuOpenCLCameraState : public NuiKinfuCameraDeviceCacheImpl
{
public:
	NuiKinfuOpenCLCameraState();
	virtual ~NuiKinfuOpenCLCameraState();

	virtual void UpdateCameraTransform(const Matrix3frm& rot, const Vector3f& tran) override;
	virtual void UpdateCameraParams(const NuiCameraParams& cameraParams, UINT nWidth, UINT nHeight) override;
	
	cl_mem	GetCameraTransformBuffer() const { return m_rigidTransformCL; }
	cl_mem	GetCameraParamsBuffer() const { return m_cameraParamsCL; }

private:
	cl_mem			m_cameraParamsCL;
	cl_mem			m_rigidTransformCL;
};