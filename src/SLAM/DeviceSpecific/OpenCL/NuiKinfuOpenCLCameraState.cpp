#include "NuiKinfuOpenCLCameraState.h"

#include "Kernels/gpu_def.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

NuiKinfuOpenCLCameraState::NuiKinfuOpenCLCameraState()
	: m_rigidTransformCL(NULL)
	, m_cameraParamsCL(NULL)
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();
	m_rigidTransformCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(NuiCLRigidTransform), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_cameraParamsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY, sizeof(NuiCLCameraParams), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}

NuiKinfuOpenCLCameraState::~NuiKinfuOpenCLCameraState()
{
	if (m_rigidTransformCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_rigidTransformCL);
		NUI_CHECK_CL_ERR(err);
		m_rigidTransformCL = NULL;
	}
	if (m_cameraParamsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_cameraParamsCL);
		NUI_CHECK_CL_ERR(err);
		m_cameraParamsCL = NULL;
	}
}

void	NuiKinfuOpenCLCameraState::UpdateCameraParams(const NuiCameraParams& camParams, UINT nWidth, UINT nHeight)
{
	NuiCLCameraParams camParamsCL;
	camParamsCL.fx = camParams.m_intrinsics.m_fx;
	camParamsCL.fx_inv = 1 / camParamsCL.fx;
	camParamsCL.fy = camParams.m_intrinsics.m_fy;
	camParamsCL.fy_inv = 1 / camParamsCL.fy;
	camParamsCL.cx = camParams.m_intrinsics.m_cx;
	camParamsCL.cy = camParams.m_intrinsics.m_cy;
	camParamsCL.depthImageWidth = nWidth;
	camParamsCL.depthImageHeight = nHeight;
	camParamsCL.sensorDepthWorldMin = camParams.m_sensorDepthMin;
	camParamsCL.sensorDepthWorldMax = camParams.m_sensorDepthMax;

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	err = clEnqueueWriteBuffer(
		queue,
		m_cameraParamsCL,
		CL_FALSE,//blocking
		0,
		sizeof(NuiCLCameraParams),
		&camParamsCL,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	NuiKinfuCameraState::UpdateCameraParams(camParams, nWidth, nHeight);
}

void NuiKinfuOpenCLCameraState::UpdateCameraTransform(const Matrix3frm& rot, const Vector3f& tran)
{
	Matrix3frm rot_inv = rot.inverse();
	NuiCLRigidTransform currTransform;
	memcpy(currTransform.R, rot.data(), 9 * sizeof(float));
	memcpy(currTransform.t, tran.data(), 3 * sizeof(float));
	memcpy(currTransform.R_inv, rot_inv.data(), 9 * sizeof(float));

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	err = clEnqueueWriteBuffer(
		queue,
		m_rigidTransformCL,
		CL_FALSE,//blocking
		0,
		sizeof(NuiCLRigidTransform),
		&currTransform,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	NuiKinfuCameraState::UpdateCameraTransform(rot, tran);
}

//void NuiKinfuTransform::setTransform(cl_mem transformCL)
//{
//	NuiCLRigidTransform currTransform;
//	err = clEnqueueReadBuffer(
//		queue,
//		transformCL,
//		CL_TRUE,//blocking
//		0,
//		sizeof(NuiCLRigidTransform),
//		&currTransform,
//		0,
//		NULL,
//		NULL
//		);
//	NUI_CHECK_CL_ERR(err);
//	memcpy(Rcurr.data(), currTransform.R, 9 * sizeof(float));
//	memcpy(tcurr.data(), currTransform.t, 3 * sizeof(float));
//}