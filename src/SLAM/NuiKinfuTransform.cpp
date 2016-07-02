#include "NuiKinfuTransform.h"

#include "Kernels/gpu_def.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

NuiKinfuTransform::NuiKinfuTransform()
	: m_rigidTransformCL(NULL)
{
	m_rotation.setIdentity();
	m_translation = Vector3f::Zero();

	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();
	m_rigidTransformCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(NuiCLRigidTransform), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}

NuiKinfuTransform::~NuiKinfuTransform()
{

	if (m_rigidTransformCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_rigidTransformCL);
		NUI_CHECK_CL_ERR(err);
		m_rigidTransformCL = NULL;
	}
}

void NuiKinfuTransform::setTransform(const Matrix3frm& rot, const Vector3f& tran)
{
	m_rotation = rot;
	m_translation = tran;

	Matrix3frm rot_inv = m_rotation.inverse();
	NuiCLRigidTransform currTransform;
	memcpy(currTransform.R, m_rotation.data(), 9 * sizeof(float));
	memcpy(currTransform.t, m_translation.data(), 3 * sizeof(float));
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