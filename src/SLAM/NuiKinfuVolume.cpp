#include "NuiKinfuVolume.h"

#include "Kernels/gpu_def.h"
#include "Foundation/NuiDebugMacro.h"

#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

static const std::string sVolume2Vertices("Volume2Vertices");

NuiKinfuVolume::NuiKinfuVolume()
	: m_vertexSumCL(NULL)
	, m_dirty(true)
{
	AcquireBuffer();
}

NuiKinfuVolume::~NuiKinfuVolume()
{
	ReleaseBuffer();
}

void NuiKinfuVolume::AcquireBuffer()
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();
	
	m_vertexSumCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}


void NuiKinfuVolume::ReleaseBuffer()
{
	if (m_vertexSumCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_vertexSumCL);
		NUI_CHECK_CL_ERR(err);
		m_vertexSumCL = NULL;
	}
}
