#include "NuiKinfuColorVolume.h"

#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

NuiKinfuColorVolume::NuiKinfuColorVolume(const Vector3i& resolution, unsigned char max_weight)
	: m_resolution(resolution)
	, m_colorVolumeCL(NULL)
	, m_max_weight(max_weight)
{
	int volume_x = m_resolution(0);
	int volume_y = m_resolution(1);
	int volume_z = m_resolution(2);

	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();
	m_colorVolumeCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, volume_y*volume_z*volume_x*sizeof(cl_char4), NULL, &err);
	NUI_CHECK_CL_ERR(err);

	reset();
}

NuiKinfuColorVolume::~NuiKinfuColorVolume()
{
	if (m_colorVolumeCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_colorVolumeCL);
		NUI_CHECK_CL_ERR(err);
		m_colorVolumeCL = NULL;
	}
}

const Vector3i& NuiKinfuColorVolume::getResolution() const
{
	return m_resolution;
}

cl_mem NuiKinfuColorVolume::data () const
{
	return m_colorVolumeCL;
}

void NuiKinfuColorVolume::reset()
{
	// Get the kernel
	cl_kernel initializeKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_INITIALIZE_COLOR_VOLUME);
	assert(initializeKernel);
	if (!initializeKernel)
	{
		NUI_ERROR("Get kernel 'E_INITIALIZE_COLOR_VOLUME' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_colorVolumeCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[3] = { m_resolution(0), m_resolution(1), m_resolution(2) };
	err = clEnqueueNDRangeKernel(
		queue,
		initializeKernel,
		3,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
}
