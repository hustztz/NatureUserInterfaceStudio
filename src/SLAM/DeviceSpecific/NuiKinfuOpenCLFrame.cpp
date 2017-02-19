#include "NuiKinfuOpenCLFrame.h"

#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"

#include "Shape\NuiCameraParams.h"
#include "Shape\NuiImageBuffer.h"
#include "Kernels/gpu_def.h"
#include "assert.h"

NuiKinfuOpenCLFrame::NuiKinfuOpenCLFrame()
	: m_rawDepthsCL(NULL)
	, m_floatDepthsCL(NULL)
	, m_colorUVsCL(NULL)
	, m_colorImageCL(NULL)
	, m_colorsCL(NULL)
	, m_cameraParamsCL(NULL)
	, m_nWidth(0)
	, m_nHeight(0)
	, m_nColorWidth(0)
	, m_nColorHeight(0)
{
	
}

void	NuiKinfuOpenCLFrame::AcquireBuffers(UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight)
{
	if(nWidth == m_nWidth && nHeight == m_nHeight && nColorWidth == m_nColorWidth && nColorHeight == m_nColorHeight)
	{
		return;
	}

	ReleaseBuffers();

	m_nWidth = nWidth;
	m_nHeight = nHeight;
	m_nColorWidth = nColorWidth;
	m_nColorHeight = nColorHeight;

	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();
	m_rawDepthsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY, m_nWidth*m_nHeight*sizeof(UINT16), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_floatDepthsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_nWidth*m_nHeight*sizeof(cl_float), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	if(m_nColorWidth * m_nColorHeight > 0)
	{
		m_colorUVsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY, m_nWidth*m_nHeight*sizeof(ColorSpacePoint), NULL, &err);
		NUI_CHECK_CL_ERR(err);
		/*cl_image_format colorImgFormat;
		colorImgFormat.image_channel_order = CL_BGRA;
		colorImgFormat.image_channel_data_type = CL_UNSIGNED_INT8;
		m_colorImageCL = NuiGPUMemManager::instance().CreateImage2DCL(context, CL_MEM_READ_ONLY, &colorImgFormat, m_nColorWidth, m_nColorHeight, 0, NULL, &err);
		NUI_CHECK_CL_ERR(err);*/
		m_colorImageCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY, m_nColorWidth*m_nColorHeight*sizeof(BGRQUAD), NULL, &err);
		NUI_CHECK_CL_ERR(err);
		m_colorsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_nWidth*m_nHeight*sizeof(BGRQUAD), NULL, &err);
		NUI_CHECK_CL_ERR(err);
	}
	m_cameraParamsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY, sizeof(NuiCLCameraParams), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}

void	NuiKinfuOpenCLFrame::ReleaseBuffers()
{
	if (m_rawDepthsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_rawDepthsCL);
		NUI_CHECK_CL_ERR(err);
		m_rawDepthsCL = NULL;
	}
	if (m_floatDepthsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_floatDepthsCL);
		NUI_CHECK_CL_ERR(err);
		m_floatDepthsCL = NULL;
	}
	if (m_colorUVsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_colorUVsCL);
		NUI_CHECK_CL_ERR(err);
		m_colorUVsCL = NULL;
	}
	if (m_colorImageCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_colorImageCL);
		NUI_CHECK_CL_ERR(err);
		m_colorImageCL = NULL;
	}
	if (m_colorsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_colorsCL);
		NUI_CHECK_CL_ERR(err);
		m_colorsCL = NULL;
	}
	if (m_cameraParamsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_cameraParamsCL);
		NUI_CHECK_CL_ERR(err);
		m_cameraParamsCL = NULL;
	}
}

void NuiKinfuOpenCLFrame::PassingDepths(float nearPlane, float farPlane)
{
	if(!m_floatDepthsCL|| !m_rawDepthsCL)
		return;

	// Get the kernel
	cl_kernel passingKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_PASSING_FILTER);
	assert(passingKernel);
	if (!passingKernel)
	{
		NUI_ERROR("Get kernel 'E_BILATERAL_FILTER' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(passingKernel, idx++, sizeof(cl_mem), &m_rawDepthsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(passingKernel, idx++, sizeof(cl_mem), &m_floatDepthsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(passingKernel, idx++, sizeof(float), &nearPlane);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(passingKernel, idx++, sizeof(float), &farPlane);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[1] = { m_nWidth * m_nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		passingKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
}

void	NuiKinfuOpenCLFrame::UpdateDepthBuffers(UINT16* pDepths, UINT nNum, float minDepth, float maxDepth)
{
	assert(m_nWidth*m_nHeight == nNum);
	if(!pDepths|| !m_rawDepthsCL)
		return;

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

#ifdef _GPU_PROFILER
	cl_event timing_event;
	cl_ulong time_start, time_end;
#endif
	clEnqueueWriteBuffer(
		queue,
		m_rawDepthsCL,
#ifdef _GPU_PROFILER
		CL_TRUE,
#else
		CL_FALSE,//blocking
#endif
		0,
		nNum * sizeof(UINT16),
		pDepths,
		0,
		NULL,
#ifdef _GPU_PROFILER
		&timing_event
#else
		NULL
#endif
		);
	NUI_CHECK_CL_ERR(err);

#ifdef _GPU_PROFILER
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_START, sizeof(time_start), &time_start, NULL);
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_END, sizeof(time_end), &time_end, NULL);
	std::cout << "write depth:" << (time_end - time_start) << std::endl;
	clReleaseEvent(timing_event);
#endif

	// filter the input depth map
	PassingDepths(minDepth, maxDepth);
}

void	NuiKinfuOpenCLFrame::UpdateColorBuffers(ColorSpacePoint* pDepthToColor, UINT nNum, const NuiColorImage& image)
{
	assert(m_nWidth*m_nHeight == nNum);
	if(!pDepthToColor || !image.GetBuffer() || !m_colorUVsCL || !m_colorImageCL)
		return;

	// Get the kernel
	cl_kernel uv2colorKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_UV2COLOR);
	assert(uv2colorKernel);
	if (!uv2colorKernel)
	{
		NUI_ERROR("Get kernel 'E_UV2COLOR' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

#ifdef _GPU_PROFILER
	cl_event timing_event;
	cl_ulong time_start, time_end;
#endif
	clEnqueueWriteBuffer(
		queue,
		m_colorUVsCL,
#ifdef _GPU_PROFILER
		CL_TRUE,
#else
		CL_FALSE,//blocking
#endif
		0,
		nNum * sizeof(ColorSpacePoint),
		pDepthToColor,
		0,
		NULL,
#ifdef _GPU_PROFILER
		&timing_event
#else
		NULL
#endif
		);
	NUI_CHECK_CL_ERR(err);

#ifdef _GPU_PROFILER
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_START, sizeof(time_start), &time_start, NULL);
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_END, sizeof(time_end), &time_end, NULL);
	std::cout << "write color:" << (time_end - time_start) << std::endl;
	clReleaseEvent(timing_event);
#endif

	//////////////////////////
	// Write Color Image
#ifdef _GPU_PROFILER
	cl_event timing_event2;
#endif
	clEnqueueWriteBuffer(
		queue,
		m_colorImageCL,
#ifdef _GPU_PROFILER
		CL_TRUE,
#else
		CL_FALSE,//blocking
#endif
		0,
		image.GetWidth() * image.GetHeight() * sizeof(BGRQUAD),
		image.GetBuffer(),
		0,
		NULL,
#ifdef _GPU_PROFILER
		&timing_event2
#else
		NULL
#endif
		);
	NUI_CHECK_CL_ERR(err);

#ifdef _GPU_PROFILER
	clGetEventProfilingInfo(timing_event2, CL_PROFILING_COMMAND_START, sizeof(time_start), &time_start, NULL);
	clGetEventProfilingInfo(timing_event2, CL_PROFILING_COMMAND_END, sizeof(time_end), &time_end, NULL);
	std::cout << "write color image:" << (time_end - time_start) << std::endl;
	clReleaseEvent(timing_event2);
#endif

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(uv2colorKernel, idx++, sizeof(cl_mem), &m_colorUVsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(uv2colorKernel, idx++, sizeof(cl_mem), &m_colorImageCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(uv2colorKernel, idx++, sizeof(cl_int), &m_nColorWidth);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(uv2colorKernel, idx++, sizeof(cl_int), &m_nColorHeight);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(uv2colorKernel, idx++, sizeof(cl_mem), &m_colorsCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { m_nWidth, m_nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		uv2colorKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
}

void	NuiKinfuOpenCLFrame::UpdateCameraParams(const NuiCameraParams& camParams)
{
	NuiCLCameraParams camParamsCL;
	camParamsCL.fx = camParams.m_intrinsics.m_fx;
	camParamsCL.fx_inv = 1 / camParamsCL.fx;
	camParamsCL.fy = camParams.m_intrinsics.m_fy;
	camParamsCL.fy_inv = 1 / camParamsCL.fy;
	camParamsCL.cx = camParams.m_intrinsics.m_cx;
	camParamsCL.cy = camParams.m_intrinsics.m_cy;
	camParamsCL.depthImageWidth = m_nWidth;
	camParamsCL.depthImageHeight = m_nHeight;
	camParamsCL.sensorDepthWorldMin = camParams.m_sensorDepthMin;
	camParamsCL.sensorDepthWorldMax = camParams.m_sensorDepthMax;

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	clEnqueueWriteBuffer(
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
}