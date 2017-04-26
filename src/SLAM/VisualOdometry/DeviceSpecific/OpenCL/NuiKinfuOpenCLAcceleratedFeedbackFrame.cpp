#include "NuiKinfuOpenCLAcceleratedFeedbackFrame.h"

#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiOpenCLFoundationUtils.h"
#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"

#include "Foundation/NuiDebugMacro.h"
#include "Shape/NuiCLMappableData.h"
#include "Kernels/hashing_gpu_def.h"

#include "assert.h"

NuiKinfuOpenCLAcceleratedFeedbackFrame::NuiKinfuOpenCLAcceleratedFeedbackFrame(UINT nWidth, UINT nHeight)
	: NuiKinfuOpenCLFeedbackFrame(nWidth, nHeight)
	, m_rangeImageCL(NULL)
{
	AcquireBuffers(nWidth, nHeight);
}

NuiKinfuOpenCLAcceleratedFeedbackFrame::~NuiKinfuOpenCLAcceleratedFeedbackFrame()
{
	ReleaseBuffers();
}

void	NuiKinfuOpenCLAcceleratedFeedbackFrame::AcquireBuffers(UINT nWidth, UINT nHeight)
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	m_rangeImageCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nWidth * nHeight * sizeof(cl_float2), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}

void	NuiKinfuOpenCLAcceleratedFeedbackFrame::ReleaseBuffers()
{
	if (m_rangeImageCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_rangeImageCL);
		NUI_CHECK_CL_ERR(err);
		m_rangeImageCL = NULL;
	}
}

void	NuiKinfuOpenCLAcceleratedFeedbackFrame::resetExpectedRange()
{
	const UINT nNum = m_nWidth * m_nHeight;
	NuiOpenCLFoundationUtils::setFloat2Buffer(FAR_AWAY, VERY_CLOSE, m_rangeImageCL, nNum);
}

bool	NuiKinfuOpenCLAcceleratedFeedbackFrame::BufferToMappableTexture(NuiCLMappableData* pMappableData, TrackerBufferType bufferType)
{
	if(eTracker_Ranges != bufferType)
		return NuiKinfuOpenCLFeedbackFrame::BufferToMappableTexture(pMappableData, bufferType);

	assert(pMappableData);
	if(!pMappableData)
		return false;

	if(!m_rangeImageCL)
		return false;

	// Get the kernel
	cl_kernel rgbaKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_FLOAT2_TO_TEXTURE);
	assert(rgbaKernel);
	if (!rgbaKernel)
	{
		NUI_ERROR("Get kernel 'E_FLOAT2_TO_RGBA' failed!\n");
		return false;
	}

	if( m_nWidth != pMappableData->FeedbackTex().width() || m_nHeight != pMappableData->FeedbackTex().height())
	{
		NuiTextureMappableAccessor::updateImpl(
			pMappableData->FeedbackTex(),
			m_nWidth,
			m_nHeight,
			NULL
			);
	}
	cl_mem texGL = NuiOpenCLBufferFactory::asTexture2DCL(pMappableData->FeedbackTex());

	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	// 
	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	// Acquire OpenGL objects before use
	cl_mem glObjs[] = {
		texGL
	};

	openclutil::enqueueAcquireHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(rgbaKernel, idx++, sizeof(cl_mem), &m_rangeImageCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(rgbaKernel, idx++, sizeof(cl_mem), &texGL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { m_nWidth, m_nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		rgbaKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	// Release OpenGL objects
	openclutil::enqueueReleaseHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

	return true;
}
