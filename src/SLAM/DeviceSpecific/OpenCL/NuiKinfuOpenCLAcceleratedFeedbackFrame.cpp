#include "NuiKinfuOpenCLAcceleratedFeedbackFrame.h"

#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "OpenCLUtilities/NuiOpenCLFoundationUtils.h"

#include "assert.h"

#define FAR_AWAY 999999.9f
#define VERY_CLOSE 0.05f

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
