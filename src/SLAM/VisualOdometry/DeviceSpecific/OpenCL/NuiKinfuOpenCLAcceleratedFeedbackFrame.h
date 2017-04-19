#pragma once

#include "NuiKinfuOpenCLFeedbackFrame.h"

class NuiKinfuOpenCLAcceleratedFeedbackFrame : public NuiKinfuOpenCLFeedbackFrame
{
public:
	NuiKinfuOpenCLAcceleratedFeedbackFrame(UINT nWidth, UINT nHeight);
	virtual ~NuiKinfuOpenCLAcceleratedFeedbackFrame();

	virtual bool	BufferToMappableTexture(NuiCLMappableData* pMappableData, TrackerBufferType bufferType) override;

	void	resetExpectedRange();

	cl_mem	getExpectedRangeCL() const { return m_rangeImageCL; }

protected:
	void	AcquireBuffers(UINT nWidth, UINT nHeight);
	void	ReleaseBuffers();
private:
	cl_mem		m_rangeImageCL;
};