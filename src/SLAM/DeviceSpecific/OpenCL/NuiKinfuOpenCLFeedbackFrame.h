#pragma once

#include "../NuiKinfuFeedbackFrame.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

class NuiKinfuOpenCLFeedbackFrame : public NuiKinfuFeedbackFrame
{
public:
	NuiKinfuOpenCLFeedbackFrame(UINT nWidth, UINT nHeight);
	virtual ~NuiKinfuOpenCLFeedbackFrame();

	virtual void	UpdateBuffers(NuiKinfuFrame* pFrame, NuiKinfuCameraState* pCameraState) override;
	virtual UINT	GetWidth() const override { return m_nWidth; }
	virtual UINT	GetHeight() const override { return m_nHeight; }

	virtual bool	VerticesToMappablePosition(NuiCLMappableData* pMappableData) override;
	virtual bool	BufferToMappableTexture(NuiCLMappableData* pMappableData, TrackerBufferType bufferType) override;

	cl_mem GetVertexBuffer() const { return m_verticesCL; }
	cl_mem GetNormalBuffer() const { return m_normalsCL; }
	cl_mem GetColorBuffer() const { return m_colorsCL; }

protected:
	void	AcquireBuffers(UINT nWidth, UINT nHeight);
	void	ReleaseBuffers();
	void	Vertex2Normal(cl_mem verticesCL, float depth_threshold);
	void	TransformBuffers(cl_mem verticesCL, cl_mem normalsCL, cl_mem transformCL);
	void	CopyColors(cl_mem colorsCL);

protected:
	UINT m_nWidth, m_nHeight;

private:
	cl_mem m_verticesCL;
	cl_mem m_normalsCL;
	cl_mem m_colorsCL;
};