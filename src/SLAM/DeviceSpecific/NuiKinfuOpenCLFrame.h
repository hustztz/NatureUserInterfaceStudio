#pragma once

#include "NuiKinfuFrameImpl.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

class NuiKinfuOpenCLFrame : public NuiKinfuFrameImpl
{
public:
	NuiKinfuOpenCLFrame();
	virtual ~NuiKinfuOpenCLFrame();

	virtual void	AcquireBuffers(UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight) override;
	virtual void	ReleaseBuffers() override;
	virtual void	UpdateDepthBuffers(UINT16* pDepths, UINT nNum, float minDepth, float maxDepth) override;
	virtual void	UpdateColorBuffers(ColorSpacePoint* pDepthToColor, UINT nNum, const NuiColorImage& image) override;
	virtual void	UpdateCameraParams(const NuiCameraParams& camParams) override;

	cl_mem GetDepthsBuffer() const { return m_floatDepthsCL; }
	cl_mem GetColorsBuffer() const { return m_colorsCL; }
	cl_mem GetCameraParamsBuffer() const { return m_cameraParamsCL; }

protected:
	void	PassingDepths(float nearPlane, float farPlane);

private:
	cl_mem m_rawDepthsCL;
	cl_mem m_floatDepthsCL;
	cl_mem m_colorUVsCL;
	cl_mem m_colorImageCL;
	cl_mem m_colorsCL;
	cl_mem m_cameraParamsCL;

	UINT m_nWidth, m_nHeight;
	UINT m_nColorWidth, m_nColorHeight;
};