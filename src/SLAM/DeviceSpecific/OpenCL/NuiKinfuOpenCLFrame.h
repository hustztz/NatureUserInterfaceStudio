#pragma once

#include "../NuiKinfuFrame.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

class NuiKinfuOpenCLFrame : public NuiKinfuFrame
{
public:
	NuiKinfuOpenCLFrame();
	virtual ~NuiKinfuOpenCLFrame();

	virtual void	AcquireBuffers(UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight) override;
	virtual void	ReleaseBuffers() override;
	virtual void	UpdateDepthBuffers(UINT16* pDepths, UINT nNum, float minDepth, float maxDepth) override;
	virtual void	UpdateColorBuffers(ColorSpacePoint* pDepthToColor, UINT nNum, const NuiColorImage& image) override;

	cl_mem GetDepthsBuffer() const { return m_floatDepthsCL; }
	cl_mem GetColorsBuffer() const { return m_colorsCL; }
	cl_mem GetNormalsBuffer() const { return m_pNormalCL ? *m_pNormalCL : NULL; }

	void SetNormalsBuffer(cl_mem* pBuffer) { m_pNormalCL = pBuffer; }

protected:
	void	PassingDepths(float nearPlane, float farPlane);

private:
	cl_mem m_rawDepthsCL;
	cl_mem m_floatDepthsCL;
	cl_mem m_colorUVsCL;
	cl_mem m_colorImageCL;
	cl_mem m_colorsCL;
	cl_mem* m_pNormalCL;

	UINT m_nColorWidth, m_nColorHeight;
};