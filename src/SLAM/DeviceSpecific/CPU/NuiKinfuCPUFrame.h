#pragma once

#include "../NuiKinfuFrame.h"

class NuiKinfuCPUFrame : public NuiKinfuFrame
{
public:
	NuiKinfuCPUFrame();
	virtual ~NuiKinfuCPUFrame();

	virtual void	AcquireBuffers(UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight) override;
	virtual void	ReleaseBuffers() override;
	virtual void	UpdateDepthBuffers(UINT16* pDepths, UINT nNum, float minDepth, float maxDepth) override;
	virtual void	UpdateColorBuffers(ColorSpacePoint* pDepthToColor, UINT nNum, const NuiColorImage& image) override;

	float*			GetDepthsBuffer() const { return m_floatDepths.GetBuffer(); }
	BGRQUAD*		GetColorsBuffer() const { return m_colors.GetBuffer(); }
	NuiFloat3*		GetNormalsBuffer() const { return m_pNormals; }

	void			SetNormalsBuffer(NuiFloat3* pBuffer) { m_pNormals = pBuffer; }

protected:

private:
	NuiFloatImage	m_floatDepths;
	NuiColorImage	m_colors;
	NuiFloat3*		m_pNormals;
};