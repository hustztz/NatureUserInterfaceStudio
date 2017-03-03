#pragma once

#include "../NuiKinfuFrame.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

struct NuiTrackerConfig;

class NuiKinfuOpenCLFrame : public NuiKinfuFrame
{
public:
	NuiKinfuOpenCLFrame(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight);
	virtual ~NuiKinfuOpenCLFrame();

	virtual void	UpdateVertexBuffers(UINT16* pDepths, UINT nNum, NuiKinfuCameraState* pCameraState) override;
	virtual void	UpdateColorBuffers(ColorSpacePoint* pDepthToColor, UINT nNum, const NuiColorImage& image) override;
	virtual UINT	GetWidth() const override { return m_nWidth; }
	virtual UINT	GetHeight() const override { return m_nHeight; }

	cl_mem	GetDepthBuffer() const { return m_floatDepthsCL; }
	cl_mem	GetFilteredDepthBuffer() const { return m_filteredDepthsCL; }
	cl_mem	GetVertexBuffer() const { return m_verticesCL; }
	cl_mem	GetColorBuffer() const { return m_colorsCL; }
	float	GetDepthThreshold() const { return m_depth_threshold; }

protected:
	void	AcquireBuffers(UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight);
	void	ReleaseBuffers();
	void	PassingDepths(float nearPlane, float farPlane);
	void	GenerateGaussianBuffer(UINT filter_radius, float sigma_space2_inv_half);
	void	SmoothDepths(UINT filter_radius, float sigma_depth2_inv_half, float depth_threshold);
	void	Depth2vertex(cl_mem cameraParamsCL);

private:
	cl_mem m_rawDepthsCL;
	cl_mem m_floatDepthsCL;
	cl_mem m_gaussianCL;
	cl_mem m_filteredDepthsCL;
	cl_mem m_verticesCL;
	cl_mem m_colorUVsCL;
	cl_mem m_colorImageCL;
	cl_mem m_colorsCL;

	UINT	m_filter_radius;
	float	m_sigma_depth2_inv_half;
	float	m_depth_threshold;

	UINT m_nWidth, m_nHeight;
	UINT m_nColorWidth, m_nColorHeight;
};