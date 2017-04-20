#pragma once

#include "../NuiKinfuFrame.h"

typedef Eigen::Vector3f Vector3f;
struct NuiTrackerConfig;
struct NuiCameraIntrinsics;

class NuiKinfuCPUFrame : public NuiKinfuFrame
{
public:
	NuiKinfuCPUFrame(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight);
	virtual ~NuiKinfuCPUFrame();

	virtual void	UpdateVertexBuffers(UINT16* pDepths, UINT* pDepthDistortionLT, UINT nNum, NuiKinfuCameraState* pCameraState) override;
	virtual void	UpdateColorBuffers(ColorSpacePoint* pDepthToColor, UINT* pDepthDistortionLT, UINT nNum, const NuiColorImage& image) override;
	virtual UINT	GetWidth() const override { return m_floatDepths.GetWidth(); }
	virtual UINT	GetHeight() const override { return m_floatDepths.GetHeight(); }

	float*			GetDepthsBuffer() const { return m_floatDepths.GetBuffer(); }
	float*			GetFilteredDepthBuffer() const { return m_filteredDepths.GetBuffer(); }
	Vector3f*		GetVertexBuffer() const { return m_vertices.GetBuffer(); }
	BGRQUAD*		GetColorsBuffer() const { return m_colors.GetBuffer(); }
	float			GetDepthThreshold() const { return m_depth_threshold; }

protected:
	void	AcquireBuffers(UINT nWidth, UINT nHeight);
	void	ReleaseBuffers();

	void	SmoothDepths(UINT filter_radius, float sigma_depth2_inv_half, float depth_threshold);
	void	Depth2vertex(NuiCameraIntrinsics cameraIntrics);

private:
	NuiFloatImage	m_floatDepths;
	NuiFloatImage	m_filteredDepths;
	NuiFloat3Image	m_vertices;
	NuiColorImage	m_colors;

	UINT	m_filter_radius;
	float	m_sigma_depth2_inv_half;
	float	m_depth_threshold;
};