#include "NuiKinfuCPUFrame.h"

#include "NuiKinfuCPUUtilities.h"
#include "../NuiKinfuCameraState.h"
#include "../../NuiTrackerConfig.h"
#include "Foundation/NuiDebugMacro.h"
#include "assert.h"

NuiKinfuCPUFrame::NuiKinfuCPUFrame(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight)
	: m_filter_radius(config.filter_radius)
	, m_sigma_depth2_inv_half(config.sigma_depth2_inv_half)
	, m_depth_threshold(config.depth_threshold)
{
	AcquireBuffers(nWidth, nHeight);
}

NuiKinfuCPUFrame::~NuiKinfuCPUFrame()
{
	ReleaseBuffers();
}

void	NuiKinfuCPUFrame::AcquireBuffers(UINT nWidth, UINT nHeight)
{
	if(nWidth == m_floatDepths.GetWidth() && nHeight == m_floatDepths.GetHeight())
	{
		return;
	}

	ReleaseBuffers();

	m_floatDepths.AllocateBuffer(nWidth, nHeight);
	m_filteredDepths.AllocateBuffer(nWidth, nHeight);
	m_vertices.AllocateBuffer(nWidth, nHeight);
	m_colors.AllocateBuffer(nWidth, nHeight);
}

void	NuiKinfuCPUFrame::ReleaseBuffers()
{
	m_floatDepths.Clear();
	m_filteredDepths.Clear();
	m_vertices.Clear();
	m_colors.Clear();
}


#define MEAN_SIGMA_L 1.2232f

void NuiKinfuCPUFrame::SmoothDepths(UINT filter_radius, float sigma_depth2_inv_half, float depth_threshold)
{
	float* floatDepths = m_floatDepths.GetBuffer();
	float* dstDepths = m_filteredDepths.GetBuffer();
	assert(floatDepths);
	if(!floatDepths || !dstDepths)
		return;

	int nWidth = (int)m_floatDepths.GetWidth();
	int nHeight = (int)m_floatDepths.GetHeight();
#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int y = 0; y < nHeight; y++)
	{
		for (int x = 0; x < nWidth; x++)
		{
			const int centerId = y * nWidth + x;
			float center = floatDepths[centerId];

			dstDepths[centerId] = NAN_FLOAT;
			if(NAN_FLOAT == center)
			{
				continue;
			}

			float sigma_z = 1.0f / (0.0012f + 0.0019f*(center - 0.4f)*(center - 0.4f) + 0.0001f / sqrt(center) * 0.25f);
			float sumDepth = 0;
			float sumWeight = 0;

			for(int cy = -(int)filter_radius; cy <= (int)filter_radius; ++ cy)
			{
				for(int cx = -(int)filter_radius; cx <= (int)filter_radius; ++ cx)
				{
					const int nearX = x + cx;
					const int nearY = y + cy;
					if( nearX>=0 && nearX<nWidth && nearY>=0 && nearY<nHeight )
					{
						const int nearId = nearY * nWidth + nearX;
						float near = floatDepths[nearId];
						if(NAN_FLOAT == near)
							continue;
						float diff = fabs(center - near);
						if(near > 0.0f && diff < depth_threshold)
						{
							float depth2 = diff * diff;
							// Different from InfiniTAM
							float weight = expf(-0.5f * ((abs(cx) + abs(cy))*MEAN_SIGMA_L*MEAN_SIGMA_L + depth2 * sigma_z * sigma_z));

							sumDepth += near * weight;
							sumWeight += weight;
						}
					}
				}
			}
			dstDepths[centerId] = sumDepth/sumWeight;
		}
	}
}

void NuiKinfuCPUFrame::Depth2vertex(NuiCameraIntrinsics cameraIntrics)
{
	float* depthsBuffer = m_filteredDepths.GetBuffer();
	Vector3f* verticesBuffer = m_vertices.GetBuffer();
	if(!depthsBuffer || !verticesBuffer)
		return;

	UINT nWidth = m_filteredDepths.GetWidth();
	UINT nHeight = m_filteredDepths.GetHeight();
#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (UINT y = 0; y < nHeight; y++)
	{
		for (UINT x = 0; x < nWidth; x++)
		{
			const UINT id = y * nWidth + x;
			float dp = depthsBuffer[id];

			if(dp != NAN_FLOAT)
			{
				const float intr_fx_inv = 1 / cameraIntrics.m_fx;
				const float intr_fy_inv = 1 / cameraIntrics.m_fy;
				const float intr_cx = cameraIntrics.m_cx;
				const float intr_cy = cameraIntrics.m_cy;
				verticesBuffer[id][0] = dp * ((float)x - intr_cx) * intr_fx_inv;
				verticesBuffer[id][1] = dp * ((float)y - intr_cy) * intr_fy_inv;
				verticesBuffer[id][2] = dp;
			}
			else
			{
				verticesBuffer[id] = Vector3f(NAN_FLOAT, NAN_FLOAT, NAN_FLOAT);
			}
		}
	}
}

void	NuiKinfuCPUFrame::UpdateVertexBuffers(UINT16* pDepths, UINT nNum, NuiKinfuCameraState* pCameraState)
{
	assert(m_floatDepths.GetWidth()*m_floatDepths.GetHeight() == nNum);
	if(!pDepths || !pCameraState)
		return;

	float minDepth = pCameraState->GetCameraPos().getSensorDepthMin();
	float maxDepth = pCameraState->GetCameraPos().getSensorDepthMax();

	float* pBuffers = m_floatDepths.GetBuffer();
#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (UINT i = 0; i < nNum; ++i)
	{
		pBuffers[i] = pDepths[i] * 0.001f;
		if((pBuffers[i] < minDepth) || (pBuffers[i] > maxDepth))
			pBuffers[i] = NAN_FLOAT;
	}

	SmoothDepths(m_filter_radius, m_sigma_depth2_inv_half, m_depth_threshold);
	Depth2vertex(pCameraState->GetCameraPos().getIntrinsics());
}

void	NuiKinfuCPUFrame::UpdateColorBuffers(ColorSpacePoint* pDepthToColor, UINT nNum, const NuiColorImage& image)
{
	assert(m_colors.GetWidth()*m_colors.GetHeight() == nNum);
	if(!pDepthToColor || !image.GetBuffer())
		return;

	BGRQUAD* pBuffers = m_colors.GetBuffer();
	BGRQUAD* pSrc = image.GetBuffer();
#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (UINT i = 0; i < nNum; ++i)
	{
		int colorX = (int)(pDepthToColor[i].X + 0.5f);
		int colorY = (int)(pDepthToColor[i].Y + 0.5f);
		if ((colorX >= 0 && colorX < (int)image.GetWidth()) && (colorY >= 0 && colorY < (int)image.GetHeight()))
		{
			pBuffers[i] = pSrc[i];
		}
		else
		{
			pBuffers[i].rgbRed = 0;
			pBuffers[i].rgbGreen = 0;
			pBuffers[i].rgbBlue = 0;
			pBuffers[i].rgbReserved = 0;
		}
	}
}
