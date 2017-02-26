#include "NuiKinfuCPUFrame.h"

#include "Foundation/NuiDebugMacro.h"
#include "Shape\NuiCameraParams.h"
#include "assert.h"

NuiKinfuCPUFrame::NuiKinfuCPUFrame()
	: m_pNormals(NULL)
{
	
}

NuiKinfuCPUFrame::~NuiKinfuCPUFrame()
{
	ReleaseBuffers();
}

void	NuiKinfuCPUFrame::AcquireBuffers(UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight)
{
	if(nWidth == m_nWidth && nHeight == m_nHeight &&
		nColorWidth == m_colors.GetWidth() && nColorHeight == m_colors.GetHeight())
	{
		return;
	}

	ReleaseBuffers();

	m_nWidth = nWidth;
	m_nHeight = nHeight;

	m_floatDepths.AllocateBuffer(m_nWidth, m_nHeight);
	m_colors.AllocateBuffer(m_nWidth, m_nHeight);
}

void	NuiKinfuCPUFrame::ReleaseBuffers()
{
	m_nWidth = 0;
	m_nHeight = 0;
	m_floatDepths.Clear();
	m_colors.Clear();
	m_pNormals = NULL;
}

void	NuiKinfuCPUFrame::UpdateDepthBuffers(UINT16* pDepths, UINT nNum, float minDepth, float maxDepth)
{
	assert(m_nWidth*m_nHeight == nNum);
	if(!pDepths)
		return;

	float* pBuffers = m_floatDepths.GetBuffer();
	for (UINT i = 0; i < nNum; ++i)
	{
		pBuffers[i] = pDepths[i] * 0.001f;
		if((pBuffers[i] < minDepth) || (pBuffers[i] > maxDepth))
			pBuffers[i] = -1.0f;
	}
}

void	NuiKinfuCPUFrame::UpdateColorBuffers(ColorSpacePoint* pDepthToColor, UINT nNum, const NuiColorImage& image)
{
	assert(m_nWidth*m_nHeight == nNum);
	if(!pDepthToColor || !image.GetBuffer())
		return;

	BGRQUAD* pBuffers = m_colors.GetBuffer();
	BGRQUAD* pSrc = image.GetBuffer();
	for (UINT i = 0; i < nNum; ++i)
	{
		int colorX = (int)(pDepthToColor[i].X + 0.5f);
		int colorY = (int)(pDepthToColor[i].Y + 0.5f);
		if ((colorX >= 0 && colorX < image.GetWidth()) && (colorY >= 0 && colorY < image.GetHeight()))
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
