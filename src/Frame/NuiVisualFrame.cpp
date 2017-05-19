#include "NuiVisualFrame.h"

#include "NuiCompositeFrame.h"

NuiVisualFrame::NuiVisualFrame()
{
}

NuiVisualFrame::~NuiVisualFrame()
{
	clear();
}

void NuiVisualFrame::clear()
{
	m_depthFrame.Clear();
	m_colorFrame.Clear();
}

void	NuiVisualFrame::acquireFromCompositeFrame(NuiCompositeFrame* pCompositeFrame)
{
	if (!pCompositeFrame)
		return;

	const UINT nWidth = pCompositeFrame->m_depthFrame.GetWidth();
	const UINT nHeight = pCompositeFrame->m_depthFrame.GetHeight();
	const UINT nPointNum = nWidth * nHeight;
	UINT16* pDepthBuffer = pCompositeFrame->m_depthFrame.GetBuffer();
	if (!pDepthBuffer)
		return;
	
	UINT* pDepthDistortionLT = pCompositeFrame->m_depthDistortionFrame.GetBuffer();

	const UINT nColorMapWidth = pCompositeFrame->m_colorMapFrame.GetWidth();
	const UINT nColorMapHeight = pCompositeFrame->m_colorMapFrame.GetHeight();
	const UINT nColorMapNum = nColorMapWidth * nColorMapHeight;
	ColorSpacePoint* pDepthToColor = pCompositeFrame->m_colorMapFrame.GetBuffer();
	//assert(nPointNum == nColorMapNum);

	const NuiColorImage& colorTex = pCompositeFrame->m_colorFrame.GetImage();
	BGRQUAD* pImgSrc = colorTex.GetBuffer();

	if (pDepthToColor && pImgSrc)
	{
		BGRQUAD* pColors = m_colorFrame.AllocateBuffer(nWidth, nHeight);
		UINT16* pDepths = m_depthFrame.AllocateBuffer(nWidth, nHeight);
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (UINT i = 0; i < nPointNum; ++i)
		{
			const UINT mappedIndex = pDepthDistortionLT ? pDepthDistortionLT[i] : i;
			if (mappedIndex < nPointNum)
			{
				pDepths[i] = pDepthBuffer[mappedIndex];

				int colorX = (int)(pDepthToColor[mappedIndex].X + 0.5f);
				int colorY = (int)(pDepthToColor[mappedIndex].Y + 0.5f);
				if ((colorX >= 0 && colorX < (int)colorTex.GetWidth()) && (colorY >= 0 && colorY < (int)colorTex.GetHeight()))
				{
					int color_id = colorY * (int)colorTex.GetWidth() + colorX;
					pColors[i] = pImgSrc[color_id];
					pColors[i].rgbReserved = 255;
				}
				else
				{
					pColors[i].rgbRed = 0;
					pColors[i].rgbGreen = 0;
					pColors[i].rgbBlue = 0;
					pColors[i].rgbReserved = 0;
				}
			}
			else
			{
				pDepths[i] = -1;

				pColors[i].rgbRed = 0;
				pColors[i].rgbGreen = 0;
				pColors[i].rgbBlue = 0;
				pColors[i].rgbReserved = 0;
			}
		}
	}
	else
	{
		m_depthFrame = pCompositeFrame->m_depthFrame;

		// Acquire color buffer.
		const UINT nColorWidth = pCompositeFrame->m_colorFrame.GetWidth();
		const UINT nColorHeight = pCompositeFrame->m_colorFrame.GetHeight();
		const UINT nColorNum = nColorWidth * nColorHeight;
		BGRQUAD* pColorBuffer = pCompositeFrame->m_colorFrame.GetBuffer();
		if (pColorBuffer && nColorNum == nPointNum)
		{
			m_colorFrame = pCompositeFrame->m_colorFrame;
		}
	}

	m_cameraParams = pCompositeFrame->m_cameraParams;
	m_cameraParams.m_sensorDepthMax = (float)(pCompositeFrame->m_depthFrame.GetMaxDepth()) / 1000.0f;
	m_cameraParams.m_sensorDepthMin = (float)(pCompositeFrame->m_depthFrame.GetMinDepth()) / 1000.0f;
}

bool	NuiVisualFrame::saveFrame(const std::string& fileName, bool bCompressed)
{
	bool bSaved = false;

	std::stringstream ssTime;
	ssTime << getTimeStamp();
	std::string timeStampString;
	ssTime >> timeStampString;

	std::string imageFileName = fileName;
	imageFileName.append("\\");
	imageFileName.append(timeStampString);
	imageFileName.append(".depth");
	if (m_depthFrame.saveFrame(imageFileName, bCompressed))
		bSaved = true;

	imageFileName = fileName;
	imageFileName.append("\\");
	imageFileName.append(timeStampString);
	imageFileName.append(".color");
	if (m_colorFrame.saveFrame(imageFileName, bCompressed))
		bSaved = true;

	imageFileName = fileName;
	imageFileName.append("\\");
	imageFileName.append(timeStampString);
	imageFileName.append(".camIntri");
	if (m_cameraParams.save(imageFileName))
		bSaved = true;

	return bSaved;
}
