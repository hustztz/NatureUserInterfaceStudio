#include "NuiKinfuManager.h"
#include "Shape\NuiCameraParams.h"

NuiKinfuManager::NuiKinfuManager()
	: m_bAutoReset(false)
{
}

NuiKinfuManager::~NuiKinfuManager()
{
}

/*virtual*/
void	NuiKinfuManager::reset()
{
	m_buffer.clear();
	m_engine.resetTracker();
	m_engine.resetVolume();
}

/*virtual*/
bool	NuiKinfuManager::process ()
{
	std::shared_ptr<NuiCompositeFrame> pCompositeFrame = m_buffer.popCompositeFrame();
	if(!pCompositeFrame)
	{
		//boost::this_thread::sleep (boost::posix_time::seconds (1));
		return true;
	}

	const UINT nWidth = pCompositeFrame->m_depthFrame.GetWidth();
	const UINT nHeight = pCompositeFrame->m_depthFrame.GetHeight();
	const UINT nPointNum = nWidth * nHeight;
	UINT16* pDepthBuffer = pCompositeFrame->m_depthFrame.GetBuffer();
	UINT* pDepthDistortionLT = pCompositeFrame->m_depthDistortionFrame.GetBuffer();
	
	const UINT nColorMapWidth = pCompositeFrame->m_colorMapFrame.GetWidth();
	const UINT nColorMapHeight = pCompositeFrame->m_colorMapFrame.GetHeight();
	const UINT nColorMapNum = nColorMapWidth * nColorMapHeight;
	ColorSpacePoint* pDepthToColor = pCompositeFrame->m_colorMapFrame.GetBuffer();
	//assert(nPointNum == nColorMapNum);

	const NuiColorImage& colorTex = pCompositeFrame->m_colorFrame.GetImage();
	BGRQUAD* pImgSrc = colorTex.GetBuffer();

	NuiColorImage colorBuffer;
	BGRQUAD* pColors = colorBuffer.AllocateBuffer(nWidth, nHeight);
	NuiDepthImage depthBuffer;
	UINT16* pDepths = depthBuffer.AllocateBuffer(nWidth, nHeight);
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

	NuiCameraParams cameraParams;
	cameraParams.m_intrinsics = pCompositeFrame->GetCameraParams().getIntrinsics();
	cameraParams.m_sensorDepthMax = (float)(pCompositeFrame->m_depthFrame.GetMaxDepth()) / 1000.0f;
	cameraParams.m_sensorDepthMin = (float)(pCompositeFrame->m_depthFrame.GetMinDepth()) / 1000.0f;

	bool bSucceed = m_engine.processFrame(
		pCompositeFrame->m_depthFrame.GetTimeStamp(),
		pDepths,
		pColors,
		nWidth,
		nHeight,
		cameraParams);

	pCompositeFrame.reset();

	if( !bSucceed )
	{
		if(m_bAutoReset)
		{
			m_engine.resetTracker();
			std::cout << "Fusion reset." << std::endl;
		}
		else
		{
			std::cout << "Fusion quite." << std::endl;
			return false;
		}
	}
	return true;
}