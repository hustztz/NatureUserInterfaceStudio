#pragma once

#include "stdafx.h"

#include "Foundation\NuiThreadObject.h"

#include <opencv2/opencv.hpp>


// Forwards
class NuiFrameCacheImpl;
class NuiCompositeFrame;

class NuiCvVis : public NuiThreadObject
{
public:
	NuiCvVis(NuiFrameCacheImpl* pBuffer);
	virtual ~NuiCvVis();

protected:
	virtual bool process() override;

	void DrawDepthImage(NuiCompositeFrame* pCompositeFrame, UINT16 minDepth, UINT16 maxDepth, IplImage* pDepthImg);
	void DrawColorImage(NuiCompositeFrame* pCompositeFrame, IplImage* pColorImg);

private:

	NuiFrameCacheImpl*					m_pBuffer;

	IplImage*							m_pImg;
};