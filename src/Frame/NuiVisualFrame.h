#pragma once

#include "NuiImageFrame.h"
#include "Shape/NuiCameraPos.h"

class NuiCompositeFrame;

class NuiVisualFrame
{
public:
	NuiVisualFrame();
	~NuiVisualFrame();

	void			clear();

	void			acquireFromCompositeFrame(NuiCompositeFrame* pCompositeFrame);
	INT64			getTimeStamp() const { return m_depthFrame.GetTimeStamp(); }
	UINT			getWidth() const { return m_depthFrame.GetWidth(); }
	UINT			getHeight() const { return m_depthFrame.GetHeight(); }
	UINT16*			getDepthBuffer() const { return m_depthFrame.GetBuffer(); }
	BGRQUAD*		getColorBuffer() const { return m_colorFrame.GetBuffer(); }
	NuiCameraParams	getCameraParams() const { return m_cameraParams; }
	bool			saveFrame(const std::string& fileName, bool bCompressed);

private:
	NuiDepthFrame			m_depthFrame;
	NuiColorFrame			m_colorFrame;

	//boost::mutex			m_bCameraMutex;
	NuiCameraParams			m_cameraParams;
};