#include "NuiCompositeFrame.h"

NuiCompositeFrame::NuiCompositeFrame()
{
}

NuiCompositeFrame::~NuiCompositeFrame()
{
	Clear();
}

void NuiCompositeFrame::Clear()
{
	m_depthFrame.Clear();
	m_depthDistortionFrame.Clear();
	m_colorFrame.Clear();
	m_bodyIndexFrame.Clear();
	m_colorMapFrame.Clear();
	m_cameraMapFrame.Clear();
	m_skeletonFrame.Clear();
	m_faceTrackingFrame.Clear();
	m_facialModelFrame.Clear();
	m_pointCloudFrame.Clear();
	m_compressedDepthBuffer.Clear();
}

void	NuiCompositeFrame::SetCameraParams(const NuiCameraPos& cam)
{
	//boost::mutex::scoped_lock buff_lock (m_bCameraMutex);
	m_cameraParams = cam;
}

const NuiCameraPos&	NuiCompositeFrame::GetCameraParams() const
{
	//boost::mutex::scoped_lock buff_lock (m_bCameraMutex);
	return m_cameraParams;
}


