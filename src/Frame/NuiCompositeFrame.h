#pragma once

#include "NuiImageFrame.h"
#include "NuiSkeletonFrame.h"
#include "NuiFaceTrackingFrame.h"
#include "NuiFacialModelFrame.h"
#include "NuiGestureFrame.h"
#include "NuiPointCloudFrame.h"
#include "NuiFusionImageFrame.h"
#include "NuiCompressedDepthFrame.h"

#include "Shape/NuiCameraPos.h"

// Forwards
class NuiCLMappableData;
class NuiMeshShape;

class NuiCompositeFrame
{
public:
	NuiCompositeFrame();
	~NuiCompositeFrame();

	void			Clear();

	void			SetCameraParams(const NuiCameraPos& cam);
	const NuiCameraPos&		GetCameraParams() const;

public:
	NuiDepthFrame			m_depthFrame;
	NuiColorFrame			m_colorFrame;
	NuiBodyIndexFrame		m_bodyIndexFrame;
	NuiColorMapFrame		m_colorMapFrame;
	NuiCameraMapFrame		m_cameraMapFrame;
	NuiDepthMapFrame		m_depthMapFrame;
	NuiSkeletonFrame		m_skeletonFrame;
	NuiFaceTrackingFrame	m_faceTrackingFrame;
	NuiFacialModelFrame		m_facialModelFrame;
	NuiGestureFrame			m_gestureFrame;
	NuiPointCloudFrame		m_pointCloudFrame;

	//boost::mutex			m_bCameraMutex;
	NuiCameraPos			m_cameraParams;

	// For KinectV1
	NuiCompressedDepthFrame	m_compressedDepthBuffer;
};