#pragma once

#include <boost/thread/thread.hpp>

#include "stdafx.h"
#include "Frame\NuiKinectV1FaceTrackingFrame.h"

#include <FaceTrackLib.h>
#include <NuiApi.h>

// Forwards
class NuiRGBDDeviceBufferImpl;
class NuiCompositeFrame;

// PCEKinectThread thread class
class NuiKinectV1FaceTracker
{
public:
	NuiKinectV1FaceTracker();
	~NuiKinectV1FaceTracker();

	bool								setupTracker(NUI_IMAGE_RESOLUTION depthResolution, NUI_IMAGE_RESOLUTION	colorResolution);
	void								updateCache(NuiRGBDDeviceBufferImpl* pCache);
	
	void								startThread ();
	void								stopThread();
	void								pauseThread();
	bool								isThreadOn() const { return m_threadOn; }

private:
	void								runThread ();
	bool								readFrame(NuiCompositeFrame* pCompositeFrame);
	void								getFaceShape(float zoomFactor, POINT* viewOffSet);
	bool								getClosestFaceHint(NuiCompositeFrame* pCompositeFrame);

private:
	boost::shared_ptr<boost::thread>	m_Thread;
	bool								m_threadOn;
	bool								m_threadPause;

	IFTFaceTracker*						m_pFaceTracker;
	IFTResult*							m_pFTResult;
	IFTImage*							m_colorImage;
	IFTImage*							m_depthImage;
	FT_CAMERA_CONFIG					m_videoConfig;

	bool								m_LastTrackSucceeded;
	FT_VECTOR3D							m_hint3D[2];
	
	NuiRGBDDeviceBufferImpl*					m_pCache;
};

