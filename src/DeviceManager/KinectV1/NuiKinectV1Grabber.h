#pragma once

#include <windows.h>
#include <NuiApi.h>
#include <boost/thread/thread.hpp>

#include "Frame\NuiExtendedDepthFrame.h"
#include "Frame\NuiImageFrame.h"

// Forwards
class NuiRGBDDeviceBufferImpl;
class NuiCompositeFrame;

// PCEKinectThread thread class
class NuiKinectV1Grabber
{
public:
	NuiKinectV1Grabber(INuiSensor* m_pNuiSensor);
	~NuiKinectV1Grabber();

	bool								setupDevice(DWORD dwFlags, NUI_IMAGE_RESOLUTION depthResolution, NUI_IMAGE_RESOLUTION	colorResolution);
	void								updateCache(NuiRGBDDeviceBufferImpl* pCache) { m_pCache = pCache; }
	
	void								startThread ();
	void								stopThread();
	void								pauseThread();
	bool								isThreadOn() const { return m_threadOn; }

	void								setNearMode(bool bNearMode);
	bool								getNearMode() const;
	void								setSkeletonSeatedMode(bool bSeatedMode);
	void								updateSkeletonSmoothParams(float smoothing, float correction, float prediction, float jitterRadius, float maxDeviationRadius);
	void								updateDepthRange(UINT minDepth, UINT maxDepth);
	void								updateNearDepthRange(UINT minDepth, UINT maxDepth);

private:
	void								updateNearMode();
	void								updateSkeletonMode();
	void								grabThread ();
	bool								grabDepthFrame (NuiCompositeFrame* pCompositeFrame);
	bool								grabColorFrame ();
	bool								grabSkeletonFrame (NuiCompositeFrame* pCompositeFrame);
	bool								frameToCache (NuiCompositeFrame* pCompositeFrame);

private:
	NuiKinectV1Grabber (const NuiKinectV1Grabber&); // Disabled copy constructor
	NuiKinectV1Grabber& operator = (const NuiKinectV1Grabber&); // Disabled assignment operator

private:
	boost::shared_ptr<boost::thread>	m_Thread;
	bool								m_threadOn;

	NuiRGBDDeviceBufferImpl*				m_pCache;

	INuiSensor*							m_pNuiSensor;
	INuiCoordinateMapper*				m_pMapper;

	NUI_IMAGE_RESOLUTION				m_depthResolution;
	NuiExtendedDepthFrame				m_depthFrameBuffer;
	UINT								m_depthFrameCount;
	LONGLONG							m_lastDepthTime;

	NUI_IMAGE_RESOLUTION				m_colorResolution;
	NuiColorFrame						m_colorFrameBuffer;
	UINT								m_colorFrameCount;
	LONGLONG							m_lastColorTime;

	NUI_COLOR_IMAGE_POINT*				m_pColorToDepth;
	
	HANDLE								m_hNextDepthFrameEvent;
	HANDLE								m_pDepthStreamHandle;
	HANDLE								m_hNextColorFrameEvent;
	HANDLE								m_pColorStreamHandle;
	HANDLE								m_hNextSkeletonFrameEvent;

	BOOL								m_bNearMode;
	BOOL								m_bSkeletonSeatedMode;
	UINT								m_minDepth;
	UINT								m_maxDepth;
	UINT								m_minNearDepth;
	UINT								m_maxNearDepth;
	NUI_TRANSFORM_SMOOTH_PARAMETERS		m_skeletonSmoothParams;
};

