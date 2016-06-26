#include "NuiKinectV2Grabber.h"

NuiKinectV2Grabber::NuiKinectV2Grabber(IKinectSensor* pNuiSensor)
	: m_pNuiSensor(pNuiSensor)
	, m_pMapper(NULL)
	, m_pCache(NULL)
	, m_pDepthFrame(NULL)
	, m_pColorFrame(NULL)
	, m_pSkeletonFrame(NULL)
	, m_depthResolution(NUI_IMAGE_RESOLUTION_640x480)
	, m_colorResolution(NUI_IMAGE_RESOLUTION_640x480)
	, m_threadOn(false)
	, m_threadPause(false)
	, m_hNextDepthFrameEvent(INVALID_HANDLE_VALUE)
	, m_pDepthStreamHandle(INVALID_HANDLE_VALUE)
	, m_hNextColorFrameEvent(INVALID_HANDLE_VALUE)
	, m_pColorStreamHandle(INVALID_HANDLE_VALUE)
	, m_hNextSkeletonFrameEvent(INVALID_HANDLE_VALUE)
	, m_bNearMode(false)
	, m_bSkeletonSeatedMode(false)
	, m_minDepth(NUI_IMAGE_DEPTH_MINIMUM>>NUI_IMAGE_PLAYER_INDEX_SHIFT)
	, m_maxDepth(NUI_IMAGE_DEPTH_MAXIMUM>>NUI_IMAGE_PLAYER_INDEX_SHIFT)
	, m_minNearDepth(NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE>>NUI_IMAGE_PLAYER_INDEX_SHIFT)
	, m_maxNearDepth(NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE>>NUI_IMAGE_PLAYER_INDEX_SHIFT)
	, m_depthFrameCount(0)
	, m_lastDepthTime(0)
	, m_colorFrameCount(0)
	, m_lastColorTime(0)
{
	m_skeletonSmoothParams.fSmoothing = 0.5f;
	m_skeletonSmoothParams.fCorrection = 0.5f;
	m_skeletonSmoothParams.fPrediction = 0.5f;
	m_skeletonSmoothParams.fJitterRadius = 0.05f;
	m_skeletonSmoothParams.fMaxDeviationRadius = 0.04f;
}

NuiKinectV2Grabber::~NuiKinectV2Grabber()
{
	stopThread ();
	m_pNuiSensor = NULL;
	m_pCache = NULL;
	if(m_pMapper)
	{
		m_pMapper->Release();
		m_pMapper = NULL;
	}
	if(m_pDepthFrame)
	{
		delete m_pDepthFrame;
		m_pDepthFrame = NULL;
	}
	if(m_pColorFrame)
	{
		delete m_pColorFrame;
		m_pColorFrame = NULL;
	}
	if(m_pSkeletonFrame)
	{
		delete m_pSkeletonFrame;
		m_pSkeletonFrame = NULL;
	}
}

bool NuiKinectV2Grabber::initializeDevice(int deviceFlag)
{
	m_threadOn = false;

	assert(m_pNuiSensor);
	if(!m_pNuiSensor)
		return false;
}