#include "NuiKinectV1Manager.h"
#include "../NuiRGBDDeviceBufferImpl.h"

/// <summary>
/// This function will be called when Kinect device status changed
/// </summary>
void CALLBACK NuiKinectV1Manager::StatusChangeCallback(HRESULT hrStatus, const OLECHAR* instancename, const OLECHAR* uniqueDeviceName, void* pUserData)
{
	NuiKinectV1Manager* hManager = reinterpret_cast<NuiKinectV1Manager*>(pUserData);

	if ( SUCCEEDED( hrStatus ) )
	{

	}
	else
	{

	}
}

NuiKinectV1Manager::NuiKinectV1Manager(NuiRGBDDeviceBufferImpl* pBuffer)
	: m_pNuiSensor(NULL)
	, m_pBuffer(pBuffer)
	, m_pGrabber(NULL)
	, m_pFaceTracker(NULL)
	, m_pFusion(NULL)
{
	// Set the sensor status callback
	NuiSetDeviceStatusCallback(StatusChangeCallback, reinterpret_cast<void*>(this));
}

NuiKinectV1Manager::~NuiKinectV1Manager()
{
	ShutdownDevice ();
	m_pBuffer = nullptr;
}

/// <summary>
/// Create the first connected Kinect found 
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT NuiKinectV1Manager::CreateFirstConnected()
{
	INuiSensor * pNuiSensor = NULL;
	HRESULT hr;

	int iSensorCount = 0;
	hr = NuiGetSensorCount(&iSensorCount);
	if (FAILED(hr) ) { return hr; }

	// Look at each Kinect sensor
	for (int i = 0; i < iSensorCount; ++i)
	{
		// Create the sensor so we can check status, if we can't create it, move on to the next
		hr = NuiCreateSensorByIndex(i, &pNuiSensor);
		if (FAILED(hr))
		{
			continue;
		}

		// Get the status of the sensor, and if connected, then we can initialize it
		hr = pNuiSensor->NuiStatus();
		if (S_OK == hr)
		{
			m_pNuiSensor = pNuiSensor;
			break;
		}

		// This sensor wasn't OK, so release it since we're not using it
		pNuiSensor->Release();
	}

	if (NULL == m_pNuiSensor)
	{
		return E_FAIL;
	}

	return hr;
}

bool NuiKinectV1Manager::InitializeDevice (DWORD deviceFlag)
{
	if(!m_pNuiSensor)
	{
		if( FAILED(CreateFirstConnected()) )
			return false;
	}
	assert(m_pNuiSensor);
	if(!m_pNuiSensor)
		return false;

	// Check initialize flag
	DWORD dwFlags = 0;
	if(deviceFlag & EDevice_Depth_On)
	{
		if(deviceFlag & EDevice_PlayIndex_On)
			dwFlags |= NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX;
		else
			dwFlags |= NUI_INITIALIZE_FLAG_USES_DEPTH;
	}
	if(deviceFlag & EDevice_Color_On)
		dwFlags |= NUI_INITIALIZE_FLAG_USES_COLOR;
	if(deviceFlag & EDevice_Skeleton_On)
		dwFlags |= NUI_INITIALIZE_FLAG_USES_SKELETON;
	if(!IsDeviceOn() || (m_pNuiSensor->NuiInitializationFlags() != dwFlags))
	{
		if(!m_pGrabber)
		{
			m_pGrabber = new NuiKinectV1Grabber(m_pNuiSensor);
		}
		assert(m_pGrabber);
		if(!m_pGrabber)
			return false;
		if( FAILED(m_pGrabber->setupDevice(dwFlags, cDepthResolution, cColorResolution)) )
			return false;
	}

	if(deviceFlag & (EDevice_Face_On | EDevice_FacialModel_On))
	{
		assert(m_pGrabber);
		if(m_pGrabber)
		{
			m_pGrabber->setSkeletonSeatedMode(true);
		}
		if(!m_pFaceTracker)
		{
			m_pFaceTracker = new NuiKinectV1FaceTracker();
		}
		assert(m_pFaceTracker);
		if( !m_pFaceTracker->setupTracker(cDepthResolution, cColorResolution) )
		{
			std::cerr << "Failed initialize KinectV1 face tracking." << std::endl;
			SafeDelete(m_pFaceTracker);
		}
	}
	else
	{
		if(m_pGrabber)
		{
			m_pGrabber->setSkeletonSeatedMode(false);
		}
		if(m_pFaceTracker)
			m_pFaceTracker->stopThread();
	}

	if(deviceFlag & EDevice_Fusion_On)
	{
		if(!m_pFusion)
		{
			m_pFusion = new NuiKinectV1FusionGrabber();
		}
		assert(m_pFusion);
		if( !m_pFusion->initializeKinectFusion(cDepthResolution, cColorResolution) )
		{
			std::cerr << "Failed initialize KinectV1 fusion." << std::endl;
			SafeDelete(m_pFusion);
		}
	}
	else
	{
		if(m_pFusion)
			m_pFusion->stopThread();
	}
	updateCacheStatus();

	return IsDeviceOn();
}

bool NuiKinectV1Manager::StartDevice ()
{
	if(!IsDeviceOn())
	{
		//std::cerr << "Can't start sensor since it's not opened." << std::endl;
		return false;
	}

	if(m_pGrabber)
		m_pGrabber->startThread();
	if(m_pFaceTracker)
		m_pFaceTracker->startThread();
	if(m_pFusion)
		m_pFusion->startThread();
	return true;
}

void NuiKinectV1Manager::PauseDevice ()
{
	if(m_pGrabber)
		m_pGrabber->pauseThread();
	if(m_pFaceTracker)
		m_pFaceTracker->pauseThread();
	if(m_pFusion)
		m_pFusion->pauseThread();
}

void NuiKinectV1Manager::ShutdownDevice()
{
	if(m_pGrabber)
		m_pGrabber->pauseThread();
	if (NULL != m_pNuiSensor)
	{
		m_pNuiSensor->NuiShutdown();
		m_pNuiSensor->Release();
		m_pNuiSensor = NULL;
	}
	SafeDelete(m_pGrabber);
	SafeDelete(m_pFaceTracker);
	SafeDelete(m_pFusion);
}

bool NuiKinectV1Manager::IsDeviceOn() const
{
	return ( m_pNuiSensor && (S_OK == m_pNuiSensor->NuiStatus()) && m_pGrabber );
}

void NuiKinectV1Manager::updateCacheStatus()
{
	if ( m_pGrabber )
		m_pGrabber->updateCache(m_pBuffer);
	if(m_pFaceTracker)
		m_pFaceTracker->updateCache(m_pBuffer);
	if(m_pFusion)
		m_pFusion->updateCache(m_pBuffer);
}

bool NuiKinectV1Manager::UpdateNearMode(bool bNearMode)
{
	if(m_pGrabber)
	{
		m_pGrabber->setNearMode(bNearMode);
		return m_pGrabber->getNearMode();
	}
	return false;
}

void NuiKinectV1Manager::UpdateElevationAngle(LONG tiltAngle)
{
	if ( m_pNuiSensor )
		m_pNuiSensor->NuiCameraElevationSetAngle(CoerceElevationAngle(tiltAngle));
}

void NuiKinectV1Manager::UpdateSkeletonSmoothParams(float smoothing, float correction, float prediction, float jitterRadius, float maxDeviationRadius)
{
	if ( m_pGrabber )
		m_pGrabber->updateSkeletonSmoothParams(smoothing, correction, prediction, jitterRadius, maxDeviationRadius);
}
