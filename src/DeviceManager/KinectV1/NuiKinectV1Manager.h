#pragma once

#include "..\NuiRGBDDeviceManagerImpl.h"
#include "NuiKinectV1Grabber.h"
#include "NuiKinectV1FaceTracker.h"
#include "NuiKinectV1FusionGrabber.h"

// PCEKinectManager thread class
class NuiKinectV1Manager : public NuiRGBDDeviceManagerImpl
{
	static const NUI_IMAGE_RESOLUTION   cDepthResolution = NUI_IMAGE_RESOLUTION_640x480;
	static const NUI_IMAGE_RESOLUTION   cColorResolution = NUI_IMAGE_RESOLUTION_640x480;
public:
	NuiKinectV1Manager(NuiRGBDDeviceBufferImpl* pBuffer);
	virtual ~NuiKinectV1Manager();

	virtual bool		InitializeDevice(DWORD deviceFlag) override;
	virtual bool		IsDeviceOn() const override;
	virtual bool		StartDevice() override;
	virtual void		PauseDevice() override;
	virtual void		ShutdownDevice() override;

	virtual bool		UpdateNearMode(bool bNearMode) override;
	virtual void		UpdateElevationAngle(long tiltAngle) override;
	virtual void		UpdateSkeletonSmoothParams(float smoothing, float correction, float prediction, float jitterRadius, float maxDeviationRadius) override;

protected:
	/// <summary>
	/// Create the first connected Kinect found 
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT				CreateFirstConnected();
	/// <summary>
	/// Coerce the requested elevation angle to a valid angle
	/// </summary>
	/// <param name="tiltAngle">The requested angle</param>
	/// <returns>The angle after coerced</returns>
	inline long			CoerceElevationAngle(long tiltAngle)
	{
		return std::min(std::max((long)NUI_CAMERA_ELEVATION_MINIMUM, tiltAngle), (long)NUI_CAMERA_ELEVATION_MAXIMUM);
	}

	void				updateCacheStatus();

	/// <summary>
	/// Called on Kinect device status changed. It will update the sensor chooser UI control
	/// based on the new sensor status. It may also updates the sensor instance if needed
	/// </summary>
	static void CALLBACK StatusChangeCallback(HRESULT hrStatus, const OLECHAR* instancename, const OLECHAR* uniqueDeviceName, void* pUserData);

private:
	INuiSensor*							m_pNuiSensor;
	NuiRGBDDeviceBufferImpl*				m_pBuffer;

	NuiKinectV1Grabber*					m_pGrabber;
	NuiKinectV1FaceTracker*				m_pFaceTracker;
	NuiKinectV1FusionGrabber*			m_pFusion;
};

