#pragma once

#include <NuiKinectFusionApi.h>
#include <boost/thread/thread.hpp>

//Forwards
class NuiFusionMesh;
class NuiDeviceCacheImpl;
class NuiCompositeFrame;

class NuiKinectFusionGrabber
{
	static const int            cResetOnTimeStampSkippedMilliseconds = 1000;
	static const int            cResetOnNumberOfLostFrames = 100;
	static const int            cTimeDisplayInterval = 10;

public:
	NuiKinectFusionGrabber();
	~NuiKinectFusionGrabber();

	bool								initializeKinectFusion();
	void								updateCache(NuiDeviceCacheImpl* pCache);
	
	void								startThread ();
	void								stopThread();
	void								pauseThread();
	bool								isThreadOn() const { return m_threadOn; }

	bool								readMesh(NuiFusionMesh* pMesh);

private:
	void								runThread ();
	HRESULT								ResetReconstruction();
	HRESULT								CameraTrackingOnly();
	HRESULT								ProcessPointCloud(NuiCompositeFrame* pFrame);

private:
	boost::shared_ptr<boost::thread>	m_Thread;
	bool								m_threadOn;
	bool								m_threadPause;

	NuiDeviceCacheImpl*					m_pCache;

	TIMESPAN							m_lastFrameTimeStamp;
	/// <summary>
	/// The Kinect Fusion Reconstruction Volume
	/// </summary>
	INuiFusionColorReconstruction*		m_pVolume;

	/// <summary>
	/// The Kinect Fusion Volume Parameters
	/// </summary>
	NUI_FUSION_RECONSTRUCTION_PARAMETERS m_reconstructionParams;

	/// <summary>
	// The Kinect Fusion Camera Transform
	/// </summary>
	Matrix4								m_worldToCameraTransform;

	/// <summary>
	// The default Kinect Fusion World to Volume Transform
	/// </summary>
	Matrix4								m_defaultWorldToVolumeTransform;

	/// <summary>
	/// Frames from the depth input
	/// </summary>
	NUI_FUSION_IMAGE_FRAME*				m_pDepthFloatImage;
	UINT								m_cDepthImagePixels;
	UINT16*								m_pDepthImagePixelBuffer;
	NUI_FUSION_IMAGE_FRAME*				m_pAlignedColorImage;

	/// <summary>
	/// Frames generated from ray-casting the Reconstruction Volume
	/// </summary>
	NUI_FUSION_IMAGE_FRAME*				m_pPointCloud;

	/// <summary>
	/// Images for display
	/// </summary>
	NUI_FUSION_IMAGE_FRAME*				m_pShadedSurface;

	/// <summary>
	/// Camera Tracking parameters
	/// </summary>
	int									m_cLostFrameCounter;
	bool								m_bTrackingFailed;

	/// <summary>
	/// Kinect camera parameters.
	/// </summary>
	NUI_FUSION_CAMERA_PARAMETERS		m_cameraParameters;
	bool								m_bHaveValidCameraParameters;

	/// <summary>
	/// Parameter to turn automatic reset of the reconstruction when camera tracking is lost on or off.
	/// Set to true in the constructor to enable auto reset on cResetOnNumberOfLostFrames lost frames,
	/// or set false to never automatically reset.
	/// </summary>
	bool								m_bAutoResetReconstructionWhenLost;

	/// <summary>
	/// Parameter to enable automatic reset of the reconstruction when there is a large
	/// difference in timestamp between subsequent frames. This should usually be set true as 
	/// default to enable recorded .xed files to generate a reconstruction reset on looping of
	/// the playback or scrubbing, however, for debug purposes, it can be set false to prevent
	/// automatic reset on timeouts.
	/// </summary>
	bool								m_bAutoResetReconstructionOnTimeout;

	/// <summary>
	/// Processing parameters
	/// </summary>
	int									m_deviceIndex;
	NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE m_processorType;
	bool								m_bInitializeError;
	float								m_fMinDepthThreshold;
	float								m_fMaxDepthThreshold;
	bool								m_bMirrorDepthFrame;
	unsigned short						m_cMaxIntegrationWeight;
	int									m_cFrameCounter;

	/// <summary>
	/// Parameter to translate the reconstruction based on the minimum depth setting. When set to
	/// false, the reconstruction volume +Z axis starts at the camera lens and extends into the scene.
	/// Setting this true in the constructor will move the volume forward along +Z away from the
	/// camera by the minimum depth threshold to enable capture of very small reconstruction volumes
	/// by setting a non-identity camera transformation in the ResetReconstruction call.
	/// Small volumes should be shifted, as the Kinect hardware has a minimum sensing limit of ~0.35m,
	/// inside which no valid depth is returned, hence it is difficult to initialize and track robustly  
	/// when the majority of a small volume is inside this distance.
	/// </summary>
	bool								m_bTranslateResetPoseByMinDepthThreshold;
};
