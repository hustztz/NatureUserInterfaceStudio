#ifdef _WantV2Fusion_

#include "NuiKinectV2FusionGrabber.h"

#include "../DeviceCache/NuiDeviceCacheImpl.h"
#include "../DeviceCache/NuiCompositeFrame.h"
#include "Shape/NuiFusionMesh.h"

#include <iostream>
#include <assert.h>

/// <summary>
/// Set Identity in a Matrix4
/// </summary>
/// <param name="mat">The matrix to set to identity</param>
#ifndef _SetIdentityMatrix_
#define _SetIdentityMatrix_
void SetIdentityMatrix(Matrix4 &mat)
{
	mat.M11 = 1; mat.M12 = 0; mat.M13 = 0; mat.M14 = 0;
	mat.M21 = 0; mat.M22 = 1; mat.M23 = 0; mat.M24 = 0;
	mat.M31 = 0; mat.M32 = 0; mat.M33 = 1; mat.M34 = 0;
	mat.M41 = 0; mat.M42 = 0; mat.M43 = 0; mat.M44 = 1;
}
#endif

NuiKinectV2FusionGrabber::NuiKinectV2FusionGrabber()
	: m_threadOn(false)
	, m_threadPause(false)
	, m_pCache(NULL)
	, m_pVolume(NULL)
	, m_bMirrorDepthFrame(false)
	, m_bTranslateResetPoseByMinDepthThreshold(true)
	, m_bAutoResetReconstructionWhenLost(false)
	, m_bAutoResetReconstructionOnTimeout(true)
	, m_cLostFrameCounter(0)
	, m_cFrameCounter(0)
	, m_bTrackingFailed(false)
	, m_cDepthImagePixels(0)
	, m_pDepthImagePixelBuffer(NULL)
	, m_pDepthFloatImage(NULL)
	, m_pAlignedColorImage(NULL)
	, m_pPointCloud(NULL)
	, m_pShadedSurface(NULL)
	, m_bInitializeError(false)
	, m_lastFrameTimeStamp(0)
	, m_bHaveValidCameraParameters(false)
{
	// Define a cubic Kinect Fusion reconstruction volume,
	// with the Kinect at the center of the front face and the volume directly in front of Kinect.
	m_reconstructionParams.voxelsPerMeter = 256;// 1000mm / 256vpm = ~3.9mm/voxel
	m_reconstructionParams.voxelCountX = 384;   // 384 / 256vpm = 1.5m wide reconstruction
	m_reconstructionParams.voxelCountY = 384;   // Memory = 384*384*384 * 4bytes per voxel
	m_reconstructionParams.voxelCountZ = 384;   // This will require a GPU with at least 256MB

	// These parameters are for optionally clipping the input depth image 
	m_fMinDepthThreshold = NUI_FUSION_DEFAULT_MINIMUM_DEPTH;   // min depth in meters
	m_fMaxDepthThreshold = NUI_FUSION_DEFAULT_MAXIMUM_DEPTH;    // max depth in meters

	// This parameter is the temporal averaging parameter for depth integration into the reconstruction
	m_cMaxIntegrationWeight = NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT;	// Reasonable for static scenes

	// This parameter sets whether GPU or CPU processing is used. Note that the CPU will likely be 
	// too slow for real-time processing.
	m_processorType = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP;

	// If GPU processing is selected, we can choose the index of the device we would like to
	// use for processing by setting this zero-based index parameter. Note that setting -1 will cause
	// automatic selection of the most suitable device (specifically the DirectX11 compatible device 
	// with largest memory), which is useful in systems with multiple GPUs when only one reconstruction
	// volume is required. Note that the automatic choice will not load balance across multiple 
	// GPUs, hence users should manually select GPU indices when multiple reconstruction volumes 
	// are required, each on a separate device.
	m_deviceIndex = -1;    // automatically choose device index for processing

	SetIdentityMatrix(m_worldToCameraTransform);
	SetIdentityMatrix(m_defaultWorldToVolumeTransform);

	// We don't know these at object creation time, so we use nominal values.
	// These will later be updated in response to the CoordinateMappingChanged event.
	m_cameraParameters.focalLengthX = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;
	m_cameraParameters.focalLengthY = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
	m_cameraParameters.principalPointX = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
	m_cameraParameters.principalPointY = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;
}

NuiKinectV2FusionGrabber::~NuiKinectV2FusionGrabber()
{
	stopThread ();
}

/// <summary>
/// Initialize Kinect Fusion volume and images for processing
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
bool NuiKinectV2FusionGrabber::initializeKinectFusion()
{
	HRESULT hr = S_OK;
	m_threadOn = false;

	// Check to ensure suitable DirectX11 compatible hardware exists before initializing Kinect Fusion
	WCHAR description[MAX_PATH];
	WCHAR instancePath[MAX_PATH];
	UINT memorySize = 0;
#ifdef _DEBUG
	//m_processorType = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU;
#endif

	if (FAILED(hr = NuiFusionGetDeviceInfo(
		m_processorType, 
		m_deviceIndex, 
		&description[0], 
		ARRAYSIZE(description), 
		&instancePath[0],
		ARRAYSIZE(instancePath), 
		&memorySize)))
	{
		if (hr ==  E_NUI_BADINDEX)
		{
			// This error code is returned either when the device index is out of range for the processor 
			// type or there is no DirectX11 capable device installed. As we set -1 (auto-select default) 
			// for the device index in the parameters, this indicates that there is no DirectX11 capable 
			// device. The options for users in this case are to either install a DirectX11 capable device
			// (see documentation for recommended GPUs) or to switch to non-real-time CPU based 
			// reconstruction by changing the processor type to NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU.
			std::cerr << "No DirectX11 device detected, or invalid device index - Kinect Fusion requires a DirectX11 device for GPU-based reconstruction." << std::endl;
		}
		else
		{
			std::cerr << "Failed in call to NuiFusionGetDeviceInfo." << std::endl;
		}
		return false;
	}

	// Create the Kinect Fusion Reconstruction Volume
	hr = NuiFusionCreateColorReconstruction(
		&m_reconstructionParams,
		m_processorType, m_deviceIndex,
		&m_worldToCameraTransform,
		&m_pVolume);
	
	if (FAILED(hr))
	{
		if (E_NUI_GPU_FAIL == hr)
		{
			std::cerr << "Device" << m_deviceIndex << " not able to run Kinect Fusion, or error initializing." << std::endl;
		}
		else if (E_NUI_GPU_OUTOFMEMORY == hr)
		{
			std::cerr << "Device" << m_deviceIndex << " out of memory error initializing reconstruction - try a smaller reconstruction volume." << std::endl;
		}
		else if (NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU != m_processorType)
		{
			std::cerr << "Failed to initialize Kinect Fusion reconstruction volume on device: " << m_deviceIndex << std::endl;
		}
		else
		{
			std::cerr << "Failed to initialize Kinect Fusion reconstruction volume on CPU." << std::endl;
		}

		return false;
	}

	// Save the default world to volume transformation to be optionally used in ResetReconstruction
	hr = m_pVolume->GetCurrentWorldToVolumeTransform(&m_defaultWorldToVolumeTransform);
	if (FAILED(hr))
	{
		std::cerr << "Failed in call to GetCurrentWorldToVolumeTransform."<< std::endl;
		return false;
	}

	if (m_bTranslateResetPoseByMinDepthThreshold)
	{
		// This call will set the world-volume transformation
		hr = ResetReconstruction();
		if (FAILED(hr))
		{
			return false;
		}
	}

	DWORD depthWidth = NUI_DEPTH_RAW_WIDTH;
	DWORD depthHeight = NUI_DEPTH_RAW_HEIGHT;
	m_cDepthImagePixels = depthWidth * depthHeight;
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, depthWidth, depthHeight, &m_cameraParameters, &m_pAlignedColorImage);
	if (FAILED(hr))
	{
		printf_s("Failed to initialize Kinect Fusion image.");
		return false;
	}

	// Frames generated from the depth input
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, depthWidth, depthHeight, &m_cameraParameters, &m_pDepthFloatImage);
	if (FAILED(hr))
	{
		std::cerr << "Failed to initialize Kinect Fusion image." << std::endl;
		return false;
	}

	// Create images to raycast the Reconstruction Volume
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, depthWidth, depthHeight, &m_cameraParameters, &m_pPointCloud);
	if (FAILED(hr))
	{
		std::cerr << "Failed to initialize Kinect Fusion image." << std::endl;
		return false;
	}

	// Create images to raycast the Reconstruction Volume
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, depthWidth, depthHeight, &m_cameraParameters, &m_pShadedSurface);
	if (FAILED(hr))
	{
		std::cerr << "Failed to initialize Kinect Fusion image." << std::endl;
		return false;
	}

	assert(m_pDepthImagePixelBuffer == nullptr);
	m_pDepthImagePixelBuffer = new(std::nothrow) UINT16[m_cDepthImagePixels];
	if (nullptr == m_pDepthImagePixelBuffer)
	{
		std::cerr << "Failed to initialize Kinect Fusion depth image pixel buffer." << std::endl;
		return false;
	}

	//assert(m_pDepthDistortionMap == nullptr);
	//m_pDepthDistortionMap = new(std::nothrow) DepthSpacePoint[m_cDepthImagePixels];
	//if (nullptr == m_pDepthDistortionMap)
	//{
	//	std::cerr << "Failed to initialize Kinect Fusion depth image distortion buffer." << std::endl;
	//	return E_OUTOFMEMORY;
	//}

	//SAFE_DELETE_ARRAY(m_pDepthDistortionLT);
	//m_pDepthDistortionLT = new(std::nothrow) UINT[m_cDepthImagePixels];

	//if (nullptr == m_pDepthDistortionLT)
	//{
	//	std::cerr << "Failed to initialize Kinect Fusion depth image distortion Lookup Table.");
	//	return E_OUTOFMEMORY;
	//}

	//// If we have valid parameters, let's go ahead and use them.
	//if (m_cameraParameters.focalLengthX != 0)
	//{
	//	SetupUndistortion();
	//}

	return true;
}

/// <summary>
/// Reset the reconstruction camera pose and clear the volume.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT NuiKinectV2FusionGrabber::ResetReconstruction()
{
	if (nullptr == m_pVolume)
	{
		return E_FAIL;
	}

	HRESULT hr = S_OK;

	SetIdentityMatrix(m_worldToCameraTransform);

	// Translate the reconstruction volume location away from the world origin by an amount equal
	// to the minimum depth threshold. This ensures that some depth signal falls inside the volume.
	// If set false, the default world origin is set to the center of the front face of the 
	// volume, which has the effect of locating the volume directly in front of the initial camera
	// position with the +Z axis into the volume along the initial camera direction of view.
	if (m_bTranslateResetPoseByMinDepthThreshold)
	{
		Matrix4 worldToVolumeTransform = m_defaultWorldToVolumeTransform;

		// Translate the volume in the Z axis by the minDepthThreshold distance
		float minDist = (m_fMinDepthThreshold < m_fMaxDepthThreshold) ? m_fMinDepthThreshold : m_fMaxDepthThreshold;
		worldToVolumeTransform.M43 -= (minDist * m_reconstructionParams.voxelsPerMeter);

		hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, &worldToVolumeTransform);
	}
	else
	{
		hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, nullptr);
	}

	m_cLostFrameCounter = 0;
	m_cFrameCounter = 0;
	//m_fStartTime = m_timer.AbsoluteTime();

	if (SUCCEEDED(hr))
	{
		m_bTrackingFailed = false;

		printf_s("Reconstruction has been reset.");
	}
	else
	{
		printf_s("Failed to reset reconstruction.");
	}

	return hr;
}

/// <summary>
/// Perform only depth conversion and camera tracking
/// </summary>
HRESULT NuiKinectV2FusionGrabber::CameraTrackingOnly()
{
	HRESULT tracking = m_pVolume->AlignDepthFloatToReconstruction(
		m_pDepthFloatImage,
		NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
		nullptr,
		nullptr,
		nullptr);

	if (FAILED(tracking))
	{
		m_cLostFrameCounter++;
		m_bTrackingFailed = true;

		if (tracking == E_NUI_FUSION_TRACKING_ERROR)
		{
			printf_s("Kinect Fusion camera tracking failed! Align the camera to the last tracked position.");
		}
		else
		{
			printf_s("Kinect Fusion AlignDepthFloatToReconstruction call failed!");
		}
	}
	else
	{
		m_pVolume->GetCurrentWorldToCameraTransform(&m_worldToCameraTransform);
		m_cLostFrameCounter = 0;
		m_bTrackingFailed = false;
	}

	return tracking;
}

HRESULT NuiKinectV2FusionGrabber::ProcessPointCloud(NuiCompositeFrame* pFrame)
{
	assert(pFrame);
	TIMESPAN currentFrameTime = pFrame->m_depthFrame.GetTimeStamp();

	// To enable playback of a .xed file through Kinect Studio and reset of the reconstruction
	// if the .xed loops, we test for when the frame timestamp has skipped a large number. 
	// Note: this will potentially continually reset live reconstructions on slow machines which
	// cannot process a live frame in less time than the reset threshold. Increase the number of
	// milliseconds in cResetOnTimeStampSkippedMilliseconds if this is a problem.
	if (m_bAutoResetReconstructionOnTimeout && (m_cFrameCounter != 0) && (0 != m_lastFrameTimeStamp)
		&& abs(currentFrameTime - m_lastFrameTimeStamp) > cResetOnTimeStampSkippedMilliseconds * 10000)
	{
		std::cerr << "Kinect Fusion Timeout! Automatically reset volume." << std::endl;
		return ResetReconstruction();
	}
	m_lastFrameTimeStamp = currentFrameTime;

	// Return if the volume is not initialized
	if (nullptr == m_pVolume)
	{
		std::cerr << "Kinect Fusion reconstruction volume not initialized. Please try reducing volume size or restarting." << std::endl;
		return S_FALSE;
	}

	NuiDevicePointCloud cachedPointCloud;
	if( pFrame->m_pointCloudFrame.ReadPointCloud(&cachedPointCloud) )
	{
		return S_FALSE;
	}

	BYTE* pAlignedColorBuffer = m_pAlignedColorImage->pFrameBuffer->pBits;
	bool bHasColor = (pAlignedColorBuffer != nullptr);

	assert(cachedPointCloud.GetPointsNum() == m_cDepthImagePixels);
	for (unsigned int i = 0; i < cachedPointCloud.GetPointsNum(); i++)
	{
		NuiDevicePoint* pPt = cachedPointCloud.AccessPoint(i);
		if(pPt)
		{
			m_pDepthImagePixelBuffer[i] = pPt->fVertex.z;
			if(pAlignedColorBuffer)
			{
				BYTE* pColor = pAlignedColorBuffer + 4*i;
				pColor[0] = pPt->fBGRA.rgbBlue;
				pColor[1] = pPt->fBGRA.rgbGreen;
				pColor[2] = pPt->fBGRA.rgbRed;
				pColor[3] = pPt->fBGRA.rgbReserved;
			}
		}
		else
		{
			m_pDepthImagePixelBuffer[i] = 0;
		}
	}
	
	////////////////////////////////////////////////////////
	// Depth to DepthFloat

	// Convert the pixels describing extended depth as unsigned short type in millimeters to depth
	// as floating point type in meters.
	HRESULT hr = m_pVolume->DepthToDepthFloatFrame(m_pDepthImagePixelBuffer, m_cDepthImagePixels * sizeof(UINT16), m_pDepthFloatImage, m_fMinDepthThreshold, m_fMaxDepthThreshold, m_bMirrorDepthFrame);
	if (FAILED(hr))
	{
		std::cerr << "Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed." << std::endl;
		return hr;
	}

	// smoothing
	/*if (m_pVolume->SmoothDepthFloatFrame(m_pDepthFloatImage, pSmoothDepthFrame, NUI_FUSION_DEFAULT_SMOOTHING_KERNEL_WIDTH, NUI_FUSION_DEFAULT_SMOOTHING_DISTANCE_THRESHOLD) != S_OK)
	{
		std::cerr << "Dpeth Frame smooth failed" << std::endl;
	}*/

	// Perform camera tracking only from this current depth frame
	if (!bHasColor)
	{
		return CameraTrackingOnly();
	}

	////////////////////////////////////////////////////////
	// ProcessFrame

	// Perform the camera tracking and update the Kinect Fusion Volume
	// This will create memory on the GPU, upload the image, run camera tracking and integrate the
	// data into the Reconstruction Volume if successful. Note that passing nullptr as the final 
	// parameter will use and update the internal camera pose.
	hr = m_pVolume->ProcessFrame(
		m_pDepthFloatImage,
		bHasColor ? m_pAlignedColorImage : nullptr,
		NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
		m_cMaxIntegrationWeight,
		NUI_FUSION_DEFAULT_COLOR_INTEGRATION_OF_ALL_ANGLES,
		nullptr,
		&m_worldToCameraTransform);

	// Test to see if camera tracking failed. 
	// If it did fail, no data integration or raycast for reference points and normals will have taken 
	//  place, and the internal camera pose will be unchanged.
	if (FAILED(hr))
	{
		if (hr == E_NUI_FUSION_TRACKING_ERROR)
		{
			m_cLostFrameCounter++;
			m_bTrackingFailed = true;
			printf_s("Kinect Fusion camera tracking failed! Align the camera to the last tracked position. ");
		}
		else
		{
			printf_s("Kinect Fusion ProcessFrame call failed!");
			return hr;
		}
	}
	else
	{
		Matrix4 calculatedCameraPose;
		hr = m_pVolume->GetCurrentWorldToCameraTransform(&calculatedCameraPose);

		if (SUCCEEDED(hr))
		{
			// Set the pose
			m_worldToCameraTransform = calculatedCameraPose;
			m_cLostFrameCounter = 0;
			m_bTrackingFailed = false;

			pFrame->m_cameraPose = calculatedCameraPose;
		}
	}

	if (m_bAutoResetReconstructionWhenLost && m_bTrackingFailed && m_cLostFrameCounter >= cResetOnNumberOfLostFrames)
	{
		// Set bad tracking message
		printf_s("Kinect Fusion camera tracking failed, automatically reset volume.");
		// Automatically clear volume and reset tracking if tracking fails
		return ResetReconstruction();
	}

	////////////////////////////////////////////////////////
	// CalculatePointCloud

	// Raycast all the time, even if we camera tracking failed, to enable us to visualize what is happening with the system
	hr = m_pVolume->CalculatePointCloud(m_pPointCloud, (bHasColor ? m_pShadedSurface : nullptr), &m_worldToCameraTransform);
	if (FAILED(hr))
	{
		printf_s("Kinect Fusion CalculatePointCloud call failed.");
		return hr;
	}
	
	////////////////////////////////////////////////////////
	// ShadePointCloud and render
	if(!bHasColor)
	{
		hr = NuiFusionShadePointCloud(m_pPointCloud, &m_worldToCameraTransform, nullptr, m_pShadedSurface, nullptr);
		if (FAILED(hr))
		{
			printf_s("Kinect Fusion NuiFusionShadePointCloud call failed.");
			return hr;
		}
	}
	
	// Cache the shaded raycast volume image
	BGRQUAD* pFusionImageBuffer = pFrame->m_fusionImageFrame.AllocateBuffer(m_pShadedSurface->width, m_pShadedSurface->height);
	if(pFusionImageBuffer)
		memcpy(reinterpret_cast<BYTE*>(pFusionImageBuffer), m_pShadedSurface->pFrameBuffer->pBits, 4 * m_pShadedSurface->width * m_pShadedSurface->height);
	
	return hr;
}

bool NuiKinectV2FusionGrabber::readMesh(NuiFusionMesh* pMesh)
{
	if(!m_pVolume || !pMesh)
		return false;

	INuiFusionColorMesh* pFusionMesh = nullptr;
	HRESULT hr = m_pVolume->CalculateMesh(1, &pFusionMesh);
	if (SUCCEEDED(hr) && pFusionMesh)
	{
		const Vector3* pNormals = pMesh->allocateNormals( pFusionMesh->NormalCount() );
		if(pNormals)
			pFusionMesh->GetNormals(&pNormals);
		const int* pTriangleIndices = pMesh->allocateTriangleIndices( pFusionMesh->TriangleVertexIndexCount() );
		if(pTriangleIndices)
			pFusionMesh->GetTriangleIndices(&pTriangleIndices);
		const int* pColors = pMesh->allocateColors( pFusionMesh->ColorCount() );
		if(pColors)
			pFusionMesh->GetColors(&pColors);
		const Vector3* pVertices = pMesh->allocateVertices( pFusionMesh->VertexCount() );
		if(pVertices)
			pFusionMesh->GetVertices(&pVertices);
	}
	else
	{
		printf_s("Kinect Fusion CalculateMesh call failed.");
		return false;
	}
	return true;
}

void NuiKinectV2FusionGrabber::runThread ()
{
	while (true)
	{
		if (!m_threadOn)
			break;
		if (m_threadPause)
			continue;

		if (!m_pCache)
			continue;

		if(0 == m_cDepthImagePixels || !m_pDepthImagePixelBuffer)
		{
			std::cerr << "Kinect Fusion depth image buffer is empty, so cannot run fusion." << std::endl;
			continue;
		}
		if(!m_pAlignedColorImage)
		{
			std::cerr << "Kinect Fusion aligned color image buffer is empty, so cannot run fusion." << std::endl;
			continue;
		}

		NuiCompositeFrame* pFrame = m_pCache->GetLatestCompositeFrame();
		if (!pFrame)
			continue;

		if( FAILED(ProcessPointCloud(pFrame)) )
		{
			printf_s( "Failed to fuse point cloud.\n" );
		}

		//boost::this_thread::sleep (boost::posix_time::microseconds (16));
		//Sleep(16);
	}
}

void NuiKinectV2FusionGrabber::startThread ()
{
	if(m_pVolume && !m_threadOn)
	{
		ResetReconstruction();

		m_Thread.reset (new boost::thread (boost::bind (&NuiKinectV2FusionGrabber::runThread, this)));
		m_threadOn = true;

		// Set an introductory message
		printf_s("Click ��Near Mode�� to change sensor range, and ��Reset Reconstruction�� to clear!");

	}
	else if(m_pVolume && m_threadPause)
		ResetReconstruction();
	m_threadPause = false;
}

void NuiKinectV2FusionGrabber::pauseThread()
{
	m_threadPause = true;
}

void NuiKinectV2FusionGrabber::stopThread ()
{
	m_threadOn = false;
	m_threadPause = false;
	if(m_Thread)
	{
		//m_Thread->interrupt();
		m_Thread->join();
	}

	// Clean up Kinect Fusion
	SafeRelease(m_pVolume);

	if(m_pPointCloud)
	{
		NuiFusionReleaseImageFrame(m_pPointCloud);
		m_pPointCloud = NULL;
	}
	if(m_pDepthFloatImage)
	{
		NuiFusionReleaseImageFrame(m_pDepthFloatImage);
		m_pDepthFloatImage = NULL;
	}
	if(m_pAlignedColorImage)
	{
		NuiFusionReleaseImageFrame(m_pAlignedColorImage);
		m_pAlignedColorImage = NULL;
	}
	if(m_pShadedSurface)
	{
		NuiFusionReleaseImageFrame(m_pShadedSurface);
		m_pShadedSurface = NULL;
	}

	SafeDeleteArray(m_pDepthImagePixelBuffer);
	m_cDepthImagePixels = 0;
}

void NuiKinectV2FusionGrabber::updateCache(NuiRGBDDeviceBufferImpl* pCache)
{
	m_pCache = pCache;
}

#endif