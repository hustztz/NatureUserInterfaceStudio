// File: PCEKinectFusion.cpp
//
// Author: Tingzhu Zhou
//
#include "NuiKinectV1FusionGrabber.h"

#include "../NuiRGBDDeviceBufferImpl.h"
#include "Frame/NuiCompositeFrame.h"
#include "Shape/NuiFusionMesh.h"

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

NuiKinectV1FusionGrabber::NuiKinectV1FusionGrabber()
	: m_threadOn(false)
	, m_threadPause(false)
	, m_pCache(NULL)
	, m_pVolume(NULL)
	, m_bMirrorDepthFrame(false)
	, m_bTranslateResetPoseByMinDepthThreshold(true)
	, m_bAutoResetReconstructionWhenLost(false)
	, m_bAutoResetReconstructionOnTimeout(true)
	, m_cLostFrameCounter(0)
	, m_bTrackingFailed(false)
	, m_pDepthFloatImage(NULL)
	, m_pDepthImagePixelBuffer(NULL)
	, m_pAlignedColorImage(NULL)
	, m_pPointCloud(NULL)
	, m_pShadedSurface(NULL)
	, m_bInitializeError(false)
	, m_cLastDepthFrameTimeStamp(0)
{
	// Define a cubic Kinect Fusion reconstruction volume,
	// with the Kinect at the center of the front face and the volume directly in front of Kinect.
	m_reconstructionParams.voxelsPerMeter = 256;// 1000mm / 256vpm = ~3.9mm/voxel    
	m_reconstructionParams.voxelCountX = 512;   // 512 / 256vpm = 2m wide reconstruction
	m_reconstructionParams.voxelCountY = 384;   // Memory = 512*384*512 * 4bytes per voxel
	m_reconstructionParams.voxelCountZ = 512;   // This will require a GPU with at least 512MB

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
}

NuiKinectV1FusionGrabber::~NuiKinectV1FusionGrabber()
{
	stopThread ();
}

/// <summary>
/// Initialize Kinect Fusion volume and images for processing
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
bool NuiKinectV1FusionGrabber::initializeKinectFusion(NUI_IMAGE_RESOLUTION depthResolution, NUI_IMAGE_RESOLUTION	colorResolution)
{
	HRESULT hr = S_OK;
	m_threadOn = false;

	// Check to ensure suitable DirectX11 compatible hardware exists before initializing Kinect Fusion
	WCHAR description[MAX_PATH];
	WCHAR instancePath[MAX_PATH];
	UINT memorySize = 0;
	m_processorType = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU;

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
			printf("No DirectX11 device detected, or invalid device index - Kinect Fusion requires a DirectX11 device for GPU-based reconstruction.");
		}
		else
		{
			printf("Failed in call to NuiFusionGetDeviceInfo.");
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
			printf_s("Device %d not able to run Kinect Fusion, or error initializing.", m_deviceIndex);
		}
		else if (E_NUI_GPU_OUTOFMEMORY == hr)
		{
			printf_s("Device %d out of memory error initializing reconstruction - try a smaller reconstruction volume.", m_deviceIndex);
		}
		else if (NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU != m_processorType)
		{
			printf_s("Failed to initialize Kinect Fusion reconstruction volume on device %d.", m_deviceIndex);
		}
		else
		{
			printf_s("Failed to initialize Kinect Fusion reconstruction volume on CPU.");
		}

		return false;
	}

	// Save the default world to volume transformation to be optionally used in ResetReconstruction
	hr = m_pVolume->GetCurrentWorldToVolumeTransform(&m_defaultWorldToVolumeTransform);
	if (FAILED(hr))
	{
		printf_s("Failed in call to GetCurrentWorldToVolumeTransform.");
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

	DWORD colorWidth = 0;
	DWORD colorHeight = 0;
	NuiImageResolutionToSize(colorResolution, colorWidth, colorHeight);
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, colorWidth, colorHeight, nullptr, &m_pAlignedColorImage);
	if (FAILED(hr))
	{
		printf_s("Failed to initialize Kinect Fusion image.");
		return false;
	}

	DWORD depthWidth = 0;
	DWORD depthHeight = 0;
	m_cDepthImagePixels = depthWidth * depthHeight;
	NuiImageResolutionToSize(depthResolution, depthWidth, depthHeight);
	m_cDepthImagePixels = depthWidth * depthHeight;
	// Frames generated from the depth input
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, depthWidth, depthHeight, nullptr, &m_pDepthFloatImage);
	if (FAILED(hr))
	{
		printf_s("Failed to initialize Kinect Fusion image.");
		return false;
	}

	// Create images to raycast the Reconstruction Volume
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, depthWidth, depthHeight, nullptr, &m_pPointCloud);
	if (FAILED(hr))
	{
		printf_s("Failed to initialize Kinect Fusion image.");
		return false;
	}

	// Create images to raycast the Reconstruction Volume
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, depthWidth, depthHeight, nullptr, &m_pShadedSurface);
	if (FAILED(hr))
	{
		printf_s("Failed to initialize Kinect Fusion image.");
		return false;
	}

	assert(m_pDepthImagePixelBuffer == nullptr);
	m_pDepthImagePixelBuffer = new(std::nothrow) NUI_DEPTH_IMAGE_PIXEL[m_cDepthImagePixels];
	if (nullptr == m_pDepthImagePixelBuffer)
	{
		std::cerr << "Failed to initialize Kinect Fusion depth image pixel buffer." << std::endl;
		return false;
	}

	return true;
}

/// <summary>
/// Reset the reconstruction camera pose and clear the volume.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT NuiKinectV1FusionGrabber::ResetReconstruction()
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
HRESULT NuiKinectV1FusionGrabber::CameraTrackingOnly()
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

bool NuiKinectV1FusionGrabber::readFrame(NuiCompositeFrame* pCompositeFrame, bool* bHasColor)
{
	assert(pCompositeFrame);
	if(!pCompositeFrame)
		return false;

	pCompositeFrame->m_compressedDepthBuffer.WriteFrameLock();
	USHORT* pCachedDepthBuffer = pCompositeFrame->m_compressedDepthBuffer.GetBuffer();
	if(pCachedDepthBuffer && m_pDepthImagePixelBuffer)
	{
		errno_t err = memcpy_s(m_pDepthImagePixelBuffer, m_cDepthImagePixels * sizeof(NUI_DEPTH_IMAGE_PIXEL), reinterpret_cast<BYTE*>(pCachedDepthBuffer), pCompositeFrame->m_compressedDepthBuffer.GetBufferSize());
		*bHasColor = (0 == err);
	}
	pCompositeFrame->m_compressedDepthBuffer.WriteFrameUnlock();

	bool readStatus = false;
	NUI_LOCKED_RECT LockedRect;
	HRESULT hr = m_pAlignedColorImage->pFrameTexture->LockRect(0, &LockedRect, NULL, 0);
	if ( SUCCEEDED(hr) )
	{
		if (LockedRect.Pitch != 0)
		{
			BGRQUAD* pCachedColorBuffer = pCompositeFrame->m_colorFrame.GetBuffer();
			pCompositeFrame->m_colorFrame.WriteFrameLock();
			if(pCachedColorBuffer)
			{
				errno_t err = memcpy_s(PBYTE(LockedRect.pBits), m_pAlignedColorImage->pFrameTexture->BufferLen(), reinterpret_cast<BYTE*>(pCachedColorBuffer), pCompositeFrame->m_colorFrame.GetBufferSize());
				readStatus = (0 == err);
			}
			pCompositeFrame->m_colorFrame.WriteFrameUnlock();
		}
	}
	
	return readStatus;
}

HRESULT NuiKinectV1FusionGrabber::FusePointCloud(LONGLONG currentDepthFrameTime, bool bHasColor)
{
	HRESULT hr = S_OK;

	// To enable playback of a .xed file through Kinect Studio and reset of the reconstruction
	// if the .xed loops, we test for when the frame timestamp has skipped a large number. 
	// Note: this will potentially continually reset live reconstructions on slow machines which
	// cannot process a live frame in less time than the reset threshold. Increase the number of
	// milliseconds in cResetOnTimeStampSkippedMilliseconds if this is a problem.
	if (m_bAutoResetReconstructionOnTimeout /*&& m_cachedFrameIndex > 0*/
		&& abs(currentDepthFrameTime - m_cLastDepthFrameTimeStamp) > cResetOnTimeStampSkippedMilliseconds)
	{
		printf_s("Kinect Fusion Timeout! Automatically reset volume.");
		return ResetReconstruction();
	}

	m_cLastDepthFrameTimeStamp = currentDepthFrameTime;

	// Return if the volume is not initialized
	if (nullptr == m_pVolume || !m_pCache)
	{
		printf_s("Kinect Fusion reconstruction volume not initialized. Please try reducing volume size or restarting.");
		return E_FAIL;
	}

	////////////////////////////////////////////////////////
	// Depth to DepthFloat

	// Convert the pixels describing extended depth as unsigned short type in millimeters to depth
	// as floating point type in meters.
	hr = m_pVolume->DepthToDepthFloatFrame(m_pDepthImagePixelBuffer, m_cDepthImagePixels * sizeof(NUI_DEPTH_IMAGE_PIXEL), m_pDepthFloatImage, m_fMinDepthThreshold, m_fMaxDepthThreshold, m_bMirrorDepthFrame);
	if (FAILED(hr))
	{
		printf_s("Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed.");
		return E_FAIL;
	}

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

			//m_pCache->setCameraPose(m_cachedFrameIndex, calculatedCameraPose);
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
	
	// Draw the shaded raycast volume image
	INuiFrameTexture * pShadedImageTexture = m_pShadedSurface->pFrameTexture;
	NUI_LOCKED_RECT ShadedLockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	hr = pShadedImageTexture->LockRect(0, &ShadedLockedRect, nullptr, 0);
	if (FAILED(hr))
	{
		return hr;
	}

	// Make sure we've received valid data
	if (ShadedLockedRect.Pitch != 0)
	{
		BYTE * pBuffer = (BYTE *)ShadedLockedRect.pBits;

		//m_pCache->setFusionImage(m_cachedFrameIndex, m_pShadedSurface->width, m_pShadedSurface->height, pBuffer);
	}

	// We're done with the texture so unlock it
	pShadedImageTexture->UnlockRect(0);
	
	return hr;
}

bool NuiKinectV1FusionGrabber::readMesh(NuiFusionMesh* pMesh)
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

void NuiKinectV1FusionGrabber::runThread ()
{
	while (true)
	{
		if (!m_threadOn)
			break;
		if (m_threadPause)
			continue;

		if (!m_pCache)
			continue;

		std::shared_ptr<NuiCompositeFrame> pCompositeFrame = m_pCache->popFrame();
		if( !pCompositeFrame )
		{
			printf_s( "Failed to acquire the latest composite frame in fusion.\n" );
			continue;
		}

		bool bHasColor = true;
		if( !readFrame(pCompositeFrame.get(), &bHasColor) )
		{
			printf_s( "Failed to read cached images in face tracker.\n" );
			continue;
		}

		if( FAILED(FusePointCloud(pCompositeFrame->m_depthFrame.GetTimeStamp(), bHasColor)) )
		{
			printf_s( "Failed to fuse point cloud.\n" );
		}

		//boost::this_thread::sleep (boost::posix_time::microseconds (16));
		//Sleep(16);
	}
}

void NuiKinectV1FusionGrabber::startThread ()
{
	if(m_pVolume && !m_threadOn)
	{
		ResetReconstruction();

		m_Thread.reset (new boost::thread (boost::bind (&NuiKinectV1FusionGrabber::runThread, this)));
		m_threadOn = true;

		// Set an introductory message
		printf_s("Click ¡®Near Mode¡¯ to change sensor range, and ¡®Reset Reconstruction¡¯ to clear!");

	}
	else if(m_pVolume && m_threadPause)
		ResetReconstruction();
	m_threadPause = false;
}

void NuiKinectV1FusionGrabber::pauseThread()
{
	m_threadPause = true;
}

void NuiKinectV1FusionGrabber::stopThread ()
{
	m_threadOn = false;
	m_threadPause = false;
	if(m_Thread)
	{
		//m_Thread->interrupt();
		m_Thread->join();
	}

	// Clean up Kinect Fusion
	if(m_pVolume)
	{
		m_pVolume->Release();
		m_pVolume = NULL;
	}

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

void NuiKinectV1FusionGrabber::updateCache(NuiRGBDDeviceBufferImpl* pCache)
{
	m_pCache = pCache;
}
