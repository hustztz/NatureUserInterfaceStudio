#include "stdafx.h"
#include <omp.h>

#include "NuiKinectV1Grabber.h"

#include "../NuiRGBDDeviceBufferImpl.h"
#include "Frame/NuiCompositeFrame.h"

NuiKinectV1Grabber::NuiKinectV1Grabber(INuiSensor* pNuiSensor)
	: m_pNuiSensor(pNuiSensor)
	, m_pMapper(NULL)
	, m_pCache(NULL)
	, m_threadOn(false)
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
	, m_depthResolution(NUI_IMAGE_RESOLUTION_640x480)
	, m_colorResolution(NUI_IMAGE_RESOLUTION_640x480)
	, m_pColorToDepth(nullptr)
{
	m_skeletonSmoothParams.fSmoothing = 0.5f;
	m_skeletonSmoothParams.fCorrection = 0.5f;
	m_skeletonSmoothParams.fPrediction = 0.5f;
	m_skeletonSmoothParams.fJitterRadius = 0.05f;
	m_skeletonSmoothParams.fMaxDeviationRadius = 0.04f;
}

NuiKinectV1Grabber::~NuiKinectV1Grabber()
{
	stopThread ();
	m_pNuiSensor = NULL;
	m_pCache = NULL;
}

bool NuiKinectV1Grabber::setupDevice(DWORD dwFlags, NUI_IMAGE_RESOLUTION depthResolution, NUI_IMAGE_RESOLUTION colorResolution)
{
	m_threadOn = false;

	assert(m_pNuiSensor);
	if(!m_pNuiSensor)
		return false;

	m_depthResolution = depthResolution;
	m_colorResolution = colorResolution;

	HRESULT hr;
	// Initialize the Kinect and specify that we'll be using depth
	hr = m_pNuiSensor->NuiInitialize(dwFlags);
	if (FAILED(hr) )
	{
		printf_s("Device Initialize failed.\n");
		return false;
	}

	// Initialize the image buffer
	//WCHAR *sensorId = m_pNuiSensor->NuiDeviceConnectionId();

	// Open the streams
	if(dwFlags & (NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX))
	{
		// Create an event that will be signaled when depth data is available
		if(INVALID_HANDLE_VALUE == m_hNextDepthFrameEvent)
			m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		else
			ResetEvent(m_hNextDepthFrameEvent);

		// Open a depth image stream to receive depth frames
		NUI_IMAGE_TYPE imageType = (dwFlags & NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX) ? NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX : NUI_IMAGE_TYPE_DEPTH;
		hr = m_pNuiSensor->NuiImageStreamOpen(
			imageType,
			m_depthResolution,
			0,
			2,
			m_hNextDepthFrameEvent,
			&m_pDepthStreamHandle);
		if (SUCCEEDED(hr) )
		{
			DWORD nDepthWidth = 0;
			DWORD nDepthHeight = 0;
			NuiImageResolutionToSize(m_depthResolution, nDepthWidth, nDepthHeight);
			m_depthFrameBuffer.AllocateBuffer((UINT)nDepthWidth, (UINT)nDepthHeight);
		}
		else
		{
			printf_s("Depth Frame Initialize failed.\n");
		}
	}

	if(dwFlags & NUI_INITIALIZE_FLAG_USES_COLOR)
	{
		// Create an event that will be signaled when color data is available
		if(INVALID_HANDLE_VALUE == m_hNextColorFrameEvent)
			m_hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		else
			ResetEvent(m_hNextColorFrameEvent);

		// Open a color image stream to receive color frames
		hr = m_pNuiSensor->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_COLOR,
			m_colorResolution,
			0,
			2,
			m_hNextColorFrameEvent,
			&m_pColorStreamHandle );
		if (SUCCEEDED(hr) )
		{
			DWORD nColorWidth = 0;
			DWORD nColorHeight = 0;
			NuiImageResolutionToSize(m_colorResolution, nColorWidth, nColorHeight);
			m_colorFrameBuffer.AllocateBuffer((UINT)nColorWidth, (UINT)nColorHeight);
		}
		else
		{
			printf_s("Depth Frame Initialize failed.\n");
		}
	}
	
	if(dwFlags & NUI_INITIALIZE_FLAG_USES_SKELETON)
	{
		// Create an event that will be signaled when skeleton data is available
		if(INVALID_HANDLE_VALUE == m_hNextSkeletonFrameEvent)
			m_hNextSkeletonFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		else
			ResetEvent(m_hNextSkeletonFrameEvent);

		hr = m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonFrameEvent, 0);
		if (FAILED(hr) )
		{
			printf_s("Skeleton Frame Initialize failed.\n");
		}
	}

	// Create the coordinate mapper for converting color to depth space
	SafeRelease(m_pMapper);
	hr = m_pNuiSensor->NuiGetCoordinateMapper(&m_pMapper);
	if (FAILED(hr) )
	{
		printf_s("Failed to get mapper.\n");
	}

	updateNearMode();

	return true;
}

/// <summary>
/// Process depth data received from Kinect
/// </summary>
/// <returns>S_OK for success, or failure code</returns>
bool NuiKinectV1Grabber::grabDepthFrame(NuiCompositeFrame* pCompositeFrame)
{
	NUI_IMAGE_FRAME imageFrame;
	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
	if ( FAILED(hr) )
	{
		printf_s("Failed to grab depth image.\n");
		return false;
	}

	bool needCompressedDepth = false;
	if(needCompressedDepth && pCompositeFrame)
	{
		INuiFrameTexture* pCompressedFrameTexture = imageFrame.pFrameTexture;
		NUI_LOCKED_RECT LockedCompressedRect;
		hr = pCompressedFrameTexture->LockRect(0, &LockedCompressedRect, NULL, 0);
		if ( FAILED(hr) )
		{
			printf_s("Failed to grab depth frame.\n");
			return false;
		}
		if (LockedCompressedRect.Pitch != 0)
		{
			USHORT* dstBuffer = pCompositeFrame->m_compressedDepthBuffer.AllocateBuffer(m_depthFrameBuffer.GetWidth(), m_depthFrameBuffer.GetHeight());
			assert(dstBuffer);
			if(dstBuffer)
			{
				errno_t err = memcpy_s(dstBuffer, pCompositeFrame->m_compressedDepthBuffer.GetBufferSize(), PBYTE(LockedCompressedRect.pBits), pCompressedFrameTexture->BufferLen());
				if(0 != err)
				{
					printf_s("No compressed depth frame buffer available.\n");
				}
				else
				{
					pCompositeFrame->m_compressedDepthBuffer.SetTimeStamp(imageFrame.liTimeStamp.QuadPart);
				}
			}
		}
		hr = pCompressedFrameTexture->UnlockRect(0);
	}

	/////////////////////////////
	INuiFrameTexture* pFrameTexture = nullptr;
	hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(m_pDepthStreamHandle, &imageFrame, &m_bNearMode, &pFrameTexture);
	if ( FAILED(hr) )
	{
		printf_s("Failed to grab depth frame.\n");
		return false;
	}

	NUI_LOCKED_RECT LockedRect;
	hr = pFrameTexture->LockRect(0, &LockedRect, NULL, 0);
	if ( FAILED(hr) )
	{
		printf_s("Failed to grab depth image.\n");
		return false;
	}
	bool returnStatus = false;
	if (LockedRect.Pitch != 0)
	{
		NUI_DEPTH_IMAGE_PIXEL* dstBuffer = m_depthFrameBuffer.GetBuffer();
		assert(dstBuffer);
		if(dstBuffer)
		{
			errno_t err = memcpy_s(dstBuffer, m_depthFrameBuffer.GetBufferSize(), PBYTE(LockedRect.pBits), pFrameTexture->BufferLen());
			returnStatus = (0 == err);
			if(returnStatus)
				m_depthFrameBuffer.SetTimeStamp(imageFrame.liTimeStamp.QuadPart);
		}
	}

	hr = pFrameTexture->UnlockRect(0);
	hr = m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);
	if ( FAILED(hr) )
	{
		printf_s("Failed to release depth frame.\n");
	}

	return returnStatus;
}

/// <summary>
/// Process color data received from Kinect
/// </summary>
/// <returns>S_OK for success, or failure code</returns>
bool NuiKinectV1Grabber::grabColorFrame()
{
	NUI_IMAGE_FRAME imageFrame;
	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pColorStreamHandle, 0, &imageFrame);
	if ( FAILED(hr) )
	{
		printf_s("Failed to grab color frame.\n");
		return false;
	}

	NUI_LOCKED_RECT LockedRect;
	hr = imageFrame.pFrameTexture->LockRect(0, &LockedRect, NULL, 0);
	if ( FAILED(hr) )
	{
		printf_s("Failed to grab color image.\n");
		return false;
	}

	bool returnStatus = false;
	if (LockedRect.Pitch != 0)
	{
		BGRQUAD* pColorBuffer = m_colorFrameBuffer.GetBuffer();
		size_t nColorBufferSize = m_colorFrameBuffer.GetBufferSize();
		assert(pColorBuffer);
		if(pColorBuffer && nColorBufferSize > 0)
		{
			errno_t err = memcpy_s(reinterpret_cast<BYTE*>(pColorBuffer), nColorBufferSize, PBYTE(LockedRect.pBits), imageFrame.pFrameTexture->BufferLen());
			returnStatus = (0 == err);
			if(returnStatus)
			{
				m_colorFrameBuffer.SetTimeStamp(imageFrame.liTimeStamp.QuadPart);
			}
			else
			{
				printf_s("No color frame buffer available.\n");
			}
		}
		else
		{
			printf_s("No color frame buffer available.\n");
		}
	}

	hr = imageFrame.pFrameTexture->UnlockRect(0);
	hr = m_pNuiSensor->NuiImageStreamReleaseFrame(m_pColorStreamHandle, &imageFrame);
	if ( FAILED(hr) )
	{
		printf_s("Failed to release depth frame.\n");
	}

	return returnStatus;
}

bool NuiKinectV1Grabber::grabSkeletonFrame(NuiCompositeFrame* pCompositeFrame)
{
	bool bNeedCache = false;
	if(m_pCache && pCompositeFrame)
	{
		bNeedCache = true;
	}
	if(!bNeedCache)
		return false;

	NUI_SKELETON_FRAME skeletonFrame = {0};
	HRESULT hr = m_pNuiSensor->NuiSkeletonGetNextFrame(0, &skeletonFrame);
	if ( FAILED(hr) )
	{
		printf_s("Failed to grab skeleton frame.\n");
		return false;
	}

	pCompositeFrame->m_skeletonFrame.SetTimeStamp(skeletonFrame.liTimeStamp.QuadPart);
	
	// smooth out the skeleton data
	m_pNuiSensor->NuiTransformSmooth(&skeletonFrame, &m_skeletonSmoothParams);

	for (int i = 0 ; i < NUI_SKELETON_COUNT; ++i)
	{
		NuiSkeletonJoints* pSkeletonJoints = nullptr;
		NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

		if (NUI_SKELETON_TRACKED == trackingState)
		{
			const NUI_SKELETON_DATA & skel = skeletonFrame.SkeletonData[i];

			pSkeletonJoints = new NuiSkeletonJoints;
			for (int jointId = 0; jointId < JOINT_TYPE_COUNT; jointId++)
			{
				bool isInferred = true;
				NUI_SKELETON_POSITION_TRACKING_STATE jointState = skel.eSkeletonPositionTrackingState[jointId];
				if (jointState == NUI_SKELETON_POSITION_TRACKED)
				{
					isInferred = false;
				}
				else if (jointState == NUI_SKELETON_POSITION_INFERRED)
				{
					isInferred = true;
				}
				else
				{
					continue;
				}

				Vector4 jointInSkeletonSpace = skeletonFrame.SkeletonData[i].SkeletonPositions[jointId];
				if(jointId == JOINT_TYPE_HEAD)
					pSkeletonJoints->SetHeadForFaceTracking((float)jointInSkeletonSpace.x, (float)jointInSkeletonSpace.y, (float)jointInSkeletonSpace.z);
				if(jointId == JOINT_TYPE_NECK)
					pSkeletonJoints->SetNeckForFaceTracking((float)jointInSkeletonSpace.x, (float)jointInSkeletonSpace.y, (float)jointInSkeletonSpace.z);
				LONG pos_x, pos_y;
				USHORT pos_depth;
				NuiTransformSkeletonToDepthImage(
					jointInSkeletonSpace,
					&pos_x,
					&pos_y,
					&pos_depth,
					m_depthResolution );
				isInferred |= (pos_depth == NUI_IMAGE_DEPTH_NO_VALUE || (pos_x == 0 && pos_y == 0));
				pSkeletonJoints->SetJoint3DPos((NuiJointType)jointId, (float)pos_x, (float)pos_y, (float)pos_depth, isInferred);
				pSkeletonJoints->SetJoint2DPos((NuiJointType)jointId, (float)pos_x, (float)pos_y, isInferred);
			}
		}
		else if (NUI_SKELETON_POSITION_ONLY == trackingState)
		{
			Vector4 jointInSkeletonSpace = skeletonFrame.SkeletonData[i].Position;
			LONG pos_x, pos_y;
			USHORT pos_depth;
			NuiTransformSkeletonToDepthImage(skeletonFrame.SkeletonData[i].Position, &pos_x, &pos_y, &pos_depth);
			bool isInferred = (pos_depth == NUI_IMAGE_DEPTH_NO_VALUE || (pos_x == 0 && pos_y == 0));

			pSkeletonJoints = new NuiSkeletonJoints;
			pSkeletonJoints->SetJoint3DPos(JOINT_TYPE_HIP_CENTER, (float)pos_x, (float)pos_y, (float)pos_depth, isInferred);
			pSkeletonJoints->SetJoint2DPos(JOINT_TYPE_HIP_CENTER, (float)pos_x, (float)pos_y, isInferred);
		}

		// Cache
		if( !pCompositeFrame->m_skeletonFrame.CacheSkeleton(i, pSkeletonJoints))
			SafeDelete(pSkeletonJoints);
	}

	return true;
}

void NuiKinectV1Grabber::grabThread ()
{
	HANDLE          hEvents[4];

	// Configure events to be listened on
	hEvents[0] = m_hNextDepthFrameEvent;
	hEvents[1] = m_hNextColorFrameEvent;
	hEvents[2] = m_hNextSkeletonFrameEvent;

	while (true)
	{
		if (!m_threadOn)
			break;
		if (!m_pCache)
			continue;
		
		// Wait for an event to be signaled
		WaitForMultipleObjects(sizeof(hEvents)/sizeof(hEvents[0]),hEvents,FALSE,100);

		std::shared_ptr<NuiCompositeFrame> pCompositeFrame = m_pCache->allocateFrame();
		if(!pCompositeFrame)
			continue;

		if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0) )
		{
			if( !grabDepthFrame(pCompositeFrame.get()) )
			{
				printf_s( "Failed to grab the depth frame.\n" );
			}
		}

		if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextColorFrameEvent, 0) )
		{
			if( grabColorFrame() )
			{
				pCompositeFrame->m_colorFrame.WriteFrameLock();
				pCompositeFrame->m_colorFrame = m_colorFrameBuffer;
				pCompositeFrame->m_colorFrame.WriteFrameUnlock();
				// Compute FPS
				m_colorFrameCount ++;
				LONGLONG colorTime = m_colorFrameBuffer.GetTimeStamp();
				LONGLONG span      = colorTime - m_lastColorTime;
				if (span >= 1000)
				{
					m_colorFrameBuffer.SetFPS( (UINT)((double)m_colorFrameCount * 1000.0 / (double)span + 0.5) );
					m_lastColorTime = colorTime;
					m_colorFrameCount = 0;
				}
			}
			else
			{
				printf_s( "Failed to grab the color frame.\n" );
			}
		}

		// Wait for 0ms, just quickly test if it is time to process a skeleton
		if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextSkeletonFrameEvent, 0) )
		{
			if( !grabSkeletonFrame(pCompositeFrame.get()) )
			{
				printf_s( "Failed to grab the skeleton frame.\n" );
			}
		}

		if( !frameToCache(pCompositeFrame.get()) )
		{
			printf_s( "Failed to cache the composite frame.\n" );
		}
		
		//boost::this_thread::sleep (boost::posix_time::seconds (1));
	}
}

bool NuiKinectV1Grabber::frameToCache(NuiCompositeFrame* pCompositeFrame)
{
	assert(pCompositeFrame);
	if(!pCompositeFrame)
		return false;

	assert(m_pMapper);
	if(!m_pMapper)
		return false;

	bool bNeedCachePointCloud = true; //(cacheFlags & NuiDeviceBufferImpl::ECache_PointCloud) ? true : false;
	bool bNeedCacheDepth = true; //(cacheFlags & NuiDeviceBufferImpl::ECache_Depth) ? true : false;
	bool bNeedCacheBodyIndex = true; //(cacheFlags & NuiDeviceBufferImpl::ECache_PlayIndex) ? true : false;

	NUI_DEPTH_IMAGE_PIXEL* pExtendedDepthBuffer = m_depthFrameBuffer.GetBuffer();
	if(!pExtendedDepthBuffer)
		return true;

	/*Vector4 reading;
	if( SUCCEEDED(m_pNuiSensor->NuiAccelerometerGetCurrentReading(&reading)) )
		pCompoundFrame->setAccelerometerReading(reading);*/

	UINT nDepthWidth = m_depthFrameBuffer.GetWidth();
	UINT nDepthHeight = m_depthFrameBuffer.GetHeight();
	UINT nDepthPointNum = nDepthWidth * nDepthHeight;

	UINT nColorWidth = m_colorFrameBuffer.GetWidth();
	UINT nColorHeight = m_colorFrameBuffer.GetHeight();
	UINT nColorPointNum = nColorWidth * nColorHeight;
	BGRQUAD* pColorBuffer = m_colorFrameBuffer.GetBuffer();
	if(pColorBuffer && nColorPointNum > 0 /*&&
		(std::abs(m_depthFrameBuffer.GetTimeStamp() - m_colorFrameBuffer.GetTimeStamp()) < cHalfADepthFrameMs)*/)
	{
		if( !m_pColorToDepth )
			m_pColorToDepth = new NUI_COLOR_IMAGE_POINT[nDepthPointNum];
		// Get the coordinates to convert color to depth space
		HRESULT hr = m_pMapper->MapDepthFrameToColorFrame(
			m_depthResolution,
			nDepthPointNum,
			pExtendedDepthBuffer,
			NUI_IMAGE_TYPE_COLOR,
			m_colorResolution,
			nColorPointNum,   // the color coordinates that get set are the same array size as the depth image
			m_pColorToDepth );
		if(FAILED(hr))
		{
			pColorBuffer = nullptr;
			printf_s("Failed to grab color coordinates.\n");
		}
	}
	else
	{
		pColorBuffer = nullptr;
	}

	UINT minDepth = m_bNearMode ? m_minNearDepth : m_minDepth;
	UINT maxDepth = m_bNearMode ? m_maxNearDepth : m_maxDepth;

	// Initialize the cache buffer
	UINT16* pDepthBuffer = nullptr;
	if(bNeedCacheDepth)
	{
		pDepthBuffer = pCompositeFrame->m_depthFrame.AllocateBuffer(nDepthWidth, nDepthHeight);
		assert(pDepthBuffer);
	}
	pCompositeFrame->m_depthFrame.WriteFrameLock();

	BYTE* pBodyIndexBuffer = nullptr;
	bool bHasPlayerIndex = (m_pNuiSensor->NuiInitializationFlags() & NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX);
	if(bHasPlayerIndex && bNeedCacheBodyIndex)
	{
		pBodyIndexBuffer = pCompositeFrame->m_bodyIndexFrame.AllocateBuffer(nDepthWidth, nDepthHeight);
		assert(pBodyIndexBuffer);
	}
	pCompositeFrame->m_bodyIndexFrame.WriteFrameLock();

	NuiDevicePoint* pPoints = nullptr;
	if(bNeedCachePointCloud)
	{
		pPoints = pCompositeFrame->m_pointCloudFrame.AllocatePoints(nDepthPointNum);
		pCompositeFrame->m_pointCloudFrame.SetWidthStep(nDepthWidth);
		assert(pPoints);
	}
	pCompositeFrame->m_pointCloudFrame.WriteFrameLock();
	
#pragma omp parallel for schedule(dynamic)
	for (UINT y = 0; y < nDepthHeight; ++y)
	{
		for (UINT x = 0; x < nDepthWidth; ++x)
		{
			// calculate index into depth array
			UINT depthIndex = x + y * nDepthWidth;
			NUI_DEPTH_IMAGE_PIXEL* pDepthValue = pExtendedDepthBuffer + depthIndex;
			bool bValidDepth = (pDepthValue->depth >= minDepth && pDepthValue->depth <= maxDepth);

			if(pDepthBuffer)
			{
				if(bValidDepth)
					pDepthBuffer[depthIndex] = pDepthValue->depth;
				else
					pDepthBuffer[depthIndex] = 0;
			}
			if(pBodyIndexBuffer)
			{
				if(bValidDepth)
					pBodyIndexBuffer[depthIndex] = (BYTE)pDepthValue->playerIndex;
				else
					pBodyIndexBuffer[depthIndex] = 0xff;
			}
			if(pPoints)
			{
				if(bValidDepth)
				{
					pPoints[depthIndex].fVertex.x = (float)x / 100.0f;
					pPoints[depthIndex].fVertex.y = (float)y / 100.0f;
					pPoints[depthIndex].fVertex.z = (float)pDepthValue->depth / 1000.0f;
					if(pColorBuffer && m_pColorToDepth)
					{
						// retrieve the depth to color mapping for the current depth pixel
						UINT colorInDepthX = m_pColorToDepth[depthIndex].x;
						UINT colorInDepthY = m_pColorToDepth[depthIndex].y;

						UINT colorIndex = depthIndex;
						// make sure the depth pixel maps to a valid point in color space
						if ( colorInDepthX >= 0 && colorInDepthX < nColorWidth && colorInDepthY >= 0 && colorInDepthY < nColorHeight )
						{
							// calculate index into color array
							colorIndex = colorInDepthX + colorInDepthY * nColorWidth;
						}
						BGRQUAD* pColorValue = pColorBuffer + colorIndex;
						pPoints[depthIndex].fBGRA = *pColorValue;
						pPoints[depthIndex].fColorSpaceU = (float)colorInDepthX / nColorWidth;
						pPoints[depthIndex].fColorSpaceV = (float)colorInDepthY / nColorHeight;
					}
					else
					{
						pPoints[depthIndex].fBGRA.rgbBlue = 0;
						pPoints[depthIndex].fBGRA.rgbGreen = 0;
						pPoints[depthIndex].fBGRA.rgbRed = 0;
						pPoints[depthIndex].fBGRA.rgbReserved = 0;
						pPoints[depthIndex].fColorSpaceU = 0.0f;
						pPoints[depthIndex].fColorSpaceV = 0.0f;
					}
				}
				else
				{
					pPoints[depthIndex].fVertex.x = 0.0f;
					pPoints[depthIndex].fVertex.y = 0.0f;
					pPoints[depthIndex].fVertex.z = 0.0f;
					pPoints[depthIndex].fBGRA.rgbBlue = 0;
					pPoints[depthIndex].fBGRA.rgbGreen = 0;
					pPoints[depthIndex].fBGRA.rgbRed = 0;
					pPoints[depthIndex].fBGRA.rgbReserved = 0;
					pPoints[depthIndex].fColorSpaceU = 0.0f;
					pPoints[depthIndex].fColorSpaceV = 0.0f;
				}
			}
		}
	}

	pCompositeFrame->m_pointCloudFrame.WriteFrameUnlock();
	pCompositeFrame->m_bodyIndexFrame.WriteFrameUnlock();
	pCompositeFrame->m_depthFrame.WriteFrameUnlock();

	INT64 timeStamp = m_depthFrameBuffer.GetTimeStamp();
	if(bNeedCacheDepth)
	{
		pCompositeFrame->m_depthFrame.SetTimeStamp(timeStamp);
		// Compute FPS
		m_depthFrameCount ++;
		LONGLONG span      = timeStamp - m_lastDepthTime;
		if (span >= 1000)
		{
			pCompositeFrame->m_depthFrame.SetFPS( (UINT)((double)m_depthFrameCount * 1000.0 / (double)span + 0.5) );
			m_lastDepthTime = timeStamp;
			m_depthFrameCount = 0;
		}
	}
	if(bNeedCacheBodyIndex)
		pCompositeFrame->m_bodyIndexFrame.SetTimeStamp(timeStamp);
	if(bNeedCachePointCloud)
		pCompositeFrame->m_pointCloudFrame.SetTimeStamp(timeStamp);

	return true;
}

void NuiKinectV1Grabber::startThread ()
{
	if(m_pNuiSensor && !m_threadOn)
	{
		m_threadOn = true;
		m_Thread.reset (new boost::thread (boost::bind (&NuiKinectV1Grabber::grabThread, this)));
	}
}

void NuiKinectV1Grabber::pauseThread()
{
	m_threadOn = false;
	m_Thread->join();
}

void NuiKinectV1Grabber::stopThread ()
{
	// Reset all the event to nonsignaled state
	ResetEvent(m_hNextDepthFrameEvent);
	ResetEvent(m_hNextColorFrameEvent);
	ResetEvent(m_hNextSkeletonFrameEvent);

	m_threadOn = false;
	//m_threadPause = false;
	if(m_Thread)
	{
		//m_Thread->interrupt();
		m_Thread->join();
	}

	SafeRelease(m_pMapper);

	m_depthFrameBuffer.Clear();
	m_colorFrameBuffer.Clear();
	SafeDeleteArray(m_pColorToDepth);
}

void NuiKinectV1Grabber::updateSkeletonMode()
{
	if ( m_pNuiSensor &&
		INVALID_HANDLE_VALUE != m_hNextSkeletonFrameEvent &&
		(m_pNuiSensor->NuiInitializationFlags() & NUI_INITIALIZE_FLAG_USES_SKELETON) )
	{
		DWORD dwSkeletonFlags = m_bNearMode ? NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE : 0;
		if (m_bSkeletonSeatedMode)
		{
			dwSkeletonFlags |= NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT;
		}
		m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonFrameEvent, dwSkeletonFlags);
	}
}

void NuiKinectV1Grabber::updateNearMode()
{
	if ( m_pNuiSensor && INVALID_HANDLE_VALUE != m_pDepthStreamHandle)
	{
		m_pNuiSensor->NuiImageStreamSetImageFrameFlags(m_pDepthStreamHandle, m_bNearMode ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0);
	}
	updateSkeletonMode();
}

void NuiKinectV1Grabber::setNearMode(bool bNearMode)
{
	m_bNearMode = bNearMode;
	updateNearMode();
}

bool NuiKinectV1Grabber::getNearMode() const
{
	bool bNearMode = false;
	if ( m_pNuiSensor && INVALID_HANDLE_VALUE != m_pDepthStreamHandle)
	{
		DWORD imageFlag;
		m_pNuiSensor->NuiImageStreamGetImageFrameFlags(m_pDepthStreamHandle, &imageFlag);
		bNearMode = (imageFlag & NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE) ? true : false;
	}
	return bNearMode;
}

void NuiKinectV1Grabber::setSkeletonSeatedMode(bool bSeatedMode)
{
	m_bSkeletonSeatedMode = bSeatedMode;
	updateSkeletonMode();
}

void NuiKinectV1Grabber::updateSkeletonSmoothParams(float smoothing, float correction, float prediction, float jitterRadius, float maxDeviationRadius)
{
	//fSmoothing：平滑值(Smoothing)属性，设置处理骨骼数据帧时的平滑量，接受一个0-1的浮点值，值越大，平滑的越多。0表示不进行平滑
	//fCorrection：修正值(Correction)属性，接受一个从0-1的浮点型。值越小，修正越多。
	//fJitterRadius：抖动半径(JitterRadius)属性，设置修正的半径，如果关节点“抖动”超过了设置的这个半径，将会被纠正到这个半径之内。该属性为浮点型，单位为米。
	//fMaxDeviationRadius：最大偏离半径(MaxDeviationRadius)属性，用来和抖动半径一起来设置抖动半径的最大边界。任何超过这一半径的点都不会认为是抖动产生的，而被认定为是一个新的点。该属性为浮点型，单位为米。
	//fPrediction：预测帧大小(Prediction)属性，返回用来进行平滑需要的骨骼帧的数目。
	//对骨骼关节点进行平滑处理会产生性能开销。平滑处理的越多，性能消耗越大。设置平滑参数没有经验可以遵循。需要不断的测试和调试已达到最好的性能和效果。在程序运行的不同阶段，可能需要设置不同的平滑参数。

	// Some smoothing with little latency (defaults).  
	// Only filters out small jitters.  
	// Good for gesture recognition.  
	//const NUI_TRANSFORM_SMOOTH_PARAMETERS DefaultParams =   
	//{0.5f, 0.5f, 0.5f, 0.05f, 0.04f};  

	// Smoothed with some latency.  
	// Filters out medium jitters.  
	// Good for a menu system that needs to be smooth but  
	// doesn't need the reduced latency as much as gesture recognition does.  
	//const NUI_TRANSFORM_SMOOTH_PARAMETERS SomewhatLatentParams =   
	//{0.5f, 0.1f, 0.5f, 0.1f, 0.1f};  

	// Very smooth, but with a lot of latency.  
	// Filters out large jitters.  
	// Good for situations where smooth data is absolutely required  
	// and latency is not an issue.  
	//const NUI_TRANSFORM_SMOOTH_PARAMETERS VerySmoothParams =   
	//{0.7f, 0.3f, 1.0f, 1.0f, 1.0f};
	m_skeletonSmoothParams.fSmoothing = smoothing;
	m_skeletonSmoothParams.fCorrection = correction;
	m_skeletonSmoothParams.fPrediction = prediction;
	m_skeletonSmoothParams.fJitterRadius = jitterRadius;
	m_skeletonSmoothParams.fMaxDeviationRadius = maxDeviationRadius;
}

void NuiKinectV1Grabber::updateDepthRange(UINT minDepth, UINT maxDepth)
{
	if(minDepth > NUI_IMAGE_DEPTH_MINIMUM >> NUI_IMAGE_PLAYER_INDEX_SHIFT)
		m_minDepth = minDepth;
	else
		m_minDepth = NUI_IMAGE_DEPTH_MINIMUM >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

	if(maxDepth < NUI_IMAGE_DEPTH_MAXIMUM >> NUI_IMAGE_PLAYER_INDEX_SHIFT)
		m_maxDepth = maxDepth;
	else
		m_maxDepth = NUI_IMAGE_DEPTH_MAXIMUM >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
}

void NuiKinectV1Grabber::updateNearDepthRange(UINT minDepth, UINT maxDepth)
{
	if(minDepth > NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE >> NUI_IMAGE_PLAYER_INDEX_SHIFT)
		m_minNearDepth = minDepth;
	else
		m_minNearDepth = NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

	if(maxDepth < NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE >> NUI_IMAGE_PLAYER_INDEX_SHIFT)
		m_maxNearDepth = maxDepth;
	else
		m_maxNearDepth = NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
}
