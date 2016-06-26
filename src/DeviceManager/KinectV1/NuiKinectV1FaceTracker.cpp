// File: PCEKinectFaceTracker.cpp
//
// Author: Tingzhu Zhou
//
#include "NuiKinectV1FaceTracker.h"

#include "../NuiRGBDDeviceBufferImpl.h"
#include "Frame/NuiCompositeFrame.h"

#include "Shape/NuiTrackedFace.h"
#include "Shape/NuiFacialModel.h"

NuiKinectV1FaceTracker::NuiKinectV1FaceTracker()
	: m_pFaceTracker(NULL)
	, m_pFTResult(NULL)
	, m_colorImage(NULL)
	, m_depthImage(NULL)
	, m_LastTrackSucceeded(false)
	, m_threadOn(false)
	, m_threadPause(false)
	, m_pCache(NULL)
{
	m_hint3D[0].x = m_hint3D[0].y = m_hint3D[0].z = m_hint3D[1].x = m_hint3D[1].y = m_hint3D[1].z = 0.0f;
}

NuiKinectV1FaceTracker::~NuiKinectV1FaceTracker()
{
	stopThread ();
	m_pFaceTracker->Release();
	m_pFaceTracker = NULL;
}

bool NuiKinectV1FaceTracker::setupTracker(NUI_IMAGE_RESOLUTION depthResolution, NUI_IMAGE_RESOLUTION	colorResolution)
{
	m_threadOn = false;

	// Try to start the face tracker.
	if(!m_pFaceTracker)
		m_pFaceTracker = FTCreateFaceTracker(NULL);
	if (!m_pFaceTracker)
	{
		printf_s( "Failed to create face tracker.\n" );
		return false;
	}

	// Color config
	if(NUI_IMAGE_RESOLUTION_640x480 == colorResolution)
	{
		m_videoConfig.FocalLength = NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS;
	}
	else if(NUI_IMAGE_RESOLUTION_1280x960 == colorResolution)
	{
		m_videoConfig.FocalLength = NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 2.f;
	}
	else
		return false;

	DWORD colorWidth = 0;
	DWORD colorHeight = 0;
	NuiImageResolutionToSize(colorResolution, colorWidth, colorHeight);
	m_videoConfig.Width = colorWidth;
	m_videoConfig.Height = colorHeight;

	if (!m_colorImage)
		m_colorImage = FTCreateImage();
	if (!m_colorImage || FAILED(m_colorImage->Allocate(m_videoConfig.Width, m_videoConfig.Height, FTIMAGEFORMAT_UINT8_B8G8R8X8)))
	{
		printf_s( "Failed to create face tracker image.\n" );
		return false;
	}

	// Depth config
	FT_CAMERA_CONFIG depthConfig;
	if(NUI_IMAGE_RESOLUTION_640x480 == depthResolution)
	{
		depthConfig.FocalLength = NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 2.f;
	}
	else if(NUI_IMAGE_RESOLUTION_320x240 == depthResolution)
	{
		depthConfig.FocalLength = NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS;
	}
	else
	{
		depthConfig.FocalLength = 0.f;
	}

	if(depthConfig.FocalLength != 0.f)
	{
		DWORD depthWidth = 0;
		DWORD depthHeight = 0;
		NuiImageResolutionToSize(depthResolution, depthWidth, depthHeight);
		depthConfig.Width = depthWidth;
		depthConfig.Height = depthHeight;

		if (!m_depthImage)
			m_depthImage = FTCreateImage();
		if (m_depthImage)
		{
			m_depthImage->Allocate(depthConfig.Width, depthConfig.Height, FTIMAGEFORMAT_UINT16_D13P3);
		}
	}
	else
	{
		depthConfig.Width = 0;
		depthConfig.Height = 0;
	}

	HRESULT hr = m_pFaceTracker->Initialize(&m_videoConfig, &depthConfig, NULL, NULL);
	if (FAILED(hr))
	{
		printf_s( "Failed to initialize face tracker.\n" );
		return false;
	}

	hr = m_pFaceTracker->CreateFTResult(&m_pFTResult);
	if (FAILED(hr) || !m_pFTResult)
	{
		printf_s( "Failed to create face tracker result.\n" );
		return false;
	}

	m_hint3D[0].x = m_hint3D[0].y = m_hint3D[0].z = m_hint3D[1].x = m_hint3D[1].y = m_hint3D[1].z = 0.0f;
	m_LastTrackSucceeded = false;

	return true;
}

void NuiKinectV1FaceTracker::getFaceShape(float zoomFactor, POINT* viewOffSet)
{
	if (!m_pCache || !m_pFaceTracker || !m_pFTResult || FAILED(m_pFTResult->GetStatus()))
		return;

	FLOAT headScale;
	FLOAT* pSU = NULL;
	UINT numSU;
	BOOL suConverged;
	HRESULT hr = m_pFaceTracker->GetShapeUnits(&headScale, &pSU, &numSU, &suConverged);
	if(FAILED(hr))
	{
		printf_s( "Failed to get face shape units.\n" );
		return;
	}
	FLOAT *pAUs;
	UINT auCount;
	hr = m_pFTResult->GetAUCoefficients(&pAUs, &auCount);
	if(FAILED(hr))
	{
		printf_s( "Failed to get face shape AUs.\n" );
		return;
	}
	FLOAT scale, rotationXYZ[3], translationXYZ[3];
	hr = m_pFTResult->Get3DPose(&scale, rotationXYZ, translationXYZ);
	if(FAILED(hr))
	{
		printf_s( "Failed to get face tracking pose.\n" );
		return;
	}
	IFTModel* ftModel;
	hr = m_pFaceTracker->GetFaceModel(&ftModel);
	if(FAILED(hr) || !ftModel)
	{
		printf_s( "Failed to get face tracking model.\n" );
		return;
	}

	UINT vertexCount = ftModel->GetVertexCount();
	FT_VECTOR2D* pPts2D = reinterpret_cast<FT_VECTOR2D*>(_malloca(sizeof(FT_VECTOR2D) * vertexCount));
	if(pPts2D)
	{
		hr = ftModel->GetProjectedShape(
			&m_videoConfig,
			zoomFactor,
			*viewOffSet,
			pSU, numSU, pAUs, auCount, scale, rotationXYZ, translationXYZ, pPts2D, vertexCount);
		if(SUCCEEDED(hr) && m_depthImage)
		{
			USHORT* pDepthBuffer = reinterpret_cast<USHORT*>(m_depthImage->GetBuffer());
			if(pDepthBuffer)
			{
				// Create a face tracking result to be put into the buffer.
				//NuiTrackedFace* pShape = new NuiTrackedFace();
				//assert(pShape);
				//pShape->writeAUs(pAUs, auCount);
				//pShape->setScale(scale);
				//pShape->setRotationXYZ(rotationXYZ);
				//pShape->setTranslationXYZ(translationXYZ);

				//// Set face vetices
				//bool* pInvalidPts = reinterpret_cast<bool*>(_malloca(sizeof(bool) * vertexCount));
				//for (UINT i = 0; i < vertexCount; i++)
				//{
				//	Vector3 pts3D;
				//	pts3D.x = pPts2D[i].x;
				//	pts3D.y = pPts2D[i].y;

				//	LONG x = LONG(pPts2D[i].x + 0.5f);
				//	LONG y = LONG(pPts2D[i].y + 0.5f);
				//	LONG ptIndex = x + y * m_depthImage->GetWidth();
				//	USHORT depthPixel = pDepthBuffer[ptIndex];
				//	USHORT depth = depthPixel;
				//	if(depth == NUI_IMAGE_DEPTH_NO_VALUE)
				//		pInvalidPts[i] = true;
				//	else
				//		pInvalidPts[i] = false;

				//	pts3D.z = (float)depth * PCECompoundFrame::cDepthToFloatCoefficient;
				//	pShape->pushbackPts3D(pts3D);
				//}
				//// Set face triangles
				//FT_TRIANGLE* pTriangles;
				//UINT triangleCount;
				//hr = ftModel->GetTriangles(&pTriangles, &triangleCount);
				//if(FAILED(hr))
				//{
				//	printf_s( "Failed to get face triangles.\n" );
				//}
				//for (UINT trId = 0; trId < triangleCount; trId++)
				//{
				//	if(!pInvalidPts[pTriangles[trId].i] &&
				//		!pInvalidPts[pTriangles[trId].j] &&
				//		!pInvalidPts[pTriangles[trId].k])
				//	{
				//		Triangle tri;
				//		tri.i = pTriangles[trId].i;
				//		tri.j = pTriangles[trId].j;
				//		tri.k = pTriangles[trId].k;
				//		pShape->pushbackTriangle(tri);
				//	}
				//}
				//_freea(pInvalidPts);

				//// Set Face Center
				//RECT faceRect;
				//m_pFTResult->GetFaceRect(&faceRect);
				//LONG centerPtX = (faceRect.left+faceRect.right)/2.0;
				//LONG centerPtY = (faceRect.top+faceRect.bottom)/2.0;
				//LONG centerIndex = centerPtX + centerPtY * m_depthImage->GetWidth();
				//Vector3		centerPt;
				//centerPt.x = (float)centerPtX;
				//centerPt.y = (float)centerPtY;
				//USHORT depthPixel = pDepthBuffer[centerIndex];
				//USHORT depth = depthPixel;
				//centerPt.z = (float)depth * PCECompoundFrame::cDepthToFloatCoefficient;
				//pShape->setCenterPt(centerPt);

				//if(!m_pCache->setFaceShape(pShape))
				//	delete pShape;
			}
		}
		else
		{
			printf_s( "Failed to get face projected shape.\n" );
		}
		 _freea(pPts2D);
	}

	ftModel->Release();
}

bool NuiKinectV1FaceTracker::readFrame(NuiCompositeFrame* pCompositeFrame)
{
	assert(pCompositeFrame);
	if(!pCompositeFrame)
		return false;
	
	bool readStatus = false;
	pCompositeFrame->m_compressedDepthBuffer.WriteFrameLock();
	USHORT* pCachedDepthBuffer = pCompositeFrame->m_compressedDepthBuffer.GetBuffer();
	BYTE* pDepthBuffer = m_depthImage ? m_depthImage->GetBuffer() : NULL;
	if(pCachedDepthBuffer && pDepthBuffer)
	{
		errno_t err = memcpy_s(pDepthBuffer, m_depthImage->GetBufferSize(), reinterpret_cast<BYTE*>(pCachedDepthBuffer), pCompositeFrame->m_compressedDepthBuffer.GetBufferSize());
		readStatus = (0 == err);
	}
	pCompositeFrame->m_compressedDepthBuffer.WriteFrameUnlock();

	pCompositeFrame->m_colorFrame.WriteFrameLock();
	BYTE* pColorBuffer = m_colorImage ? m_colorImage->GetBuffer() : NULL;
	BGRQUAD* pCachedColorBuffer = pCompositeFrame->m_colorFrame.GetBuffer();
	if(pCachedColorBuffer && pColorBuffer)
	{
		errno_t err = memcpy_s(pColorBuffer, m_colorImage->GetBufferSize(), reinterpret_cast<BYTE*>(pCachedColorBuffer), pCompositeFrame->m_colorFrame.GetBufferSize());
		readStatus = (0 == err);
	}
	pCompositeFrame->m_colorFrame.WriteFrameUnlock();

	return readStatus;
}

bool NuiKinectV1FaceTracker::getClosestFaceHint(NuiCompositeFrame* pCompositeFrame)
{
	int selectedSkeleton = -1;
	float smallestDistance = 0;
	NuiSkeletonJoints skeletonPos;
	const NuiSkeletonFrame& skeletonFrame = pCompositeFrame->m_skeletonFrame;

	if (m_hint3D[1].x == 0 && m_hint3D[1].y == 0 && m_hint3D[1].z == 0)
	{
		// Get the skeleton closest to the camera
		for (UINT iBody = 0 ; iBody < skeletonFrame.GetBodyCount() ; iBody++ )
		{
			if( !skeletonFrame.ReadSkeleton(iBody, &skeletonPos) )
				continue;
			if( skeletonPos.IsHeadInferred() || skeletonPos.IsNeckInferred())
				continue;

			float headZ = skeletonPos.GetHeadPosZ();
			if (smallestDistance == 0 || headZ < smallestDistance)
			{
				smallestDistance = headZ;
				selectedSkeleton = iBody;
			}
		}
	}
	else
	{   // Get the skeleton closest to the previous position
		for (UINT iBody = 0 ; iBody < skeletonFrame.GetBodyCount() ; iBody++ )
		{
			if( !skeletonFrame.ReadSkeleton(iBody, &skeletonPos) )
				continue;
			if( skeletonPos.IsHeadInferred() || skeletonPos.IsNeckInferred())
				continue;

			float headX = skeletonPos.GetHeadPosX();
			float headY = skeletonPos.GetHeadPosY();
			float headZ = skeletonPos.GetHeadPosZ();
			float d = abs(headX - m_hint3D[1].x) +
				abs(headY - m_hint3D[1].y) +
				abs(headZ - m_hint3D[1].z);
			if (smallestDistance == 0 || d < smallestDistance)
			{
				smallestDistance = d;
				selectedSkeleton = iBody;
			}
		}
	}
	if (selectedSkeleton != -1)
	{
		skeletonFrame.ReadSkeleton(selectedSkeleton, &skeletonPos);
		m_hint3D[0].x = skeletonPos.GetNeckPosX();
		m_hint3D[0].y = skeletonPos.GetNeckPosY();
		m_hint3D[0].z = skeletonPos.GetNeckPosZ();
		m_hint3D[1].x = skeletonPos.GetHeadPosX();
		m_hint3D[1].y = skeletonPos.GetHeadPosY();
		m_hint3D[1].z = skeletonPos.GetHeadPosZ();
		return true;
	}

	return false;
}

void NuiKinectV1FaceTracker::runThread ()
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
			printf_s( "Failed to acquire the latest composite frame in face tracker.\n" );
			continue;
		}
		if( !readFrame(pCompositeFrame.get()) )
		{
			printf_s( "Failed to read cached images in face tracker.\n" );
			continue;
		}
		
		float zoomFactor = pCompositeFrame->m_compressedDepthBuffer.GetZoomFactor();
		POINT viewOffset;
		Point2d* pViewOffset = pCompositeFrame->m_compressedDepthBuffer.GetViewOffSet();
		if(pViewOffset)
		{
			viewOffset.x = pViewOffset->x;
			viewOffset.y = pViewOffset->y;
		}
		else
		{
			viewOffset.x = 0;
			viewOffset.y = 0;
		}
		FT_SENSOR_DATA sensorData(m_colorImage, m_depthImage, zoomFactor, &viewOffset);

		if ( !getClosestFaceHint(pCompositeFrame.get()) )
		{
			/*USHORT* pDepthBuffer = reinterpret_cast<USHORT*>(m_depthImage->GetBuffer());
			if(pDepthBuffer)
			{
				NUI_IMAGE_RESOLUTION res = NUI_IMAGE_RESOLUTION_320x240;
				if(m_depthImage->GetWidth() == 640 && m_depthImage->GetHeight() == 480)
					res = NUI_IMAGE_RESOLUTION_640x480;
				else if(m_depthImage->GetWidth() == 1280 && m_depthImage->GetHeight() == 960)
					res = NUI_IMAGE_RESOLUTION_1280x960;

				LONG halfX = m_depthImage->GetWidth() / 2;
				LONG halfY = m_depthImage->GetHeight() / 2 - 100;
				LONG inferredIndex = halfX + halfY * m_depthImage->GetWidth();
				USHORT depthPixel = pDepthBuffer[inferredIndex];
				USHORT depth = depthPixel;
				if(m_depthImage->GetFormat() == FTIMAGEFORMAT_UINT16_D13P3)
					depth = NuiDepthPixelToDepth(depthPixel);
				Vector4 halfPt = NuiTransformDepthImageToSkeleton(halfX, halfY, depth, res);

				hint[0].x = halfPt.x;
				hint[0].y = halfPt.y;
				hint[0].z = halfPt.z;

				halfX = m_depthImage->GetWidth() / 2;
				halfY = m_depthImage->GetHeight() / 2;
				inferredIndex = halfX + halfY * m_depthImage->GetWidth();
				depthPixel = pDepthBuffer[inferredIndex];
				depth = depthPixel;
				if(m_depthImage->GetFormat() == FTIMAGEFORMAT_UINT16_D13P3)
					depth = NuiDepthPixelToDepth(depthPixel);
				
				hint[1].x = halfPt.x;
				hint[1].y = halfPt.y;
				hint[1].z = halfPt.z;
			}*/
		}

		HRESULT hrFT = E_FAIL;
		if (m_LastTrackSucceeded)
		{
			hrFT = m_pFaceTracker->ContinueTracking(&sensorData, m_hint3D, m_pFTResult);
		}
		else
		{
			hrFT = m_pFaceTracker->StartTracking(&sensorData, NULL, m_hint3D, m_pFTResult);
		}

		m_LastTrackSucceeded = SUCCEEDED(hrFT) && SUCCEEDED(m_pFTResult->GetStatus());
		if (m_LastTrackSucceeded)
		{
			getFaceShape(zoomFactor, &viewOffset);
		}
		else
		{
			m_pFTResult->Reset();
		}
		//boost::this_thread::sleep (boost::posix_time::microseconds (16));
		//Sleep(16);
	}
}

void NuiKinectV1FaceTracker::startThread ()
{
	if(m_pFaceTracker && !m_threadOn)
	{
		m_hint3D[0].x = m_hint3D[0].y = m_hint3D[0].z = m_hint3D[1].x = m_hint3D[1].y = m_hint3D[1].z = 0.0f;

		m_Thread.reset (new boost::thread (boost::bind (&NuiKinectV1FaceTracker::runThread, this)));
		m_threadOn = true;
	}
	m_threadPause = false;
}

void NuiKinectV1FaceTracker::pauseThread()
{
	m_threadPause = true;
	m_LastTrackSucceeded = false;
}

void NuiKinectV1FaceTracker::stopThread ()
{
	m_threadOn = false;
	m_threadPause = false;
	if(m_Thread)
	{
		//m_Thread->interrupt();
		m_Thread->join();
	}

	SafeRelease(m_pFTResult);
	SafeRelease(m_colorImage);
	SafeRelease(m_depthImage);
	
	m_LastTrackSucceeded = false;
}

void NuiKinectV1FaceTracker::updateCache(NuiRGBDDeviceBufferImpl* pCache)
{
	m_pCache = pCache;
}
