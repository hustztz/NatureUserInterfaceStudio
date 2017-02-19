#include "NuiCvVis.h"

#include "Frame/Buffer/NuiFrameCacheImpl.h"
#include "Frame/NuiCompositeFrame.h"


// color array
const CvScalar cFPSColor = CV_RGB(255,255,255);
const CvScalar cFaceColor = CV_RGB(255,255,255);
const CvScalar cGestureColor = CV_RGB(255,255,255);
const CvScalar cDepthColor = CV_RGB(255,255,255);
const CvScalar cBodyIndexColorTable[7] = {
	CV_RGB(255,0,0),
	CV_RGB(0,255,0),
	CV_RGB(0,0,255),
	CV_RGB(255,255,0),
	CV_RGB(255,0,255),
	CV_RGB(0,255,255),
	CV_RGB(0,0,0),
};

NuiCvVis::NuiCvVis(NuiFrameCacheImpl* pBuffer)
	: m_pBuffer(pBuffer)
	, m_pImg(NULL)
{
	cv::setUseOptimized( true );

	cv::namedWindow( "Previewer" );
}

NuiCvVis::~NuiCvVis()
{
	cv::destroyAllWindows();
	if(m_pImg)
		cvReleaseImage(&m_pImg);
}

void NuiCvVis::DrawDepthImage(NuiCompositeFrame* pCompositeFrame, UINT16 minDepth, UINT16 maxDepth, IplImage* pDepthImg)
{
	assert(pCompositeFrame);

	const NuiDepthFrame& depthFrame = pCompositeFrame->m_depthFrame;
	size_t nDepthBufferSize = depthFrame.GetBufferSize();
	UINT16* pDepthBuffer = depthFrame.GetBuffer();
	if(!pDepthBuffer || nDepthBufferSize == 0)
		return;

	UINT nDepthWidth = depthFrame.GetWidth();
	UINT nDepthHeight = depthFrame.GetHeight();
	if(!pDepthImg)
	{
		pDepthImg = cvCreateImage(cvSize(nDepthWidth,nDepthHeight),IPL_DEPTH_8U ,3);
	}
	assert(pDepthImg);

	const NuiBodyIndexFrame& bodyIndexFrame = pCompositeFrame->m_bodyIndexFrame;
	BYTE* pBodyIndexBuffer = bodyIndexFrame.GetBuffer();
	UINT nBodyIndexWidth = bodyIndexFrame.GetWidth();
	UINT nBodyIndexHeight = bodyIndexFrame.GetHeight();
	if(pBodyIndexBuffer)
		assert((nBodyIndexWidth == nDepthWidth) && (nBodyIndexHeight == nDepthHeight));
	
	pCompositeFrame->m_depthFrame.ReadFrameLock();
	pCompositeFrame->m_bodyIndexFrame.ReadFrameLock();
	for (UINT y = 0; y < nDepthHeight; y ++)
	{
		for (UINT x = 0; x < nDepthWidth; x ++)
		{
			UINT index = x + y * nDepthWidth;
			UINT16* pDepthValue = pDepthBuffer + index;
			double depthScale = (double)(*pDepthValue - minDepth) / (maxDepth - minDepth);
			CvScalar color = cDepthColor;
			if(pBodyIndexBuffer)
			{
				BYTE* pBodyIndexValue = pBodyIndexBuffer + index;
				if(*pBodyIndexValue < 6)
				{
					color = cBodyIndexColorTable[*pBodyIndexValue];
				}
			}
			CvScalar scaledColor = CvScalar(color.val[0] * depthScale, color.val[1] * depthScale, color.val[2] * depthScale);
			cvSet2D(pDepthImg, y, x, scaledColor);
		}
	}
	pCompositeFrame->m_depthFrame.ReadFrameUnlock();
	pCompositeFrame->m_bodyIndexFrame.ReadFrameUnlock();

	if(pDepthImg)
		cvShowImage( "DepthPreviewer", pDepthImg );
}

void NuiCvVis::DrawColorImage(NuiCompositeFrame* pCompositeFrame, IplImage* pColorImg)
{
	assert(pCompositeFrame);

	const NuiColorFrame& colorFrame = pCompositeFrame->m_colorFrame;
	size_t nColorBufferSize = colorFrame.GetBufferSize();
	UINT nColorWidth = colorFrame.GetWidth();
	UINT nColorHeight = colorFrame.GetHeight();
	BGRQUAD* pColorBuffer = colorFrame.GetBuffer();
	if(pColorBuffer && nColorBufferSize > 0)
	{
		if(!pColorImg)
		{
			pColorImg = cvCreateImage(cvSize(nColorWidth,nColorHeight),IPL_DEPTH_8U,4);
		}
		assert(pColorImg);
		pCompositeFrame->m_colorFrame.ReadFrameLock();
		errno_t err = memcpy_s(
			pColorImg->imageData, pColorImg->imageSize,
			pColorBuffer, nColorBufferSize);
		pCompositeFrame->m_colorFrame.ReadFrameUnlock();
	}

	if(!pColorImg)
		return;

	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 2, 8);
	char fps[20];
	sprintf_s(fps, 20, "%g", (float)colorFrame.GetFPS());
	/*char text[30] = "FPS:";
	strcat_s("FPS:", 30, fps);*/
	cvPutText(pColorImg, fps, cvPoint(nColorWidth*4/5, nColorHeight*4/5), &font, cFPSColor);

	const NuiSkeletonFrame& skeletonFrame = pCompositeFrame->m_skeletonFrame;
	const NuiFaceTrackingFrame& faceTrackingFrame = pCompositeFrame->m_faceTrackingFrame;
	const NuiGestureFrame& gestureFrame = pCompositeFrame->m_gestureFrame;
	for (UINT iBody = 0; iBody < skeletonFrame.GetBodyCount(); ++iBody)
	{
		// Skeletons On Color Image
		NuiSkeletonJoints skeleton;
		if( skeletonFrame.ReadSkeleton(iBody, &skeleton) )
		{
			// Gestures
			NuiGestureResult gesture;
			if( gestureFrame.ReadGestureResult(iBody, &gesture) )
			{
				NuiDiscreteGestureResult gestureDiscreteResult = gesture.GetDiscreteGestureResult(Gesture_Discrete_HandUp);
				if(gestureDiscreteResult.bDetected && gestureDiscreteResult.fConfidence > 0.0f)
				{
					CvPoint pt2D;
					if( !skeleton.IsInferred(JOINT_TYPE_HAND_LEFT) )
					{
						pt2D.x = (int)(skeleton.GetJoint2DPosX(JOINT_TYPE_HAND_LEFT));
						pt2D.y = (int)(skeleton.GetJoint2DPosY(JOINT_TYPE_HAND_LEFT));
					}
					if( !skeleton.IsInferred(JOINT_TYPE_HAND_RIGHT) )
					{
						int handY = (int)(skeleton.GetJoint2DPosY(JOINT_TYPE_HAND_RIGHT));
						if(pt2D.y > handY)
						{
							pt2D.x = (int)(skeleton.GetJoint2DPosX(JOINT_TYPE_HAND_RIGHT));
							pt2D.y = handY;
						}
					}
					cvPutText(pColorImg, "handUp", pt2D, &font, cGestureColor);

					NuiContinuousGestureResult gestureContinuousResult = gesture.GetContinuousGestureResult(Gesture_Continuous_Swipe);
					if(gestureContinuousResult.fProgress > 0.5f)
					{
						pt2D.x += 20;
						pt2D.y -= 20;
						cvPutText(pColorImg, "Swipe", pt2D, &font, cGestureColor);
					}
				}

				gestureDiscreteResult = gesture.GetDiscreteGestureResult(Gesture_Discrete_OpenHand);
				if(gestureDiscreteResult.bDetected && gestureDiscreteResult.fConfidence > 0.0f)
				{
					if( !skeleton.IsInferred(JOINT_TYPE_HIP_CENTER) )
					{
						CvPoint pt2D;
						pt2D.x = (int)(skeleton.GetJoint2DPosX(JOINT_TYPE_HIP_CENTER));
						pt2D.y = (int)(skeleton.GetJoint2DPosY(JOINT_TYPE_HIP_CENTER));

						cvPutText(pColorImg, "openHand", pt2D, &font, cGestureColor);
					}
				}
			}
			// Draw joints
			for(int iJoint = 0; iJoint < JOINT_TYPE_COUNT; ++iJoint)
			{
				if( !skeleton.IsInferred((NuiJointType)iJoint) )
				{
					CvPoint pt2D;
					pt2D.x = (int)(skeleton.GetJoint2DPosX((NuiJointType)iJoint));
					pt2D.y = (int)(skeleton.GetJoint2DPosY((NuiJointType)iJoint));
					cvCircle(pColorImg, pt2D, 5, cBodyIndexColorTable[iBody]);
				}
			}
			// Draw Bones
		}
		// Faces On Color Image
		NuiTrackedFace face;
		if( faceTrackingFrame.ReadTrackedFace(iBody, &face) )
		{
			PointF facePoints[FacePointType::FacePointType_Count];
			for (UINT iType = FacePointType::FacePointType_EyeLeft; iType < FacePointType::FacePointType_Count; ++iType)
			{
				face.GetFace2DPoint(iType, &(facePoints[iType]));
				CvPoint pt2D;
				pt2D.x = (int)(facePoints[iType].X + 0.5F);
				pt2D.y = (int)(facePoints[iType].Y + 0.5F);
				cvCircle(pColorImg, pt2D, 4, cFaceColor);
			}
		}
	}

	if(pColorImg)
	{
		/*if(!pScaleColorImg)
		{
			pScaleColorImg = cvCreateImage(cvSize(pColorImg->width/2,pColorImg->height/2),IPL_DEPTH_32S,1);
		}
		cvResize(pColorImg, pScaleColorImg, CV_INTER_NN);*/
		cvShowImage( "ColorPreviewer", pColorImg );
	}
}

bool NuiCvVis::process()
{
	assert(m_pBuffer);
	if (!m_pBuffer)
		return true;

	bool bIsDrawDepth = false;
	std::shared_ptr<NuiCompositeFrame> pCompositeFrame = m_pBuffer->getLatestFrame();
	if(pCompositeFrame)
	{
		if(bIsDrawDepth)
		{
			DrawDepthImage(pCompositeFrame.get(), pCompositeFrame->m_depthFrame.GetMinDepth(), pCompositeFrame->m_depthFrame.GetMaxDepth(), m_pImg);
		}
		else
		{
			DrawColorImage(pCompositeFrame.get(), m_pImg);
		}
	}

	char cvKey = cvWaitKey( 50 );
	switch (cvKey)
	{
	case 's':
		break;
	default:
		break;
	}
	// Exit
	if( cvKey == 27 ){
		return false;
	}

	return true;
}