#include "NuiKinfuTrackingEngine.h"

#include "Foundation/NuiMatrixUtilities.h"

#include "DeviceSpecific/NuiKinfuFrame.h"
#include "DeviceSpecific/NuiKinfuTracker.h"
#include "DeviceSpecific/NuiKinfuScene.h"
#include "DeviceSpecific/NuiKinfuTrackingFactory.h"

#include <iostream>

#define KINFU_DEFAULT_DEPTH_FOCAL_X 370.f
#define KINFU_DEFAULT_DEPTH_FOCAL_Y 370.f

using namespace NuiKinfuEngine;

NuiKinfuTrackingEngine::NuiKinfuTrackingEngine(NuiTrackerConfig& tracerConfig, UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight)
	: m_pTracker(NULL)
	, m_pFrame(NULL)
	, m_pCameraState(NULL)
	, m_integration_metric_threshold(0.15f)
{
	initialize(tracerConfig, nWidth, nHeight, nColorWidth, nColorHeight);
}

NuiKinfuTrackingEngine::NuiKinfuTrackingEngine()
	: m_pTracker(NULL)
	, m_pFrame(NULL)
	, m_pCameraState(NULL)
	, m_integration_metric_threshold(0.15f)
{
	reset( Vector3f::Zero() );
}

NuiKinfuTrackingEngine::~NuiKinfuTrackingEngine()
{
	SafeDelete(m_pFrame);
	SafeDelete(m_pTracker);
	SafeDelete(m_pCameraState);
}

void NuiKinfuTrackingEngine::reset(const Vector3f& translateBasis)
{
	if (m_poses.size() > 0)
		std::cout << "Reset" << std::endl;

	m_poses.clear ();
	m_poses.reserve (30000);

	if(!m_pCameraState)
		return;
	m_pCameraState->UpdateCameraTransform(Matrix3frm::Identity(), translateBasis);
	m_lastIntegrationPos = m_pCameraState->GetCameraPos();
}

void NuiKinfuTrackingEngine::initialize(const NuiTrackerConfig& trackerConfig, UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight)
{
	NuiKinfuTrackingFactory::Instance().BuildTrackingEngine(
		&m_pTracker, &m_pFrame, &m_pCameraState, trackerConfig, nWidth, nHeight, nColorWidth, nColorHeight);

	reset( Vector3f::Zero() );
}

bool	NuiKinfuTrackingEngine::RunTracking(
	UINT16* pDepths,
	ColorSpacePoint* pDepthToColor,
	UINT nPointNum,
	const NuiColorImage& colorImage,
	NuiKinfuScene*	pScene,
	const NuiCameraParams& cameraParams)
{
	if(!m_pTracker || !m_pFrame || !m_pCameraState)
		return true;

	if(!pDepths || 0 == nPointNum)
		return true;

	// Build the frame buffers
	m_pCameraState->UpdateCameraParams(cameraParams, m_pFrame->GetWidth(), m_pFrame->GetHeight());
	m_pFrame->UpdateDepthBuffers(pDepths, nPointNum, cameraParams.m_sensorDepthMin, cameraParams.m_sensorDepthMax);
	if( m_pTracker->hasColorData() ||
		(pScene && pScene->hasColorData()))
	{
		m_pFrame->UpdateColorBuffers(pDepthToColor, nPointNum, colorImage);
	}
	
	// Tracking
	UINT frameTime = (UINT)m_poses.size();
	if (frameTime == 0)
	{
		if( !m_pTracker->EvaluateFrame(m_pFrame, m_pCameraState) )
			return false;

		if( pScene )
		{
			if( pScene->integrateVolume(m_pFrame, m_pCameraState) )
				return false;
		}
		m_pTracker->FeedbackPose(m_pCameraState);
	}
	else
	{
		if(!m_pTracker->EvaluateFrame(m_pFrame, m_pCameraState))
			return false;

		if( !m_pTracker->EstimatePose(m_pCameraState, NULL) )
			return false;
		// Debug
		//m_transform.setTransform(Matrix3frm::Identity(), Vector3f::Zero());
		if( pScene )
		{
			///////////////////////////////////////////////////////////////////////////////////////////
			// Integration check - We do not integrate volume if camera does not move.
			const NuiCameraPos& cameraPos = m_pCameraState->GetCameraPos();
			float rnorm = NuiMatrixUtilities::rodrigues2(cameraPos.getRotation().inverse() * m_lastIntegrationPos.getRotation()).norm();
			float tnorm = (cameraPos.getTranslation() - m_lastIntegrationPos.getTranslation()).norm();
			const float alpha = 1.f;
			bool bNeedIntegrate = (rnorm + alpha * tnorm)/2 >= m_integration_metric_threshold;
			// Integrate
			if (bNeedIntegrate)
			{
				if( pScene->integrateVolume(m_pFrame, m_pCameraState) )
					return false;

				m_lastIntegrationPos = cameraPos;
			}
			m_pTracker->FeedbackPose(m_pCameraState, pScene);
		}
		else
		{
			// Only ICP
			m_pTracker->FeedbackPose(m_pCameraState);
		}
	}

	//save camera state
	m_poses.push_back( m_pCameraState->GetCameraPos() );

	return true;
}

bool NuiKinfuTrackingEngine::log(const std::string& fileName) const
{
	return m_pTracker ? m_pTracker->log(fileName) : false;
}

float NuiKinfuTrackingEngine::getTrackerError() const
{
	return m_pTracker ? m_pTracker->getError() : 0.0f;
}

float NuiKinfuTrackingEngine::getTrackerCount() const
{
	return m_pTracker ? m_pTracker->getCount() : 0.0f;
}

const NuiCameraPos&	NuiKinfuTrackingEngine::getCameraPose (int time /*= -1*/) const
{
	if(m_poses.size() == 0)
		return m_lastIntegrationPos;

	if (time > (int)m_poses.size () || time < 0)
		time = (int)m_poses.size () - 1;

	return m_poses[time];
}

bool NuiKinfuTrackingEngine::VerticesToMappablePosition(NuiCLMappableData* pMappableData)
{
	return m_pTracker ? m_pTracker->VerticesToMappablePosition(pMappableData) : NULL;
}

bool	NuiKinfuTrackingEngine::BufferToMappableTexture(NuiCLMappableData* pMappableData)
{
	NuiKinfuTracker::BufferType bufferType = NuiKinfuTracker::eTracking_Vertices;
	return m_pTracker ? m_pTracker->BufferToMappableTexture(pMappableData, bufferType) : NULL;
}
