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

NuiKinfuTrackingEngine::NuiKinfuTrackingEngine(NuiTrackerConfig& tracerConfig, UINT nWidth, UINT nHeight)
	: m_pTracker(NULL)
	, m_pFrame(NULL)
	, m_pFeedbackFrame(NULL)
	, m_pCameraState(NULL)
	, m_integration_metric_threshold(0.15f)
{
	//initialize(tracerConfig, nWidth, nHeight);
}

NuiKinfuTrackingEngine::NuiKinfuTrackingEngine()
	: m_pTracker(NULL)
	, m_pFrame(NULL)
	, m_pFeedbackFrame(NULL)
	, m_pCameraState(NULL)
	, m_integration_metric_threshold(0.15f)
	, m_translateBasis(Vector3f::Zero())
{
	reset();
}

NuiKinfuTrackingEngine::~NuiKinfuTrackingEngine()
{
	SafeDelete(m_pFrame);
	SafeDelete(m_pFeedbackFrame);
	SafeDelete(m_pTracker);
	SafeDelete(m_pCameraState);
}

void NuiKinfuTrackingEngine::reset()
{
	if (m_poses.size() > 0)
		std::cout << "Reset" << std::endl;

	m_poses.clear ();
	m_poses.reserve (30000);

	if(!m_pCameraState)
		return;
	m_pCameraState->UpdateCameraTransform(Matrix3frm::Identity(), m_translateBasis);
	m_lastIntegrationPos = m_pCameraState->GetCameraPos();
}

void NuiKinfuTrackingEngine::initialize(const NuiTrackerConfig& trackerConfig, bool bAcceleratedFeedback, UINT nWidth, UINT nHeight)
{
	NuiKinfuTrackingFactory::Instance().BuildTrackingEngine(
		&m_pTracker, &m_pFrame, &m_pFeedbackFrame, &m_pCameraState, trackerConfig, bAcceleratedFeedback, nWidth, nHeight);

	reset();
}

bool	NuiKinfuTrackingEngine::RunTracking(
	INT64 timeStamp,
	UINT16* pDepths,
	BGRQUAD* pColors,
	UINT nPointNum,
	NuiKinfuScene*	pScene,
	const NuiCameraParams& cameraParams)
{
	if(!m_pTracker || !m_pFrame || !m_pCameraState)
		return true;

	if(!pDepths || 0 == nPointNum)
		return true;

	// Build the frame buffers
	m_pCameraState->UpdateCameraParams(cameraParams, m_pFrame->GetWidth(), m_pFrame->GetHeight());
	m_pFrame->UpdateVertexBuffers(pDepths, nPointNum, m_pCameraState);
	if( m_pTracker->hasColorData() ||
		(pScene && pScene->hasColorData()))
	{
		m_pFrame->UpdateColorBuffers(pColors, nPointNum);
		m_pFrame->setDirty();
	}
	
	// Tracking
	if (m_poses.size() == 0) //frameTime
	{
		m_pFeedbackFrame->UpdateBuffers(m_pFrame, m_pCameraState);
		m_pFeedbackFrame->setDirty();
		if( pScene )
		{
			if( !pScene->integrateVolume(m_pFrame, m_pCameraState) )
			{
				//std::warning
			}
		}
	}
	else
	{
		if( !m_pTracker->EstimatePose(m_pFrame, m_pFeedbackFrame, m_pCameraState, NULL) )
			return false;
		// Debug
		//m_transform.setTransform(Matrix3frm::Identity(), Vector3f::Zero());
		if( pScene )
		{
			///////////////////////////////////////////////////////////////////////////////////////////
			// Integration check - We do not integrate volume if camera does not move.
			const NuiCameraPos& cameraPos = m_pCameraState->GetCameraPos();
			float rnorm = NuiMatrixUtilities::rodrigues2(cameraPos.getRotation().inverse() * m_lastIntegrationPos.getRotation()).norm();
			float tnorm = (cameraPos.getGlobalTranslation() - m_lastIntegrationPos.getGlobalTranslation()).norm();
			const float alpha = 1.f;
			bool bNeedIntegrate = (rnorm + alpha * tnorm)/2 >= m_integration_metric_threshold;
			// Integrate
			if (bNeedIntegrate)
			{
				if( !pScene->integrateVolume(m_pFrame, m_pCameraState) )
					return false;

				m_lastIntegrationPos = cameraPos;
			}
			pScene->raycastRender(m_pFeedbackFrame, m_pCameraState);
			m_pFeedbackFrame->setDirty();
		}
		else
		{
			// Only ICP
			m_pFeedbackFrame->UpdateBuffers(m_pFrame, m_pCameraState);
		}
	}

	//save camera state
	m_poses.push_back( NuiDensePose(m_pCameraState->GetCameraPos()) );

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

int NuiKinfuTrackingEngine::getTrackerCount() const
{
	return m_pTracker ? m_pTracker->getCount() : 0;
}

const NuiCameraPos&	NuiKinfuTrackingEngine::getCameraPose (int time /*= -1*/) const
{
	if(m_poses.size() == 0)
		return m_lastIntegrationPos;

	if (time > (int)m_poses.size () || time < 0)
		time = (int)m_poses.size () - 1;

	return m_poses[time].m_cameraPos;
}

bool NuiKinfuTrackingEngine::VerticesToMappablePosition(NuiCLMappableData* pMappableData)
{
	return m_pFeedbackFrame ? m_pFeedbackFrame->VerticesToMappablePosition(pMappableData) : NULL;
}

bool	NuiKinfuTrackingEngine::BufferToMappableTexture(NuiCLMappableData* pMappableData)
{
	NuiKinfuFeedbackFrame::TrackerBufferType bufferType = NuiKinfuFeedbackFrame::eTracker_Ranges;
	bool returnStatus = false;
	if (m_pFeedbackFrame)
		returnStatus |= m_pFeedbackFrame->BufferToMappableTexture(pMappableData, bufferType);
	if (m_pFrame)
		returnStatus |= m_pFrame->BufferToMappableTexture(pMappableData);
	return returnStatus;
}
