#include "NuiKinfuTrackingEngine.h"

#include "Foundation/NuiMatrixUtilities.h"

#include "DeviceSpecific/NuiKinfuScene.h"
#include "DeviceSpecific/OpenCL/NuiKinfuOpenCLFrame.h"
#include "DeviceSpecific/OpenCL/NuiKinfuOpenCLDepthTracker.h"

#define KINFU_DEFAULT_DEPTH_FOCAL_X 370.f
#define KINFU_DEFAULT_DEPTH_FOCAL_Y 370.f

NuiKinfuTrackingEngine::NuiKinfuTrackingEngine(NuiTrackerConfig& tracerConfig, UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight)
	: m_pTracker(NULL)
	, m_pFrame(NULL)
	, m_integration_metric_threshold(0.15f)
{
	m_pFrame = new NuiKinfuOpenCLFrame();
	m_pFrame->AcquireBuffers(nWidth, nHeight,nColorWidth, nColorHeight);

	m_pTracker = new NuiKinfuOpenCLDepthTracker(tracerConfig, nWidth, nHeight);
	reset( Vector3f::Zero() );
}

NuiKinfuTrackingEngine::NuiKinfuTrackingEngine()
	: m_pTracker(NULL)
	, m_pFrame(NULL)
	, m_integration_metric_threshold(0.15f)
{
	reset( Vector3f::Zero() );
}

NuiKinfuTrackingEngine::~NuiKinfuTrackingEngine()
{
	SafeDelete(m_pTracker);
	SafeDelete(m_pFrame);
}

void NuiKinfuTrackingEngine::initialize(const NuiTrackerConfig& tracerConfig, UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight)
{
	if(!m_pFrame)
		m_pFrame = new NuiKinfuOpenCLFrame();
	m_pFrame->AcquireBuffers(nWidth, nHeight, nColorWidth, nColorHeight);

	if(!m_pTracker)
		m_pTracker = new NuiKinfuOpenCLDepthTracker(tracerConfig, nWidth, nHeight);
}

void NuiKinfuTrackingEngine::reset(const Vector3f& translateBasis)
{
	if (m_poses.size() > 0)
		std::cout << "Reset" << std::endl;

	m_poses.clear ();
	m_poses.reserve (30000);

	m_cameraState.UpdateCameraTransform(Matrix3frm::Identity(), translateBasis);
	m_lastIntegrationPos = m_cameraState.GetCameraPos();
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

bool	NuiKinfuTrackingEngine::RunTracking(
	UINT16* pDepths,
	ColorSpacePoint* pDepthToColor,
	UINT nPointNum,
	const NuiColorImage& colorImage,
	NuiKinfuScene*	pScene,
	const NuiCameraParams& cameraParams)
{
	if(!m_pTracker || !m_pFrame)
		return true;

	if(!pDepths || 0 == nPointNum)
		return true;

	// Build the frame buffers
	m_cameraState.UpdateCameraParams(cameraParams, m_pFrame->GetWidth(), m_pFrame->GetHeight());
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
		if( !m_pTracker->EvaluateFrame(m_pFrame, &m_cameraState) )
			return false;

		if( pScene )
		{
			if( pScene->integrateVolume(m_pFrame, &m_cameraState) )
				return false;
		}
		m_pTracker->FeedbackPose(&m_cameraState);
	}
	else
	{
		if(!m_pTracker->EvaluateFrame(m_pFrame, &m_cameraState))
			return false;

		if( !m_pTracker->EstimatePose(&m_cameraState, NULL) )
			return false;
		// Debug
		//m_transform.setTransform(Matrix3frm::Identity(), Vector3f::Zero());
		if( pScene )
		{
			///////////////////////////////////////////////////////////////////////////////////////////
			// Integration check - We do not integrate volume if camera does not move.
			const NuiCameraPos& cameraPos = m_cameraState.GetCameraPos();
			float rnorm = NuiMatrixUtilities::rodrigues2(cameraPos.getRotation().inverse() * m_lastIntegrationPos.getRotation()).norm();
			float tnorm = (cameraPos.getTranslation() - m_lastIntegrationPos.getTranslation()).norm();
			const float alpha = 1.f;
			bool bNeedIntegrate = (rnorm + alpha * tnorm)/2 >= m_integration_metric_threshold;
			// Integrate
			if (bNeedIntegrate)
			{
				if( pScene->integrateVolume(m_pFrame, &m_cameraState) )
					return false;

				m_lastIntegrationPos = cameraPos;
			}
			m_pTracker->FeedbackPose(&m_cameraState, pScene);
		}
		else
		{
			// Only ICP
			m_pTracker->FeedbackPose(&m_cameraState);
		}
	}

	//save camera state
	m_poses.push_back( m_cameraState.GetCameraPos() );

	return true;
}

bool NuiKinfuTrackingEngine::previousBufferToData(NuiCLMappableData* pMappableData)
{
	assert(pMappableData);
	if(!pMappableData)
		return false;

	return m_pTracker ? m_pTracker->previousBufferToData(pMappableData) : NULL;
}

bool	NuiKinfuTrackingEngine::previousNormalImageToData(NuiCLMappableData* pMappableData)
{
	assert(pMappableData);
	if(!pMappableData)
		return false;

	return m_pTracker ? m_pTracker->previousNormalImageToData(pMappableData) : NULL;
}
