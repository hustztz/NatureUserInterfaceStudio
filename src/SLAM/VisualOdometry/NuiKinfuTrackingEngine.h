#pragma once

#include "DeviceSpecific/NuiKinfuCameraState.h"
#include "NuiTrackerConfig.h"

#include "Shape\NuiImageBuffer.h"

//Forwards
class NuiCLMappableData;
class NuiKinfuScene;
class NuiKinfuFrame;
class NuiKinfuFeedbackFrame;
class NuiKinfuTracker;

typedef Eigen::Vector3i Vector3i;

namespace NuiKinfuEngine
{
	class NuiKinfuTrackingEngine
	{
	public:
		NuiKinfuTrackingEngine(NuiTrackerConfig& tracerConfig, UINT nWidth, UINT nHeight);
		NuiKinfuTrackingEngine();
		~NuiKinfuTrackingEngine();

		bool	isInit() const { return (m_pTracker ? true : false); }
		void	initialize(const NuiTrackerConfig& tracerConfig, bool bAcceleratedFeedback, UINT nWidth, UINT nHeight);
		bool	log(const std::string& fileName) const;

		/** \brief Performs the tracker reset to initial  state. It's used if case of camera tracking fail.  */
		void	reset(const Vector3f& translateBasis);

		bool	RunTracking(UINT16* pDepths,
			UINT* pDepthDistortionLT,
			ColorSpacePoint* pDepthToColor,
			UINT nPointNum,
			const NuiColorImage& image,
			NuiKinfuScene*	pVolume,
			const NuiCameraParams& cameraParams);

		/** \brief Returns camera pose at given time, default the last pose
			* \param[in] time Index of frame for which camera pose is returned.
			* \return camera pose
			*/
		const NuiCameraPos&		getCameraPose (int time = -1) const;
		float					getTrackerError() const;
		int						getTrackerCount() const;
		int						getFrameID() const { return (int)m_poses.size(); }
		void					setIntegrationMetricThreshold(float threshold) { m_integration_metric_threshold = threshold; }

		bool                    VerticesToMappablePosition(NuiCLMappableData* pCLData);
		bool                    BufferToMappableTexture(NuiCLMappableData* pCLData);

	private:
		NuiKinfuTracker*		m_pTracker;
		NuiKinfuFrame*			m_pFrame;
		NuiKinfuFeedbackFrame*	m_pFeedbackFrame;
		NuiKinfuCameraState*	m_pCameraState;
		std::vector<NuiCameraPos> m_poses;

		NuiCameraPos			m_lastIntegrationPos;
		float					m_integration_metric_threshold;
	};
}