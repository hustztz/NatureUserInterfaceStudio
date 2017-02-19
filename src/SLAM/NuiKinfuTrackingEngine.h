#pragma once

#include "NuiKinfuTransform.h"
#include "Shape\NuiCameraPos.h"
#include "DeviceSpecific\NuiTrackerConfig.h"
#include "DeviceSpecific\NuiKinfuFrameImpl.h"
#include "DeviceSpecific\NuiKinfuTracker.h"

//Forwards
class NuiCLMappableData;
class NuiKinfuVolume;
class NuiCameraParams;

typedef Eigen::Vector3i Vector3i;

class NuiKinfuTrackingEngine
{
public:
	NuiKinfuTrackingEngine(NuiTrackerConfig& tracerConfig, UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight);
	NuiKinfuTrackingEngine();
	~NuiKinfuTrackingEngine();

	bool	isInit() const { return (m_tracker ? true : false); }
	void	initialize(const NuiTrackerConfig& tracerConfig, UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight);
	bool	log(const std::string& fileName) const;

	/** \brief Performs the tracker reset to initial  state. It's used if case of camera tracking fail.  */
	void	reset(const Vector3f& translateBasis);

	bool	RunTracking(UINT16* pDepths,
		ColorSpacePoint* pDepthToColor,
		UINT nPointNum,
		const NuiColorImage& image,
		NuiKinfuVolume*	pVolume,
		const NuiCameraParams& cameraParams);

	/** \brief Returns camera pose at given time, default the last pose
        * \param[in] time Index of frame for which camera pose is returned.
        * \return camera pose
        */
    const NuiCameraPos&		getCameraPose (int time = -1) const;
	float					getTrackerError() const;
	float					getTrackerCount() const;
	void					setIntegrationMetricThreshold(float threshold) { m_integration_metric_threshold = threshold; }

	bool					previousBufferToData(NuiCLMappableData* pCLData);
	bool					previousNormalImageToData(NuiCLMappableData* pCLData);

private:
	NuiKinfuTracker*	m_tracker;
	NuiKinfuFrameImpl*	m_frame;
	NuiKinfuTransform m_transform;
	std::vector<NuiCameraPos> m_poses;

	NuiCameraPos		m_lastIntegrationPos;
	float				m_integration_metric_threshold;
};