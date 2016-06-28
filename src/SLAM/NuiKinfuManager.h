#pragma once

#include "Foundation\NuiThreadObject.h"

#include "NuiKinfuTracker.h"
#include "NuiKinfuVolumeConfig.h"
#include "Frame/Buffer/NuiFrameCircleBuffer.h"

class NuiMeshShape;
class NuiKinfuVolume;
class NuiCameraParams;

class NuiKinfuManager : public NuiThreadObject
{
public:
	NuiKinfuManager();
	virtual ~NuiKinfuManager();

	virtual void	reset() override;
	void	resetVolume();
	bool	pushbackFrame(std::shared_ptr<NuiCompositeFrame> pFrame) { return m_buffer.pushbackCompositeFrame(pFrame); }
	bool	getCLData(NuiCLMappableData* pCLData, bool bIsMesh);
	bool	getMesh(NuiMeshShape* pMesh);
    bool	getCameraPose (NuiCameraParams* cam) const;
	size_t	getLagFrames() { return m_buffer.size(); }

	float	getTrackerError() const { return m_tracker.getIcpError(); }
	float	getTrackerCount() const { return m_tracker.getIcpCount(); }

	void	setAutoReset(bool autoReset) { m_bAutoReset = autoReset; }
	void	setTranslateBasis(const Vector3f& basis) { m_translateBasis = basis; }
	void	setIntegrationMetricThreshold(float threshold) { m_tracker.setIntegrationMetricThreshold(threshold); }

public:
	NuiICPConfig						m_trackerConfig;
	NuiKinfuVolumeConfig				m_volumeConfig;

private:
	virtual bool process() override;

private:
	NuiKinfuTracker						m_tracker;
	NuiFrameCircleBuffer				m_buffer;

	/** \brief Tsdf volume container. */
	NuiKinfuVolume*						m_tsdf_volume;

	NuiColorImage						m_frameColorImage;

	bool								m_bAutoReset;
	Vector3f							m_translateBasis;
};