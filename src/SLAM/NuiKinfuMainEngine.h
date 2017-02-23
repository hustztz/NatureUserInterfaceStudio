#pragma once

#include "NuiKinfuTrackingEngine.h"
#include "NuiKinfuVolumeConfig.h"

class NuiMeshShape;
class NuiKinfuScene;
class NuiCameraPos;
class NuiCameraParams;

class NuiKinfuMainEngine
{
public:
	NuiKinfuMainEngine();
	~NuiKinfuMainEngine();

	void	resetTracker();
	void	resetVolume();
	void	setVolume(float voxelSize, bool bHashingSDF);
	void	offlineRender();
	bool	getCLData(NuiCLMappableData* pCLData, bool bIsMesh);
	bool	getMesh(NuiMeshShape* pMesh);
	bool	getCameraPose (NuiCameraPos* cam) const;

	float	getTrackerError() const { return m_trackingEngine.getTrackerError(); }
	float	getTrackerCount() const { return m_trackingEngine.getTrackerCount(); }

	void	setTranslateBasis(const Vector3f& basis) { m_translateBasis = basis; }
	void	setIntegrationMetricThreshold(float threshold);
	void	setColorTracker(bool bHasColor) { m_trackingConfig.bHasColor = bHasColor; }
	void	log(const std::string& fileName) const;

	bool	processFrame(
		UINT16* pDepthBuffer,
		ColorSpacePoint* pDepthToColor,
		UINT nWidth,
		UINT nHeight,
		const NuiColorImage& image,
		const NuiCameraParams& cameraParams
		);

private:
	NuiKinfuTrackingEngine				m_trackingEngine;

	/** \brief Tsdf volume container. */
	NuiKinfuScene*						m_pScene;

	NuiTrackerConfig					m_trackingConfig;
	Vector3f							m_translateBasis;
};