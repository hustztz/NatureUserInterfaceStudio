#pragma once

#include "NuiKinfuTrackingEngine.h"
#include "NuiKinfuVolumeConfig.h"

#include <boost/thread/mutex.hpp>

class NuiMeshShape;
class NuiKinfuScene;
class NuiCameraPos;
struct NuiCameraParams;

namespace NuiKinfuEngine
{
	class NuiKinfuMainEngine
	{
	public:
		NuiKinfuMainEngine();
		~NuiKinfuMainEngine();

		void	resetTracker();
		void	resetVolume();
		void	setVolume(float voxelSize, bool bHashingSDF);
		bool	getCLData(NuiCLMappableData* pCLData, bool bIsMesh);
		bool	getMesh(NuiMeshShape* pMesh);
		bool	getCameraPose (NuiCameraPos* cam) const;

		float	getTrackerError() const { return m_trackingEngine.getTrackerError(); }
		int		getTrackerCount() const { return m_trackingEngine.getTrackerCount(); }
		int		getFrameID() const { return m_trackingEngine.getFrameID(); }

		void	setTranslateBasis(const Vector3f& basis);
		void	setIntegrationMetricThreshold(float threshold);
		void	setColorTracker(bool bHasColor) { m_trackingConfig.bHasColor = bHasColor; }
		void	log(const std::string& fileName) const;

		bool	processFrame(
			UINT16* pDepthBuffer,
			UINT* pDepthDistortionLT,
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

		boost::mutex						m_trackingMutex;
	};
}