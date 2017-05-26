#pragma once

#include "NuiKinfuTrackingEngine.h"
#include "NuiKinfuVolumeConfig.h"
#include "NuiKinfuPointCloudCache.h"

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
		enum NuiKinfuSceneMode
		{
			eScene_FusionVolume = 0,
			eScene_ShiftingVolume = 1,
			eScene_HashingVolume = 2,
		};
		enum NuiKinfuSceneDrawMode
		{
			eDraw_None = 0,
			eDraw_PointCloud = 1,
			eDraw_RealtimeMesh = 2,
			eDraw_PolygonMesh = 3,
		};
	public:
		NuiKinfuMainEngine();
		~NuiKinfuMainEngine();

		void	resetTracker();
		void	resetVolume();
		void	setVolume(float voxelSize, int sceneMode);
		bool	getCLData(NuiCLMappableData* pCLData, int drawMode);
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
			INT64 timeStamp,
			UINT16* pDepthBuffer,
			BGRQUAD* pColorBuffer,
			UINT nWidth,
			UINT nHeight,
			const NuiCameraParams& cameraParams
			);
	protected:
		void	CachePointCloud(NuiCLMappableData* pCLData);

	private:
		NuiKinfuTrackingEngine				m_trackingEngine;

		/** \brief Tsdf volume container. */
		NuiKinfuScene*						m_pScene;

		NuiKinfuPointCloudCache				m_cachedPointCloud;

		NuiTrackerConfig					m_trackingConfig;

		boost::mutex						m_trackingMutex;
	};
}