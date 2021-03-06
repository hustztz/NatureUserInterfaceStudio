#pragma once

#include "SLAM/VisualOdometry/NuiKinfuVolumeConfig.h"
#include "SLAM/VisualOdometry/NuiKinfuTrackingManager.h"
#include "SLAM/VisualOdometry/NuiKinfuVertexCache.h"
#include "NuiSLAMPointCloudManager.h"

class NuiMeshShape;
class NuiKinfuScene;
class NuiCameraPos;
struct NuiCameraParams;

namespace NuiSLAMEngine
{
	class NuiSLAMController
	{
	public:
		enum NuiSLAMSceneMode
		{
			eScene_FusionVolume = 0,
			eScene_ShiftingVolume = 1,
			eScene_HashingVolume = 2,
		};
		enum NuiSLAMSceneDrawMode
		{
			eDraw_None = 0,
			eDraw_PointCloud = 1,
			eDraw_RealtimeMesh = 2,
			eDraw_PolygonMesh = 3,
		};
	public:
		NuiSLAMController();
		~NuiSLAMController();

		void	resetScene();
		void	setVolume(float voxelSize, int sceneMode);
		bool	evaluateCLData(NuiCLMappableData* pCLData, int drawMode);
		bool	getMesh(NuiMeshShape* pMesh);
		void	setSceneDirty();
		void	log(const std::string& fileName) const;

	protected:
		bool	CachePointCloud(NuiCLMappableData* pCLData);

	public:
		NuiKinfuEngine::NuiKinfuTrackingManager		m_tracker;

	private:
		/** \brief Tsdf volume container. */
		NuiKinfuScene*					m_pScene;
		NuiKinfuVertexCache				m_vertexCache;
		NuiSLAMPointCloudManager		m_pointCloudProcesser;
	};
}