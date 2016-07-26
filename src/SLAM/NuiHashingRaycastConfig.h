#pragma once

struct NuiHashingRaycastConfig {
	NuiHashingRaycastConfig();

	float m_rayIncrementFactor; //(don't touch) s_SDFRayIncrement = s_SDFRayIncrementFactor*s_SDFTrunaction;
	float m_thresSampleDistFactor; //(don't touch) s_SDFRayThresSampleDist = s_SDFRayThresSampleDistFactor*s_rayIncrement;
	float m_thresDistFactor; //(don't touch) s_SDFRayThresDist = s_SDFRayThresSampleDistFactor*s_rayIncrement;
};