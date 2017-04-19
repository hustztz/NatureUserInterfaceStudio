#pragma once

#include "stdafx.h"

#include <vector>
#include <string>

struct NuiTrackerConfig
{
	typedef enum
	{
		eTracker_Iteration_Rotation = 1,
		eTracker_Iteration_Translation = 2,
		eTracker_Iteration_Both = 3,
		eTracker_Iteration_None = 4
	} TrackerIterationType;

	struct TrackerIterationParams
	{
		UINT m_num;
		TrackerIterationType m_type;
	};

	bool bHasColor;
	UINT filter_radius;				// bilateral filter radius
	float sigma_space2_inv_half;	// gaussian delta
	float sigma_depth2_inv_half;	// euclidean delta

	float depth_threshold;      // 1D distance threshold for depth leap
	float dist_threshold;       // 3D distance threshold for ICP correspondences
	float normal_threshold;     // dot product normal threshold for ICP correspondences
	float color_dist_threshold;       // 
	float color_gradiant_min;     // 
	float track_threshold;      // percent of tracked pixels to accept tracking result
	typedef std::vector<TrackerIterationParams> ITERATION_CLASS;
	ITERATION_CLASS iterations;  // max number of iterations per level

	NuiTrackerConfig();

	bool	load(const std::string& fileName);
	bool	log(const std::string& fileName) const;
};
