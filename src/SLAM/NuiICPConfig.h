#pragma once

#include "stdafx.h"

#include <vector>

struct NuiICPConfig
{
	UINT filter_radius;				// bilateral filter radius
	float sigma_space2_inv_half;	// gaussian delta
	float sigma_depth2_inv_half;	// euclidean delta

	float depth_threshold;      // 1D distance threshold for depth leap
	float dist_threshold;       // 3D distance threshold for ICP correspondences
	float normal_threshold;     // dot product normal threshold for ICP correspondences
	std::vector<UINT> iterations;  // max number of iterations per level
	float track_threshold;      // percent of tracked pixels to accept tracking result

	NuiICPConfig(){

		filter_radius = 3; //static_cast<int>(sigma_space * 1.5);
		sigma_space2_inv_half = 0.08f; // 0.5f / (sigma_space * sigma_space) //const float sigma_space = 2.5;     // in pixels
		sigma_depth2_inv_half = 50.0f; // 0.5f / (sigma_color * sigma_color) // const float sigma_color = 0.3;     //in meter

		depth_threshold = 0.3f; //meters
		dist_threshold = 0.1f; //meters
		normal_threshold = sin (20.f * 3.14159254f / 180.f); //0.8f;
		iterations.push_back( 5 );
		iterations.push_back( 5 );
		iterations.push_back( 5 );
		track_threshold = 0.15f;
	}
};
