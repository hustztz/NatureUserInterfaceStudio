#pragma once

#define MAX_OUTPUT_VERTEX_SIZE 5000000

struct TsdfParams
{
	float resolution[3];
	float dimension[3];
	float cell_size[3];
	float tranc_dist;
};

struct NuiCLCameraParams
{
	float fx;
	float fy;
	float fx_inv;
	float fy_inv;
	float cx;
	float cy;

	unsigned int depthImageWidth;
	unsigned int depthImageHeight;

	float sensorDepthWorldMin;
	float sensorDepthWorldMax;
};

struct NuiCLRigidTransform
{
	float R[9];
	float R_inv[9];
	float t[3];
};