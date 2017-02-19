#pragma once
#include "NuiCameraIntrinsics.h"

struct NuiCameraParams
{
	NuiCameraIntrinsics		m_intrinsics;

	float m_sensorDepthMin;
	float m_sensorDepthMax;
};