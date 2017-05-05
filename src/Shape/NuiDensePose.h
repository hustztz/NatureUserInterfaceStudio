#pragma once

#include "NuiCameraPos.h"

struct NuiDensePose
{
	NuiDensePose(const NuiCameraPos& camPos)
		: m_cameraPos(camPos)
	{}
	NuiCameraPos	m_cameraPos;
};
