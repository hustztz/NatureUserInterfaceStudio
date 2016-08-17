#pragma once

#include "OpenCLUtilities/NuiMappable.h"

namespace NuiPangoRayIntervalSplattingShader
{
	void initializeShader();
	void render(NuiMappable4f& vb, NuiMappablef& rbMin, NuiMappablef& rbMax, int size, float sensorDepthMin, float sensorDepthMax);

};