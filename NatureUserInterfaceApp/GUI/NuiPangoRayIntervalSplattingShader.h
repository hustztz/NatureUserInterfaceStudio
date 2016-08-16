#pragma once

#include "OpenCLUtilities/NuiMappable.h"

namespace NuiPangoRayIntervalSplattingShader
{
	void initializeShader(const std::string& shaderDir);
	void render(NuiMappable4f& vb, int size, UINT16 sensorDepthMin, UINT16 sensorDepthMax);

};