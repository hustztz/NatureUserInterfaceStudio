#pragma once

#include "stdafx.h"
#include "vector"

#include <Foundation/SgVec3T.h>

namespace NuiExperimentCPU
{
	SgVec3f estimateNormalsCovariance(std::vector<SgVec3f>& positions, UINT gidx, UINT gidy, UINT gsizex, UINT gsizey, float  depthThreshold);
}