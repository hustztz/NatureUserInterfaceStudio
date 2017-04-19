#pragma once

// Forwards
class NuiCLMappableData;

namespace NuiMeshingUtil
{
	void	EvaluateMappableDataCPU(NuiCLMappableData* pCLData, bool bIsMesh, bool bOnlyShowBody);


	void	NormalEstimationCL(NuiCLMappableData* pCLData);
	void	SmoothPositionCL(NuiCLMappableData* pCLData);
	void	CalcColorCL(NuiCLMappableData* pCLData);
}