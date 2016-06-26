#pragma once

// Forwards
class NuiCompositeFrame;
class NuiCLMappableData;
class NuiMeshShape;

namespace NuiFrameUtilities
{
	bool		FrameToPointCloud (NuiCompositeFrame* pCompositeFrame);
	bool		FrameToCompoundImage (NuiCompositeFrame* pCompositeFrame);
	bool		FrameToMappableData(NuiCompositeFrame* pCompositeFrame, NuiCLMappableData* pData, int indexFlags, bool bOnlyShowBody, float depthThreshold);
	bool		FrameToMesh(NuiCompositeFrame* pCompositeFrame, NuiMeshShape* pMesh, bool bOnlyShowBody, float depthThreshold);
}