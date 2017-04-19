#include "NuiHashingSDFConfig.h"

NuiHashingSDFConfig::NuiHashingSDFConfig()
	: m_hashNumBuckets(512*512)
	, m_numSDFBlocks(400000)
	, m_hashMaxCollisionLinkedListSize(7)
	, m_virtualVoxelSize(0.005f)
	, m_truncScale(0.02f)
	, m_truncation(0.06f)
	, m_integrationWeightSample(3)
	, m_integrationWeightMax(255)
	, m_bUseSwapping(false)
	, m_bUseForwardRender(false)
{
}