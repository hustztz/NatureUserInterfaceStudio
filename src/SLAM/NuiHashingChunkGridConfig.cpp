#include "NuiHashingChunkGridConfig.h"

NuiHashingChunkGridConfig::NuiHashingChunkGridConfig()
	: m_enable(false)
	, m_initialChunkDescListSize(2000)
{
	m_voxelExtends = SgVec3f(1.0f, 1.0f, 1.0f);
	m_gridDimensions = SgVec3i(257, 257, 257); // dimensions have to be odd (number of samples)
	m_minGridPos = SgVec3i(-128, -128, -128);
	m_maxGridPos = -m_minGridPos;
}