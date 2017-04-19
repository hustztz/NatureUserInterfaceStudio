#pragma once

#include "Foundation/SgVec3T.h"

struct NuiHashingChunkGridConfig
{
	bool						m_enable;
	SgVec3f						m_voxelExtends;		// extend of the voxels in meters
	SgVec3i						m_gridDimensions;	    // number of voxels in each dimension

	SgVec3i						m_minGridPos;
	SgVec3i						m_maxGridPos;

	unsigned int				m_initialChunkDescListSize;	 // initial size for vectors in the ChunkDesc

	NuiHashingChunkGridConfig();
};
