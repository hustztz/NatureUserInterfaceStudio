#pragma once

struct NuiHashingSDFConfig {
	NuiHashingSDFConfig();

	bool			m_bUseSwapping;
	bool			m_bUseForwardRender;

	unsigned int	m_hashNumBuckets; //smaller voxels require more space
	unsigned int	m_numSDFBlocks; //smaller voxels require more space
	unsigned int	m_hashMaxCollisionLinkedListSize;

	float			m_virtualVoxelSize; //voxel size in meter

	float			m_truncScale; //truncation scale in meter per meter (2.5f*s_SDFVoxelSize )
	float			m_truncation; //truncation in meter (5.0f*s_SDFVoxelSize )
	unsigned int	m_integrationWeightSample; //weight for an integrated depth value
	unsigned char	m_integrationWeightMax; //maximum integration weight for a voxel
};