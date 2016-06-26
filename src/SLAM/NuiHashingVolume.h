#pragma once

#include "stdafx.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> Matrix4frm;

struct NuiHashParams {
	NuiHashParams() {
	}

	Matrix4frm		m_rigidTransform;
	Matrix4frm		m_rigidTransformInverse;

	unsigned int	m_hashNumBuckets;
	unsigned int	m_hashBucketSize;
	unsigned int	m_hashMaxCollisionLinkedListSize;
	unsigned int	m_numSDFBlocks;

	int				m_SDFBlockSize;
	float			m_virtualVoxelSize;
	unsigned int	m_numOccupiedBlocks;	//occupied blocks in the viewing frustum

	float			m_maxIntegrationDistance;
	float			m_truncScale;
	float			m_truncation;
	unsigned int	m_integrationWeightSample;
	unsigned int	m_integrationWeightMax;

	/*float3			m_streamingVoxelExtents;
	int3			m_streamingGridDimensions;
	int3			m_streamingMinGridPos;
	unsigned int	m_streamingInitialChunkListSize;
	uint2			m_dummy;*/

};

class NuiHashingVolume
{
public:
	NuiHashingVolume();
	~NuiHashingVolume();

	void	reset();

protected:
	void	AcquireBuffers();
	void	ReleaseBuffers();
	void	ResetBuffers();
	void	ResetHeapBuffer();
	void	ResetHashBuffer();
	void	ResetHashBucketMutexBuffer();

	void	AllocSDFs(UINT nWidth, UINT nHeight, cl_mem floatDepthsCL);

private:
	cl_mem		m_heapCL;
	cl_mem		m_heapCountCL;
	cl_mem		m_hashCL;
	cl_mem		m_hashDecisionCL;
	cl_mem		m_hashDecisionPrefixCL;
	cl_mem		m_hashCompactified;
	cl_mem		m_SDFBlocksCL;
	cl_mem		m_hashBucketMutexCL;

	NuiHashParams	m_params;

	unsigned int	m_numIntegratedFrames;	//used for garbage collect
};