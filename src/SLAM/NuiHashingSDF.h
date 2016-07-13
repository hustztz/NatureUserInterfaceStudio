#pragma once

#include "stdafx.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> Matrix4frm;

struct NuiHashParams {
	NuiHashParams() {
	}

	unsigned int	m_hashNumBuckets;
	unsigned int	m_hashMaxCollisionLinkedListSize;
	unsigned int	m_numSDFBlocks;

	float			m_virtualVoxelSize;
	unsigned int	m_numOccupiedBlocks;	//occupied blocks in the viewing frustum

	float			m_maxIntegrationDistance;
	float			m_truncScale;
	float			m_truncation;
	unsigned int	m_integrationWeightSample;
	unsigned int	m_integrationWeightMax;

	float			m_streamingVoxelExtents[3];
	int				m_streamingGridDimensions[3];
	int				m_streamingMinGridPos[3];
	unsigned int	m_streamingInitialChunkListSize;
	unsigned int	m_dummy[2];

};

class NuiHashingSDF
{
public:
	NuiHashingSDF();
	~NuiHashingSDF();

	void	reset();
	void	resetHashBucketMutexBuffer();
	void	integrate(UINT nWidth, UINT nHeight, cl_mem floatDepthsCL, cl_mem colorsCL, cl_mem cameraParamsCL, cl_mem transformCL, cl_mem bitMaskCL);

	const NuiHashParams& getParams() const { return m_params; }
	cl_mem	getHashCL() const { return m_hashCL; }
	cl_mem	getHeapCL() const { return m_heapCL; }
	cl_mem	getHeapCountCL() const { return m_heapCountCL; }
	cl_mem	getHashBucketMutexCL() const { return m_hashBucketMutexCL; }
	cl_mem	getSDFBlocksCL() const { return m_SDFBlocksCL; }

protected:
	void	AcquireBuffers();
	void	ReleaseBuffers();
	void	ResetBuffers();
	void	ResetHeapBuffer();
	void	ResetHashBuffer();

	void	alloc(UINT nWidth, UINT nHeight, cl_mem floatDepthsCL, cl_mem cameraParamsCL, cl_mem transformCL, cl_mem bitMaskCL);
	UINT	prefixSum();
	UINT	compactifyHashEntries(cl_mem cameraParamsCL, cl_mem transformCL);
	void	integrateDepthMap(UINT numOccupiedBlocks, cl_mem floatDepthsCL, cl_mem colorsCL, cl_mem cameraParamsCL, cl_mem transformCL);
	void	garbageCollect(bool bGarbageCollectionStarve, UINT numOccupiedBlocks, cl_mem cameraParamsCL);

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
	bool			m_bGarbageCollectionEnabled;
	UINT			m_garbageCollectionStarve;

	unsigned int	m_numIntegratedFrames;	//used for garbage collect
};