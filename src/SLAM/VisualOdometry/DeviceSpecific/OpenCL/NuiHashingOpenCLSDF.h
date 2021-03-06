#pragma once

#include "stdafx.h"
#include "Foundation/SgVec3T.h"
#include "../../NuiHashingSDFConfig.h"

#include "OpenCLUtilities/NuiOpenCLUtil.h"

class NuiOpenCLPrefixSum;

class NuiHashingOpenCLSDF
{
public:
	NuiHashingOpenCLSDF(NuiHashingSDFConfig config);
	~NuiHashingOpenCLSDF();

	void	reset();
	void	resetHashBucketMutexBuffer();
	UINT	integrate(
		UINT nWidth, UINT nHeight,
		cl_mem floatDepthsCL,
		cl_mem colorsCL,
		cl_mem cameraParamsCL,
		cl_mem transformCL,
		cl_mem bitMaskCL,
		const SgVec3f&	voxelExtends,
		const SgVec3i&	gridDimensions,
		const SgVec3i&	minGridPos);

	const NuiHashingSDFConfig& getConfig() const { return m_config; }
	cl_mem	getHashCL() const { return m_hashCL; }
	cl_mem	getHashCompactifiedCL() const { return m_hashCompactified; }
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

	void	alloc(
		UINT nWidth, UINT nHeight,
		cl_mem floatDepthsCL,
		cl_mem cameraParamsCL,
		cl_mem transformCL,
		cl_mem bitMaskCL,
		const SgVec3f&	voxelExtends,
		const SgVec3i&	gridDimensions,
		const SgVec3i&	minGridPos);
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

	NuiOpenCLPrefixSum*	m_pScan;
	NuiHashingSDFConfig	m_config;
	bool			m_bGarbageCollectionEnabled;
	UINT			m_garbageCollectionStarve;

	UINT			m_numIntegratedFrames;	//used for garbage collect
};