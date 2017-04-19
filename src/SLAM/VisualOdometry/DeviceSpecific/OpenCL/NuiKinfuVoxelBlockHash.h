#pragma once

#include "stdafx.h"
#include "Foundation/SgVec3T.h"
#include "../../NuiHashingSDFConfig.h"

#include "OpenCLUtilities/NuiOpenCLUtil.h"

class NuiKinfuVoxelBlockHash
{
public:
	NuiKinfuVoxelBlockHash();
	~NuiKinfuVoxelBlockHash();

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

	cl_mem	getHashEntriesCL() const { return m_hashEntriesCL; }
	cl_mem	getExcessAllocationListCL() const { return m_excessAllocationListCL; }
	cl_mem	getLastFreeExcessListIdCL() const { return m_lastFreeExcessListIdCL; }
	cl_mem	getVoxelBlocksCL() const { return m_voxelBlocksCL; }
	cl_mem	getVoxelAllocationListCL() const { return m_allocationListCL; }
	cl_mem	getLastFreeBlockIdCL() const { return m_lastFreeBlockIdCL; }

protected:
	void	AcquireBuffers();
	void	ReleaseBuffers();
	void	ResetVoxelBlockBuffer();
	void	ResetHashEntryBuffer();

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
	// Hash entry
	cl_mem		m_hashEntriesCL;
	cl_mem		m_excessAllocationListCL;
	cl_mem		m_lastFreeExcessListIdCL;

	// Voxel Block
	cl_mem		m_voxelBlocksCL;
	cl_mem		m_allocationListCL;
	cl_mem		m_lastFreeBlockIdCL;
};