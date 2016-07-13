#pragma once

#include "Kernels/hashing_gpu_def.h"
#include "Foundation/SgVec3T.h"
#include "Foundation/NuiBitArray.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

#include <vector>

// Forwards
class NuiHashingSDF;


struct NuiCLSDFBlock
{
	NuiCLVoxel data[SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];

	SgVec3ui delinearizeVoxelIndex(unsigned int idx) {
		unsigned int x = idx % SDF_BLOCK_SIZE;
		unsigned int y = (idx % (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE;
		unsigned int z = idx / (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE);	
		return SgVec3ui(x,y,z);
	}
};

class NuiCLSDFBlockDesc
{
public:
	NuiCLSDFBlockDesc()
		: ptr(LOCK_ENTRY)
	{
		pos[0] = pos[1] = pos[2] = 0;
	}

	NuiCLSDFBlockDesc(const NuiCLHashEntry& hashEntry) {
		pos[0] = hashEntry.pos[0];
		pos[1] = hashEntry.pos[1];
		pos[2] = hashEntry.pos[2];
		ptr = hashEntry.ptr;
	}

	bool operator<(const NuiCLSDFBlockDesc& other) const	{
		if(pos[0] == other.pos[0]) {
			if (pos[1] == other.pos[1]) {
				return pos[2] < other.pos[2];
			}
			return pos[1] < other.pos[1];
		}
		return pos[0] < other.pos[0];
	}

	bool operator==(const NuiCLSDFBlockDesc& other) const {
		return pos[0] == other.pos[0] && pos[1] == other.pos[1] && pos[2] == other.pos[2];
	}

	int			pos[3];
	int			ptr;
};

template<>
struct std::hash<NuiCLSDFBlockDesc> : public std::unary_function<NuiCLSDFBlockDesc, size_t> {
	size_t operator()(const NuiCLSDFBlockDesc& other) const
	{
		//TODO larger prime number (64 bit) to match size_t
		const size_t p0 = 73856093;
		const size_t p1 = 19349669;
		const size_t p2 = 83492791;
		const size_t res = ((size_t)other.pos[0] * p0)^((size_t)other.pos[1] * p1)^((size_t)other.pos[2] * p2);
		return res;
	}
};

class NuiChunkDesc {
public:
	NuiChunkDesc(UINT initialChunkListSize) {
		m_SDFBlocks = std::vector<NuiCLSDFBlock>(); m_SDFBlocks.reserve(initialChunkListSize);
		m_ChunkDesc = std::vector<NuiCLSDFBlockDesc>(); m_ChunkDesc.reserve(initialChunkListSize);
	}

	void addSDFBlock(const NuiCLSDFBlockDesc& desc, const NuiCLSDFBlock& data) {
		m_ChunkDesc.push_back(desc);
		m_SDFBlocks.push_back(data);
	}

	UINT getNElements()	{
		return (UINT)m_SDFBlocks.size();
	}

	NuiCLSDFBlockDesc& getSDFBlockDesc(UINT i) {
		return m_ChunkDesc[i];
	}

	NuiCLSDFBlock& getSDFBlock(UINT i) {
		return m_SDFBlocks[i];
	}

	void clear() {
		m_ChunkDesc.clear();
		m_SDFBlocks.clear();
	}

	bool isStreamedOut() {
		return m_SDFBlocks.size() > 0;
	}

	std::vector<NuiCLSDFBlockDesc>& getSDFBlockDescs() {
		return m_ChunkDesc;
	}

	std::vector<NuiCLSDFBlock>& getSDFBlocks() {
		return m_SDFBlocks;
	}

	const std::vector<NuiCLSDFBlockDesc>& getSDFBlockDescs() const {
		return m_ChunkDesc;
	}

	const std::vector<NuiCLSDFBlock>& getSDFBlocks() const {
		return m_SDFBlocks;
	}

private:
	std::vector<NuiCLSDFBlock>		m_SDFBlocks;
	std::vector<NuiCLSDFBlockDesc>	m_ChunkDesc;
};

class NuiHashingChunkGrid
{
public:
	NuiHashingChunkGrid(NuiHashingSDF* pSDF, const SgVec3i& gridDimensions, const SgVec3i& minGridPos, const SgVec3f& voxelExtends, UINT initialChunkListSize);
	~NuiHashingChunkGrid();

	// Stream Out
	void streamOutToCPUAll(UINT nStreamOutParts);
	UINT streamOutToCPU(UINT nStreamOutParts, const SgVec3f& posCamera, float radius, bool useParts);

	UINT streamOutToCPUPass0GPU(UINT nStreamOutParts, const SgVec3f& posCamera, float radius, bool useParts);
	void streamOutToCPUPass1CPU(UINT nStreamedBlocks);

	// Stream In
	void streamInToGPUAll();
	UINT streamInToGPUAll(const SgVec3f& posCamera, float radius, bool useParts);

	void streamInToGPUChunk(const SgVec3i& chunkPos);
	void streamInToGPUChunkNeighborhood(const SgVec3i& chunkPos, int kernelRadius);
	UINT streamInToGPU(const SgVec3f& posCamera, float radius, bool useParts);

	UINT streamInToGPUPass0CPU(const SgVec3f& posCamera, float radius, bool useParts);
	void streamInToGPUPass1GPU(UINT nStreamedBlocks);

	unsigned int integrateInHash(const SgVec3f& posCamera, float radius, bool useParts);

protected:
	void	AcquireBuffers();
	void	ReleaseBuffers();
	SgVec3f chunkToWorld(const SgVec3i& posChunk) const;
	SgVec3i worldToChunks(const SgVec3f& posWorld) const;
	bool	isValidChunk(const SgVec3i& chunk) const;
	UINT	linearizeChunkPos(const SgVec3i& chunkPos) const;
	float	getGridRadiusInMeter() const;
	float	getChunkRadiusInMeter() const {
		return m_voxelExtends.length()/2.0f;
	}
	SgVec3i	meterToNumberOfChunksCeil(float f) const;
	SgVec3i delinearizeChunkIndex(unsigned int idx);
	bool	isChunkInSphere(const SgVec3i& chunk, const SgVec3f& center, float radius) const;

private:
	NuiCLSDFBlock*				m_SDFBlockOutput;
	NuiCLSDFBlockDesc*			m_SDFBlockDescOutput;

	cl_mem						m_SDFBlockDescOutputCL;
	cl_mem						m_SDFBlockDescInputCL;
	cl_mem						m_SDFBlockOutputCL;
	cl_mem						m_SDFBlockInputCL;
	cl_mem						m_SDFBlockCounterCL;
	cl_mem						m_bitMaskCL;

	NuiHashingSDF*				m_pHashingSDF;

	std::vector<NuiChunkDesc*>	m_grid; // Grid data
	NuiBitArray<UINT>			m_bitMask;

	SgVec3f						m_voxelExtends;		// extend of the voxels in meters
	SgVec3i						m_gridDimensions;	    // number of voxels in each dimension

	SgVec3i						m_minGridPos;
	SgVec3i						m_maxGridPos;

	UINT						m_initialChunkDescListSize;	 // initial size for vectors in the ChunkDesc

	UINT						m_maxNumberOfSDFBlocksIntegrateFromGlobalHash;
	UINT						m_currentPart;
};
