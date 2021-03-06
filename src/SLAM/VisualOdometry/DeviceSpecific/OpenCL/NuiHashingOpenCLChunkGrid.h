#pragma once

#include "Kernels/hashing_gpu_def.h"
#include "../../NuiHashingChunkGridConfig.h"
#include "Foundation/NuiBitArray.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

#include <vector>

// Forwards
class NuiHashingOpenCLSDF;


struct NuiOpenCLSDFBlock
{
	NuiCLVoxel data[SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];

	SgVec3ui delinearizeVoxelIndex(unsigned int idx) {
		unsigned int x = idx % SDF_BLOCK_SIZE;
		unsigned int y = (idx % (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE;
		unsigned int z = idx / (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE);	
		return SgVec3ui(x,y,z);
	}
};

class NuiOpenCLSDFBlockDesc
{
public:
	NuiOpenCLSDFBlockDesc()
	{
		m_entry.ptr = LOCK_ENTRY;
		m_entry.pos[0] = m_entry.pos[1] = m_entry.pos[2] = 0;
	}

	NuiOpenCLSDFBlockDesc(const NuiCLHashEntry& hashEntry) {
		m_entry = hashEntry;
	}

	bool operator<(const NuiOpenCLSDFBlockDesc& other) const	{
		if(m_entry.pos[0] == other.m_entry.pos[0]) {
			if (m_entry.pos[1] == other.m_entry.pos[1]) {
				return m_entry.pos[2] < other.m_entry.pos[2];
			}
			return m_entry.pos[1] < other.m_entry.pos[1];
		}
		return m_entry.pos[0] < other.m_entry.pos[0];
	}

	bool operator==(const NuiOpenCLSDFBlockDesc& other) const {
		return m_entry.pos[0] == other.m_entry.pos[0] && m_entry.pos[1] == other.m_entry.pos[1] && m_entry.pos[2] == other.m_entry.pos[2];
	}

	NuiCLHashEntry m_entry;
};

template<>
struct std::hash<NuiOpenCLSDFBlockDesc> : public std::unary_function<NuiOpenCLSDFBlockDesc, size_t> {
	size_t operator()(const NuiOpenCLSDFBlockDesc& other) const
	{
		//TODO larger prime number (64 bit) to match size_t
		const size_t p0 = 73856093;
		const size_t p1 = 19349669;
		const size_t p2 = 83492791;
		const size_t res = ((size_t)other.m_entry.pos[0] * p0)^((size_t)other.m_entry.pos[1] * p1)^((size_t)other.m_entry.pos[2] * p2);
		return res;
	}
};

class NuiChunkDesc {
public:
	NuiChunkDesc(UINT initialChunkListSize) {
		m_SDFBlocks = std::vector<NuiOpenCLSDFBlock>(); m_SDFBlocks.reserve(initialChunkListSize);
		m_ChunkDesc = std::vector<NuiOpenCLSDFBlockDesc>(); m_ChunkDesc.reserve(initialChunkListSize);
	}

	void addSDFBlock(const NuiOpenCLSDFBlockDesc& desc, const NuiOpenCLSDFBlock& data) {
		m_ChunkDesc.push_back(desc);
		m_SDFBlocks.push_back(data);
	}

	UINT getNElements()	{
		return (UINT)m_SDFBlocks.size();
	}

	NuiOpenCLSDFBlockDesc& getSDFBlockDesc(UINT i) {
		return m_ChunkDesc[i];
	}

	NuiOpenCLSDFBlock& getSDFBlock(UINT i) {
		return m_SDFBlocks[i];
	}

	void clear() {
		m_ChunkDesc.clear();
		m_SDFBlocks.clear();
	}

	bool isStreamedOut() {
		return m_SDFBlocks.size() > 0;
	}

	std::vector<NuiOpenCLSDFBlockDesc>& getSDFBlockDescs() {
		return m_ChunkDesc;
	}

	std::vector<NuiOpenCLSDFBlock>& getSDFBlocks() {
		return m_SDFBlocks;
	}

	const std::vector<NuiOpenCLSDFBlockDesc>& getSDFBlockDescs() const {
		return m_ChunkDesc;
	}

	const std::vector<NuiOpenCLSDFBlock>& getSDFBlocks() const {
		return m_SDFBlocks;
	}

private:
	std::vector<NuiOpenCLSDFBlock>		m_SDFBlocks;
	std::vector<NuiOpenCLSDFBlockDesc>	m_ChunkDesc;
};

class NuiHashingOpenCLChunkGrid
{
public:
	NuiHashingOpenCLChunkGrid(NuiHashingOpenCLSDF* pSDF);
	~NuiHashingOpenCLChunkGrid();

	void reset();
	void updateConfig(const NuiHashingChunkGridConfig& config) { m_config = config; }
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

	cl_mem	getBitMaskCL() const { return m_bitMaskCL; }
	const NuiHashingChunkGridConfig& getConfig() const { return m_config; }

protected:
	void	AcquireBuffers();
	void	ReleaseBuffers();
	SgVec3f chunkToWorld(const SgVec3i& posChunk) const;
	SgVec3i worldToChunks(const SgVec3f& posWorld) const;
	bool	isValidChunk(const SgVec3i& chunk) const;
	UINT	linearizeChunkPos(const SgVec3i& chunkPos) const;
	float	getGridRadiusInMeter() const;
	float	getChunkRadiusInMeter() const {
		return m_config.m_voxelExtends.length()/2.0f;
	}
	SgVec3i	meterToNumberOfChunksCeil(float f) const;
	SgVec3i delinearizeChunkIndex(unsigned int idx);
	bool	isChunkInSphere(const SgVec3i& chunk, const SgVec3f& center, float radius) const;

private:
	NuiOpenCLSDFBlock*				m_SDFBlockOutput;
	NuiOpenCLSDFBlockDesc*			m_SDFBlockDescOutput;

	cl_mem						m_SDFBlockDescOutputCL;
	cl_mem						m_SDFBlockDescInputCL;
	cl_mem						m_SDFBlockOutputCL;
	cl_mem						m_SDFBlockInputCL;
	cl_mem						m_SDFBlockCounterCL;
	cl_mem						m_bitMaskCL;

	NuiHashingOpenCLSDF*				m_pHashingSDF;

	std::vector<NuiChunkDesc*>	m_grid; // Grid data
	NuiBitArray<UINT>			m_bitMask;

	NuiHashingChunkGridConfig	m_config;
	UINT						m_maxNumberOfSDFBlocksIntegrateFromGlobalHash;
	UINT						m_currentPart;
};
