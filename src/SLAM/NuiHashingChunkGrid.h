#pragma once

#include "Kernels/hashing_gpu_def.h"
#include "Foundation/NuiBitArray.h"
#include "Foundation/SgVec3T.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

#include <vector>

// Forwards
class NuiHashingSDF;

struct NuiSDFBlock
{
	NuiCLVoxel data[SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];

	SgVec3ui delinearizeVoxelIndex(UINT idx) {
		UINT x = idx % SDF_BLOCK_SIZE;
		UINT y = (idx % (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE;
		UINT z = idx / (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE);	
		return SgVec3ui(x,y,z);
	}
};

class NuiSDFBlockDesc
{
public:

	NuiSDFBlockDesc()
		: ptr(LOCK_ENTRY)
	{
		pos[0] = pos[1] = pos[2] = 0;
	}

	NuiSDFBlockDesc(const NuiCLHashEntry& hashEntry) {
		pos[0] = hashEntry.pos[0];
		pos[1] = hashEntry.pos[1];
		pos[2] = hashEntry.pos[2];
		ptr = hashEntry.ptr;
	}

	bool operator<(const NuiSDFBlockDesc& other) const	{
		if(pos[0] == other.pos[0]) {
			if (pos[1] == other.pos[1]) {
				return pos[2] < other.pos[2];
			}
			return pos[1] < other.pos[1];
		}
		return pos[0] < other.pos[0];
	}

	bool operator==(const NuiSDFBlockDesc& other) const {
		return pos[0] == other.pos[0] && pos[1] == other.pos[1] && pos[2] == other.pos[2];
	}

	int			pos[3];
	int			ptr;
};

template<>
struct std::hash<NuiSDFBlockDesc> : public std::unary_function<NuiSDFBlockDesc, size_t> {
	size_t operator()(const NuiSDFBlockDesc& other) const
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
		m_SDFBlocks = std::vector<NuiSDFBlock>(); m_SDFBlocks.reserve(initialChunkListSize);
		m_ChunkDesc = std::vector<NuiSDFBlockDesc>(); m_ChunkDesc.reserve(initialChunkListSize);
	}

	void addSDFBlock(const NuiSDFBlockDesc& desc, const NuiSDFBlock& data) {
		m_ChunkDesc.push_back(desc);
		m_SDFBlocks.push_back(data);
	}

	UINT getNElements()	{
		return (UINT)m_SDFBlocks.size();
	}

	NuiSDFBlockDesc& getSDFBlockDesc(UINT i) {
		return m_ChunkDesc[i];
	}

	NuiSDFBlock& getSDFBlock(UINT i) {
		return m_SDFBlocks[i];
	}

	void clear() {
		m_ChunkDesc.clear();
		m_SDFBlocks.clear();
	}

	bool isStreamedOut() {
		return m_SDFBlocks.size() > 0;
	}

	std::vector<NuiSDFBlockDesc>& getSDFBlockDescs() {
		return m_ChunkDesc;
	}

	std::vector<NuiSDFBlock>& getSDFBlocks() {
		return m_SDFBlocks;
	}

	const std::vector<NuiSDFBlockDesc>& getSDFBlockDescs() const {
		return m_ChunkDesc;
	}

	const std::vector<NuiSDFBlock>& getSDFBlocks() const {
		return m_SDFBlocks;
	}

private:
	std::vector<NuiSDFBlock>		m_SDFBlocks;
	std::vector<NuiSDFBlockDesc>	m_ChunkDesc;
};

class NuiHashingChunkGrid
{
public:
	NuiHashingChunkGrid(NuiHashingSDF* pSDF);
	~NuiHashingChunkGrid();

	// Stream Out
	void streamOutToCPUAll(UINT nStreamOutParts, const SgVec3i& minGridPos, const SgVec3f& voxelExtends);
	UINT streamOutToCPU(UINT nStreamOutParts, const SgVec3f& posCamera, float radius, bool useParts);

	UINT streamOutToCPUPass0GPU(UINT nStreamOutParts, const SgVec3f& posCamera, float radius, bool useParts);
	void streamOutToCPUPass1CPU(UINT nStreamedBlocks);
	void integrateInChunkGrid(const int* desc, const int* block, unsigned int nSDFBlocks);

	// Stream In
	void streamInToGPUAll();
	void streamInToGPUAll(const SgVec3f& posCamera, float radius, bool useParts, UINT& nStreamedBlocks);

	void streamInToGPUChunk(const SgVec3f& chunkPos);
	void streamInToGPUChunkNeighborhood(const SgVec3i& chunkPos, int kernelRadius);
	void streamInToGPU(const SgVec3f& posCamera, float radius, bool useParts, unsigned int& nStreamedBlocks);

	void streamInToGPUPass0CPU(const SgVec3f& posCamera, float radius, bool useParts, bool multiThreaded = true);
	void streamInToGPUPass1GPU(bool multiThreaded = true);

	unsigned int integrateInHash(const SgVec3f& posCamera, float radius, bool useParts);

protected:
	void	AcquireBuffers();
	void	ReleaseBuffers();
	const SgVec3i& worldToChunks(const SgVec3f& posWorld, const SgVec3f& voxelExtends) const;

private:
	cl_mem						m_SDFBlockDescOutputCL;
	cl_mem						m_SDFBlockDescInputCL;
	cl_mem						m_SDFBlockOutputCL;
	cl_mem						m_SDFBlockInputCL;
	cl_mem						m_SDFBlockCounterCL;
	cl_mem						m_bitMaskCL;

	NuiHashingSDF*				m_pHashingSDF;

	std::vector<NuiChunkDesc*>	m_grid; // Grid data
	NuiBitArray<UINT>			m_bitMask;

	UINT						m_maxNumberOfSDFBlocksIntegrateFromGlobalHash;
	UINT						m_currentPart;
};
