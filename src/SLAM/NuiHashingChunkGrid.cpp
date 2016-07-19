#include "NuiHashingChunkGrid.h"
#include "NuiHashingSDF.h"

#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

#include <assert.h>

NuiHashingChunkGrid::NuiHashingChunkGrid(NuiHashingSDF* pSDF)
	: m_pHashingSDF(pSDF)
	, m_currentPart(0)
	, m_maxNumberOfSDFBlocksIntegrateFromGlobalHash(100000)
{
	AcquireBuffers();
}

NuiHashingChunkGrid::~NuiHashingChunkGrid()
{
	ReleaseBuffers();
}

void NuiHashingChunkGrid::AcquireBuffers()
{
	m_SDFBlockOutput = new NuiCLSDFBlock[m_maxNumberOfSDFBlocksIntegrateFromGlobalHash];
	m_SDFBlockDescOutput = new NuiCLSDFBlockDesc[m_maxNumberOfSDFBlocksIntegrateFromGlobalHash];

	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	m_SDFBlockDescOutputCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, sizeof(NuiCLHashEntry)*m_maxNumberOfSDFBlocksIntegrateFromGlobalHash, m_SDFBlockDescOutput, &err);
	NUI_CHECK_CL_ERR(err);
	m_SDFBlockDescInputCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(NuiCLHashEntry)*m_maxNumberOfSDFBlocksIntegrateFromGlobalHash, NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_SDFBlockOutputCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, sizeof(NuiCLSDFBlock)*m_maxNumberOfSDFBlocksIntegrateFromGlobalHash, m_SDFBlockOutput, &err);
	NUI_CHECK_CL_ERR(err);
	m_SDFBlockInputCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(NuiCLSDFBlock)*m_maxNumberOfSDFBlocksIntegrateFromGlobalHash, NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_SDFBlockCounterCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(cl_uint), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_bitMaskCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_bitMask.getByteWidth(), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}

void NuiHashingChunkGrid::ReleaseBuffers()
{
	if (m_SDFBlockDescOutputCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_SDFBlockDescOutputCL);
		NUI_CHECK_CL_ERR(err);
		m_SDFBlockDescOutputCL = NULL;
	}
	if (m_SDFBlockDescInputCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_SDFBlockDescInputCL);
		NUI_CHECK_CL_ERR(err);
		m_SDFBlockDescInputCL = NULL;
	}
	if (m_SDFBlockOutputCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_SDFBlockOutputCL);
		NUI_CHECK_CL_ERR(err);
		m_SDFBlockOutputCL = NULL;
	}
	if (m_SDFBlockInputCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_SDFBlockInputCL);
		NUI_CHECK_CL_ERR(err);
		m_SDFBlockInputCL = NULL;
	}
	if (m_SDFBlockCounterCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_SDFBlockCounterCL);
		NUI_CHECK_CL_ERR(err);
		m_SDFBlockCounterCL = NULL;
	}
	if (m_bitMaskCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_bitMaskCL);
		NUI_CHECK_CL_ERR(err);
		m_bitMaskCL = NULL;
	}

	SafeDeleteArray(m_SDFBlockOutput);
	SafeDeleteArray(m_SDFBlockDescOutput);
}

void NuiHashingChunkGrid::reset()
{
	for (unsigned int i = 0; i<m_grid.size(); i++) {
		SafeDelete(m_grid[i]);
	}
	m_grid.clear();
	m_bitMask.reset();
}

SgVec3i NuiHashingChunkGrid::worldToChunks(const SgVec3f& posWorld) const {
	SgVec3f p;
	p[0] = posWorld[0] / m_config.m_voxelExtends[0];
	p[1] = posWorld[1] / m_config.m_voxelExtends[1];
	p[2] = posWorld[2] / m_config.m_voxelExtends[2];

	SgVec3f s;
	s[0] = (float)sign(p[0]);
	s[1] = (float)sign(p[1]);
	s[2] = (float)sign(p[2]);

	return SgVec3i(p+s*0.5f);
}

void NuiHashingChunkGrid::streamOutToCPUAll(UINT nStreamOutParts)
{
	unsigned int nStreamedBlocksSum = 1;
	while(nStreamedBlocksSum != 0) {
		nStreamedBlocksSum = 0;
		for (unsigned int i = 0; i < nStreamOutParts; i++) {
			unsigned int nStreamedBlocks = streamOutToCPU(
				nStreamOutParts,
				worldToChunks((m_config.m_minGridPos-SgVec3i(1, 1, 1))),
				0.0f,
				true);
			nStreamedBlocksSum += nStreamedBlocks;
		}
	}
}

UINT NuiHashingChunkGrid::streamOutToCPU(UINT nStreamOutParts, const SgVec3f& posCamera, float radius, bool useParts)
{
	UINT nStreamedBlocks = streamOutToCPUPass0GPU(nStreamOutParts, posCamera, radius, useParts);
	if(nStreamedBlocks > 0)
	{
		streamOutToCPUPass1CPU(nStreamedBlocks);
	}
	return nStreamedBlocks;
}

UINT NuiHashingChunkGrid::streamOutToCPUPass0GPU(UINT nStreamOutParts, const SgVec3f& posCamera, float radius, bool useParts )
{
	if(!m_pHashingSDF)
		return 0;

	m_pHashingSDF->resetHashBucketMutexBuffer();

	// Get the kernel
	cl_kernel pass1 = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_GPU_TO_CPU_PASS1);
	assert(pass1);
	if (!pass1)
	{
		NUI_ERROR("Get kernel 'E_HASHING_GPU_TO_CPU_PASS1' failed!\n");
		return 0;
	}

	cl_kernel pass2 = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_GPU_TO_CPU_PASS2);
	assert(pass2);
	if (!pass2)
	{
		NUI_ERROR("Get kernel 'E_HASHING_GPU_TO_CPU_PASS2' failed!\n");
		return 0;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	//clearSDFBlockCounter
	UINT nSDFBlockDescs = 0;
	err = clEnqueueWriteBuffer(
		queue,
		m_SDFBlockCounterCL,
		CL_FALSE,//blocking
		0,
		sizeof(cl_uint),
		&nSDFBlockDescs,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	//-------------------------------------------------------
	// Pass 1: Find all SDFBlocks that have to be transfered
	//-------------------------------------------------------

	const NuiHashParams& hashParams = m_pHashingSDF->getParams();
	unsigned int threadsPerPart = (hashParams.m_hashNumBuckets*HASH_BUCKET_SIZE + nStreamOutParts - 1) / nStreamOutParts;
	if (!useParts) threadsPerPart = hashParams.m_hashNumBuckets*HASH_BUCKET_SIZE;

	if(0 == threadsPerPart)
		return nSDFBlockDescs;

	UINT start = m_currentPart*threadsPerPart;
	cl_mem hashCL = m_pHashingSDF->getHashCL();
	cl_mem heapCL = m_pHashingSDF->getHeapCL();
	cl_mem heapCountCL = m_pHashingSDF->getHeapCountCL();
	cl_mem hashBucketMutexCL = m_pHashingSDF->getHashBucketMutexCL();

	cl_uint idx = 0;
	err = clSetKernelArg(pass1, idx++, sizeof(cl_mem), &hashCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_mem), &heapCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_mem), &heapCountCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_mem), &hashBucketMutexCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_uint), &hashParams.m_hashNumBuckets);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_uint), &hashParams.m_hashMaxCollisionLinkedListSize);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_float), &hashParams.m_virtualVoxelSize);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_uint), &start);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_float), &radius);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_float) * 3, posCamera.getValue());
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_mem), &m_SDFBlockCounterCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_mem), &m_SDFBlockDescOutputCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[1] = { threadsPerPart };
	err = clEnqueueNDRangeKernel(
		queue,
		pass1,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	// Read Block count
	err = clEnqueueReadBuffer(
		queue,
		m_SDFBlockCounterCL,
		CL_TRUE,//blocking
		0,
		sizeof(cl_uint),
		&nSDFBlockDescs,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	if (useParts) m_currentPart = (m_currentPart+1) % nStreamOutParts;

	if (nSDFBlockDescs != 0) {
		//std::cout << "SDFBlocks streamed out: " << nSDFBlockDescs << std::endl;

		//-------------------------------------------------------
		// Pass 2: Copy SDFBlocks to output buffer
		//-------------------------------------------------------
		cl_mem SDFBlocksCL = m_pHashingSDF->getSDFBlocksCL();
		idx = 0;
		err = clSetKernelArg(pass2, idx++, sizeof(cl_mem), &SDFBlocksCL);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pass2, idx++, sizeof(cl_mem), &m_SDFBlockCounterCL);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pass2, idx++, sizeof(cl_mem), &m_SDFBlockOutputCL);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pass2, idx++, sizeof(cl_mem), &m_SDFBlockDescOutputCL);
		NUI_CHECK_CL_ERR(err);

		size_t local_ws[1] = {SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE};
		kernelGlobalSize[0] *= local_ws[0];
		err = clEnqueueNDRangeKernel(
			queue,
			pass2,
			1,
			nullptr,
			kernelGlobalSize,
			local_ws,
			0,
			NULL,
			NULL
			);
		NUI_CHECK_CL_ERR(err);

		// Map buffers
		NuiCLHashEntry* SDFBlockDescOutputMapper = (NuiCLHashEntry*)clEnqueueMapBuffer(
			queue,
			m_SDFBlockDescOutputCL,
			CL_TRUE,//blocking
			CL_MAP_READ,
			0,
			nSDFBlockDescs * sizeof(NuiCLHashEntry),
			0,
			NULL,
			NULL,
			&err
			);
		NUI_CHECK_CL_ERR(err);
		memcpy_s(m_SDFBlockDescOutput, nSDFBlockDescs * sizeof(NuiCLSDFBlockDesc), SDFBlockDescOutputMapper, nSDFBlockDescs * sizeof(NuiCLHashEntry));
		err = clEnqueueUnmapMemObject(
			queue,
			m_SDFBlockDescOutputCL,
			(void*)SDFBlockDescOutputMapper,
			0,
			NULL,
			NULL);
		NUI_CHECK_CL_ERR(err);
		
		NuiCLSDFBlock* SDFBlockOutputMapper = (NuiCLSDFBlock*)clEnqueueMapBuffer(
			queue,
			m_SDFBlockOutputCL,
			CL_TRUE,//blocking
			CL_MAP_READ,
			0,
			nSDFBlockDescs * sizeof(NuiCLSDFBlock),
			0,
			NULL,
			NULL,
			&err
			);
		NUI_CHECK_CL_ERR(err);
		memcpy(m_SDFBlockOutput, SDFBlockOutputMapper, nSDFBlockDescs * sizeof(NuiCLSDFBlock));
		err = clEnqueueUnmapMemObject(
			queue,
			m_SDFBlockOutputCL,
			(void*)SDFBlockOutputMapper,
			0,
			NULL,
			NULL);
		NUI_CHECK_CL_ERR(err);
	}

	return nSDFBlockDescs;
}

bool NuiHashingChunkGrid::isValidChunk(const SgVec3i& chunk) const	{
	if(chunk[0] < m_config.m_minGridPos[0] || chunk[1] < m_config.m_minGridPos[1] || chunk[2] < m_config.m_minGridPos[2]) return false;
	if(chunk[0] > m_config.m_maxGridPos[0] || chunk[1] > m_config.m_maxGridPos[1] || chunk[2] > m_config.m_maxGridPos[2]) return false;

	return true;
}

UINT NuiHashingChunkGrid::linearizeChunkPos(const SgVec3i& chunkPos) const {
	SgVec3ui p = chunkPos-m_config.m_minGridPos;

	return  p[2] * m_config.m_gridDimensions[0] * m_config.m_gridDimensions[1] +
		p[1] * m_config.m_gridDimensions[0] +
		p[0];
}

void NuiHashingChunkGrid::streamOutToCPUPass1CPU(UINT nStreamedBlocks)
{
	const NuiHashParams& hashParams = m_pHashingSDF->getParams();

	for (unsigned int i = 0; i < nStreamedBlocks; i++)
	{
		SgVec3i pos;
		pos[0] = m_SDFBlockDescOutput[i].m_entry.pos[0];
		pos[1] = m_SDFBlockDescOutput[i].m_entry.pos[1];
		pos[2] = m_SDFBlockDescOutput[i].m_entry.pos[2];
		//vec3f posWorld = VoxelUtilHelper::SDFBlockToWorld(pos);
		SgVec3f posWorld = SgVec3f(pos*SDF_BLOCK_SIZE)*hashParams.m_virtualVoxelSize;
		SgVec3i chunk = worldToChunks(posWorld);

		if (!isValidChunk(chunk)) {
			std::cout << "Chunk out of bounds" << std::endl;
			continue;
		}

		unsigned int index = linearizeChunkPos(chunk);

		if (m_grid[index] == NULL) // Allocate memory for chunk
		{
			m_grid[index] = new NuiChunkDesc(m_config.m_initialChunkDescListSize);
		}

		// Add element
		m_grid[index]->addSDFBlock(m_SDFBlockDescOutput[i], m_SDFBlockOutput[i]);
		m_bitMask.setBit(index);
	}
}

SgVec3f NuiHashingChunkGrid::chunkToWorld(const SgVec3i& posChunk) const
{
	SgVec3f res;
	res[0] = posChunk[0]*m_config.m_voxelExtends[0];
	res[1] = posChunk[1]*m_config.m_voxelExtends[1];
	res[2] = posChunk[2]*m_config.m_voxelExtends[2];

	return res;
}

float NuiHashingChunkGrid::getGridRadiusInMeter() const
{
	SgVec3f minPos = chunkToWorld(m_config.m_minGridPos)-m_config.m_voxelExtends/2.0f;
	SgVec3f maxPos = chunkToWorld(m_config.m_maxGridPos)+m_config.m_voxelExtends/2.0f;

	return (minPos-maxPos).length()/2.0f;
}

void NuiHashingChunkGrid::streamInToGPUAll()
{
	unsigned int nStreamedBlocks = 1;
	while (nStreamedBlocks != 0) {
		nStreamedBlocks = streamInToGPU(chunkToWorld(SgVec3i(0, 0, 0)), 1.1f*getGridRadiusInMeter(), true);
	}
}

UINT NuiHashingChunkGrid::streamInToGPUAll(const SgVec3f& posCamera, float radius, bool useParts )
{
	unsigned int nStreamedBlocksSum = 0;
	unsigned int nBlock = 1;
	while (nBlock != 0) // Should not be necessary
	{
		nBlock = streamInToGPU(posCamera, radius, useParts);
		nStreamedBlocksSum += nBlock;
	}
	return nStreamedBlocksSum;
}

void NuiHashingChunkGrid::streamInToGPUChunk(const SgVec3i& chunkPos )
{
	unsigned int nStreamedBlocks = 1;
	while (nStreamedBlocks != 0) // Should not be necessary
	{
		nStreamedBlocks = streamInToGPU(chunkToWorld(chunkPos), 1.1f*getChunkRadiusInMeter(), true);
	}
}

void NuiHashingChunkGrid::streamInToGPUChunkNeighborhood(const SgVec3i& chunkPos, int kernelRadius )
{
	SgVec3i startChunk = SgVec3i(std::max(chunkPos[0]-kernelRadius, m_config.m_minGridPos[0]), std::max(chunkPos[1]-kernelRadius, m_config.m_minGridPos[1]), std::max(chunkPos[2]-kernelRadius, m_config.m_minGridPos[2]));
	SgVec3i endChunk = SgVec3i(std::min(chunkPos[0]+kernelRadius, m_config.m_maxGridPos[0]), std::min(chunkPos[1]+kernelRadius, m_config.m_maxGridPos[1]), std::min(chunkPos[2]+kernelRadius, m_config.m_maxGridPos[2]));

	for (int x = startChunk[0]; x<endChunk[0]; x++) {
		for (int y = startChunk[1]; y<endChunk[1]; y++) {
			for (int z = startChunk[2]; z<endChunk[2]; z++) {
				streamInToGPUChunk(SgVec3i(x, y, z));
			}
		}
	}
}

UINT NuiHashingChunkGrid::streamInToGPU(const SgVec3f& posCamera, float radius, bool useParts)
{
	UINT nStreamedBlocks = streamInToGPUPass0CPU(posCamera, radius, useParts);
	if(nStreamedBlocks > 0)
		streamInToGPUPass1GPU(nStreamedBlocks);

	return nStreamedBlocks;
}

SgVec3i	NuiHashingChunkGrid::meterToNumberOfChunksCeil(float f) const {
	return SgVec3i((int)ceil(f/m_config.m_voxelExtends[0]), (int)ceil(f/m_config.m_voxelExtends[1]), (int)ceil(f/m_config.m_voxelExtends[2]));
}

SgVec3i NuiHashingChunkGrid::delinearizeChunkIndex(unsigned int idx)	{
	unsigned int x = idx % m_config.m_gridDimensions[0];
	unsigned int y = (idx % (m_config.m_gridDimensions[0] * m_config.m_gridDimensions[1])) / m_config.m_gridDimensions[0];
	unsigned int z = idx / (m_config.m_gridDimensions[0] * m_config.m_gridDimensions[1]);

	return m_config.m_minGridPos+SgVec3i(x,y,z);
}

bool NuiHashingChunkGrid::isChunkInSphere(const SgVec3i& chunk, const SgVec3f& center, float radius) const {
	SgVec3f posWorld = chunkToWorld(chunk);
	SgVec3f offset = m_config.m_voxelExtends/2.0f;

	for (int x = -1; x<=1; x+=2)	{
		for (int y = -1; y<=1; y+=2)	{
			for (int z = -1; z<=1; z+=2)	{
				SgVec3f p = SgVec3f(posWorld[0]+x*offset[0], posWorld[1]+y*offset[1], posWorld[2]+z*offset[2]);
				float d = (p-center).length();

				if (d > radius) {
					return false;
				}
			}
		}
	}

	return true;
}

UINT NuiHashingChunkGrid::streamInToGPUPass0CPU( const SgVec3f& posCamera, float radius, bool useParts )
{
	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	const unsigned int blockSize = sizeof(NuiCLSDFBlock)/sizeof(int);
	const unsigned int descSize = sizeof(NuiCLSDFBlockDesc)/sizeof(int);

	SgVec3i camChunk = worldToChunks(posCamera);
	SgVec3i chunkRadius = meterToNumberOfChunksCeil(radius);
	SgVec3i startChunk = SgVec3i(std::max(camChunk[0]-chunkRadius[0], m_config.m_minGridPos[0]), std::max(camChunk[1]-chunkRadius[1], m_config.m_minGridPos[1]), std::max(camChunk[2]-chunkRadius[2], m_config.m_minGridPos[2]));
	SgVec3i endChunk = SgVec3i(std::min(camChunk[0]+chunkRadius[0], m_config.m_maxGridPos[0]), std::min(camChunk[1]+chunkRadius[1], m_config.m_maxGridPos[1]), std::min(camChunk[2]+chunkRadius[2], m_config.m_maxGridPos[2]));

	unsigned int nSDFBlocks = 0;
	for (int x = startChunk[0]; x <= endChunk[0]; x++) {
		for (int y = startChunk[1]; y <= endChunk[1]; y++) {
			for (int z = startChunk[2]; z <= endChunk[2]; z++) {

				unsigned int index = linearizeChunkPos(SgVec3i(x, y, z));
				if (m_grid[index] != NULL && m_grid[index]->isStreamedOut()) // As been allocated and has streamed out blocks
				{
					if (isChunkInSphere(delinearizeChunkIndex(index), posCamera, radius)) // Is in camera range
					{
						unsigned int nBlock = m_grid[index]->getNElements();
						if (nBlock > m_maxNumberOfSDFBlocksIntegrateFromGlobalHash) {
							throw ("not enough memory allocated for intermediate GPU buffer");
						}
						// Copy data to GPU
						NuiCLHashEntry* SDFBlockDescOutputMapper = (NuiCLHashEntry*)clEnqueueMapBuffer(
							queue,
							m_SDFBlockDescOutputCL,
							CL_TRUE,//blocking
							CL_MAP_WRITE,
							0,
							nBlock * sizeof(NuiCLHashEntry),
							0,
							NULL,
							NULL,
							&err
							);
						NUI_CHECK_CL_ERR(err);
						memcpy_s(SDFBlockDescOutputMapper, nBlock * sizeof(NuiCLHashEntry), &(m_grid[index]->getSDFBlockDescs()[0]), nBlock * sizeof(NuiCLSDFBlockDesc));
						err = clEnqueueUnmapMemObject(
							queue,
							m_SDFBlockDescOutputCL,
							(void*)SDFBlockDescOutputMapper,
							0,
							NULL,
							NULL);
						NUI_CHECK_CL_ERR(err);

						NuiCLSDFBlock* SDFBlockOutputMapper = (NuiCLSDFBlock*)clEnqueueMapBuffer(
							queue,
							m_SDFBlockOutputCL,
							CL_TRUE,//blocking
							CL_MAP_WRITE,
							0,
							nBlock * sizeof(NuiCLSDFBlock),
							0,
							NULL,
							NULL,
							&err
							);
						NUI_CHECK_CL_ERR(err);
						memcpy(SDFBlockOutputMapper, &(m_grid[index]->getSDFBlocks()[0]), nBlock * sizeof(NuiCLSDFBlock));
						err = clEnqueueUnmapMemObject(
							queue,
							m_SDFBlockOutputCL,
							(void*)SDFBlockOutputMapper,
							0,
							NULL,
							NULL);
						NUI_CHECK_CL_ERR(err);

						// Remove data from CPU
						m_grid[index]->clear();
						m_bitMask.resetBit(index);

						nSDFBlocks += nBlock;

						if (useParts) return nSDFBlocks; // only in one chunk per frame
					}
				}
			}
		}
	}
	return nSDFBlocks;
}

void NuiHashingChunkGrid::streamInToGPUPass1GPU(UINT nStreamedBlocks)
{
	// Get the kernel
	cl_kernel pass1 = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_CPU_TO_GPU_PASS1);
	assert(pass1);
	if (!pass1)
	{
		NUI_ERROR("Get kernel 'E_HASHING_CPU_TO_GPU_PASS1' failed!\n");
		return;
	}

	cl_kernel pass2 = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_CPU_TO_GPU_PASS2);
	assert(pass2);
	if (!pass2)
	{
		NUI_ERROR("Get kernel 'E_HASHING_CPU_TO_GPU_PASS2' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	//-------------------------------------------------------
	// Pass 1: Alloc memory for chunks
	//-------------------------------------------------------
	const NuiHashParams& hashParams = m_pHashingSDF->getParams();
	cl_mem hashCL = m_pHashingSDF->getHashCL();
	cl_mem heapCL = m_pHashingSDF->getHeapCL();
	cl_mem heapCountCL = m_pHashingSDF->getHeapCountCL();

	cl_uint idx = 0;
	err = clSetKernelArg(pass1, idx++, sizeof(cl_mem), &heapCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_mem), &heapCountCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_mem), &hashCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_uint), &hashParams.m_hashNumBuckets);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_uint), &hashParams.m_hashMaxCollisionLinkedListSize);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass1, idx++, sizeof(cl_mem), &m_SDFBlockDescInputCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[1] = { nStreamedBlocks };
	err = clEnqueueNDRangeKernel(
		queue,
		pass1,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	//-------------------------------------------------------
	// Pass 2: Initialize corresponding SDFBlocks
	//-------------------------------------------------------
	cl_mem SDFBlocksCL = m_pHashingSDF->getSDFBlocksCL();
	idx = 0;
	err = clSetKernelArg(pass2, idx++, sizeof(cl_mem), &heapCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass2, idx++, sizeof(cl_mem), &heapCountCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass2, idx++, sizeof(cl_mem), &SDFBlocksCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(pass2, idx++, sizeof(cl_mem), &m_SDFBlockInputCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t local_ws[1] = {SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE};
	kernelGlobalSize[0] *= local_ws[0];
	err = clEnqueueNDRangeKernel(
		queue,
		pass2,
		1,
		nullptr,
		kernelGlobalSize,
		local_ws,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
}
