#include "NuiHashingChunkGrid.h"
#include "NuiHashingSDF.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

NuiHashingChunkGrid::NuiHashingChunkGrid(NuiHashingSDF* pSDF)
	: m_pHashingSDF(pSDF)
	, m_currentPart(0)
	, m_maxNumberOfSDFBlocksIntegrateFromGlobalHash(100000)
{
}

NuiHashingChunkGrid::~NuiHashingChunkGrid()
{
}

void NuiHashingChunkGrid::AcquireBuffers()
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	m_SDFBlockDescOutputCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(NuiSDFBlockDesc)*m_maxNumberOfSDFBlocksIntegrateFromGlobalHash, NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_SDFBlockDescInputCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(NuiSDFBlockDesc)*m_maxNumberOfSDFBlocksIntegrateFromGlobalHash, NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_SDFBlockOutputCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(NuiSDFBlock)*m_maxNumberOfSDFBlocksIntegrateFromGlobalHash, NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_SDFBlockInputCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(NuiSDFBlock)*m_maxNumberOfSDFBlocksIntegrateFromGlobalHash, NULL, &err);
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
}

const SgVec3i& NuiHashingChunkGrid::worldToChunks(const SgVec3f& posWorld, const SgVec3f& voxelExtends) const {
	SgVec3f p;
	p[0] = posWorld[0] / voxelExtends[0];
	p[1] = posWorld[1] / voxelExtends[1];
	p[2] = posWorld[2] / voxelExtends[2];

	SgVec3f s;
	s[0] = (float)sign(p[0]);
	s[1] = (float)sign(p[1]);
	s[2] = (float)sign(p[2]);

	return SgVec3i(p+s*0.5f);
}

void NuiHashingChunkGrid::streamOutToCPUAll(UINT nStreamOutParts, const SgVec3i& minGridPos, const SgVec3f& voxelExtends)
{
	unsigned int nStreamedBlocksSum = 1;
	while(nStreamedBlocksSum != 0) {
		nStreamedBlocksSum = 0;
		for (unsigned int i = 0; i < nStreamOutParts; i++) {
			unsigned int nStreamedBlocks = streamOutToCPU(
				nStreamOutParts,
				worldToChunks((minGridPos-SgVec3i(1, 1, 1)), voxelExtends),
				0.0f,
				true);
			nStreamedBlocksSum += nStreamedBlocks;
		}
	}
}

UINT NuiHashingChunkGrid::streamOutToCPU(UINT nStreamOutParts, const SgVec3f& posCamera, float radius, bool useParts)
{
	UINT nStreamedBlocks = streamOutToCPUPass0GPU(nStreamOutParts, posCamera, radius, useParts);
	streamOutToCPUPass1CPU(nStreamedBlocks);
}

UINT NuiHashingChunkGrid::streamOutToCPUPass0GPU(UINT nStreamOutParts, const SgVec3f& posCamera, float radius, bool useParts )
{
	if(!m_pHashingSDF)
		return 0;

	m_pHashingSDF->resetHashBucketMutexBuffer();

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

	const unsigned int hashNumBuckets = m_sceneRepHashSDF->getHashParams().m_hashNumBuckets;
	const unsigned int hashBucketSize = m_sceneRepHashSDF->getHashParams().m_hashBucketSize;

	//-------------------------------------------------------
	// Pass 1: Find all SDFBlocks that have to be transfered
	//-------------------------------------------------------

	unsigned int threadsPerPart = (hashNumBuckets*hashBucketSize + nStreamOutParts - 1) / nStreamOutParts;
	if (!useParts) threadsPerPart = hashNumBuckets*hashBucketSize;

	UINT start = m_currentPart*threadsPerPart;
	integrateFromGlobalHashPass1CUDA(m_sceneRepHashSDF->getHashParams(), m_sceneRepHashSDF->getHashData(), threadsPerPart, start, radius, MatrixConversion::toCUDA(posCamera), d_SDFBlockCounter, d_SDFBlockDescOutput);

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

		integrateFromGlobalHashPass2CUDA(m_sceneRepHashSDF->getHashParams(), m_sceneRepHashSDF->getHashData(), threadsPerPart, d_SDFBlockDescOutput, (Voxel*)d_SDFBlockOutput, nSDFBlockDescs);


		MLIB_CUDA_SAFE_CALL(cudaMemcpy(h_SDFBlockDescOutput, d_SDFBlockDescOutput, sizeof(SDFBlockDesc)*nSDFBlockDescs, cudaMemcpyDeviceToHost));
		MLIB_CUDA_SAFE_CALL(cudaMemcpy(h_SDFBlockOutput, d_SDFBlockOutput, sizeof(SDFBlock)*nSDFBlockDescs, cudaMemcpyDeviceToHost));
	}

	return nSDFBlockDescs;
}
