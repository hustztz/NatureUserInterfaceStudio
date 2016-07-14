#include "NuiHashingVolume.h"

#include "NuiHashingChunkGrid.h"
#include "NuiKinfuTransform.h"

#include "Foundation/NuiMatrixUtilities.h"
#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"

NuiHashingVolume::NuiHashingVolume()
	: m_integration_metric_threshold(0.15f)
{
	m_pChunkGrid = new NuiHashingChunkGrid(&m_sdfData, m_gridDimensions, m_minGridPos, m_voxelExtends, m_initialChunkListSize);
	reset();
}

NuiHashingVolume::~NuiHashingVolume()
{
	SafeDelete(m_pChunkGrid);
}

void	NuiHashingVolume::reset()
{
	m_sdfData.reset();
	if(m_pChunkGrid)
		m_pChunkGrid->reset();
	m_lastIntegrationRotation.setIdentity();
	m_lastIntegrationTranslation = Vector3f::Zero();
}

void NuiHashingVolume::raycastRender(
	NuiHashingSDF* pSDF,
	cl_mem cameraParamsCL,
	cl_mem transformCL,
	cl_mem verticesCL,
	cl_mem normalsCL,
	float rayIncrement,
	float thresSampleDist,
	float thresDist,
	float minDepth,
	float maxDepth,
	UINT nWidth, UINT nHeight
	)
{
	// Get the kernel
	cl_kernel raycastKernel = nullptr;
	NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_RAYCAST_SDF);
	assert(raycastKernel);
	if (!raycastKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_RAYCAST_SDF' failed!\n");
		return;
	}

	cl_mem hashCL = pSDF->getHashCL();
	cl_mem SDFBlocksCL = pSDF->getSDFBlocksCL();
	const NuiHashParams& hashParams = pSDF->getParams();

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &verticesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &normalsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), NULL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &hashCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &SDFBlocksCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), &hashParams.m_virtualVoxelSize);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), &hashParams.m_hashNumBuckets);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_uint), &hashParams.m_hashMaxCollisionLinkedListSize);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), &thresSampleDist);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_uint), &thresDist);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_uint), &rayIncrement);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_uint), &minDepth);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_uint), &maxDepth);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[2] = { nWidth, nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		raycastKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
}

void	NuiHashingVolume::incrementVolume(
	cl_mem floatDepthsCL,
	cl_mem colorsCL,
	cl_mem normalsCL,
	cl_mem cameraParamsCL,
	const NuiKinfuTransform& currPos,
	UINT nWidth, UINT nHeight
	)
{
	m_sdfData.integrate(nWidth, nHeight, floatDepthsCL, colorsCL, cameraParamsCL, currPos.getTransformCL(), m_pChunkGrid ? m_pChunkGrid->getBitMaskCL() : NULL);
	m_lastIntegrationRotation = currPos.getRotation();
	m_lastIntegrationTranslation = currPos.getTranslation();
}

bool	NuiHashingVolume::evaluateVolume(
	cl_mem floatDepthsCL,
	cl_mem colorsCL,
	cl_mem normalsCL,
	cl_mem renderVertices,
	cl_mem renderNormals,
	cl_mem cameraParamsCL,
	const NuiKinfuTransform& currPos,
	UINT nWidth, UINT nHeight
	)
{
	///////////////////////////////////////////////////////////////////////////////////////////
	// Integration check - We do not integrate volume if camera does not move.  
	float rnorm = NuiMatrixUtilities::rodrigues2(currPos.getRotation().inverse() * m_lastIntegrationRotation).norm();
	float tnorm = (currPos.getTranslation() - m_lastIntegrationTranslation).norm();
	const float alpha = 1.f;
	bool integrate = (rnorm + alpha * tnorm)/2 >= m_integration_metric_threshold;
	// Integrate
	if (integrate)
	{
		incrementVolume(floatDepthsCL, colorsCL, normalsCL, cameraParamsCL, currPos, nWidth, nHeight);
	}
	raycastRender(
		&m_sdfData,
		cameraParamsCL,
		currPos.getTransformCL(),
		renderVertices,
		renderNormals,
		m_rayIncrement,
		m_thresSampleDist,
		m_thresDist,
		m_minDepth,
		m_maxDepth,
		nWidth, nHeight);
	return integrate;
}