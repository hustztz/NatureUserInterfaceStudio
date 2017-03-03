#include "NuiKinfuOpenCLFeedbackFrame.h"

#include "NuiKinfuOpenCLFrame.h"
#include "NuiKinfuOpenCLCameraState.h"

#include "Foundation/NuiDebugMacro.h"
#include "Shape/NuiCLMappableData.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"

#include "assert.h"

NuiKinfuOpenCLFeedbackFrame::NuiKinfuOpenCLFeedbackFrame(UINT nWidth, UINT nHeight)
	: m_nWidth(0)
	, m_nHeight(0)
	, m_verticesCL(NULL)
	, m_normalsCL(NULL)
{
	AcquireBuffers(nWidth, nHeight);
}

NuiKinfuOpenCLFeedbackFrame::~NuiKinfuOpenCLFeedbackFrame()
{
	ReleaseBuffers();
}

void	NuiKinfuOpenCLFeedbackFrame::AcquireBuffers(UINT nWidth, UINT nHeight)
{
	if(nWidth == m_nWidth && nHeight == m_nHeight)
	{
		return;
	}

	ReleaseBuffers();

	m_nWidth = nWidth;
	m_nHeight = nHeight;

	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();
	m_verticesCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_nWidth*m_nHeight*3*sizeof(cl_float), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_normalsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_nWidth*m_nHeight*3*sizeof(cl_float), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}

void	NuiKinfuOpenCLFeedbackFrame::ReleaseBuffers()
{
	if (m_verticesCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_verticesCL);
		NUI_CHECK_CL_ERR(err);
		m_verticesCL = NULL;
	}
	if (m_normalsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_normalsCL);
		NUI_CHECK_CL_ERR(err);
		m_normalsCL = NULL;
	}
}

void	NuiKinfuOpenCLFeedbackFrame::UpdateBuffers(NuiKinfuFrame* pFrame, NuiKinfuCameraState* pCameraState)
{
	if(!pFrame)
		return;
	NuiKinfuOpenCLFrame* pCLFrame = dynamic_cast<NuiKinfuOpenCLFrame*>(pFrame);
	if(!pCLFrame)
		return;

	if(!pCameraState)
		return;
	NuiKinfuOpenCLCameraState* pCLCamera = dynamic_cast<NuiKinfuOpenCLCameraState*>(pCameraState->GetDeviceCache());
	if(!pCLCamera)
		return;

	if(pCLFrame->GetWidth() != m_nWidth || pCLFrame->GetHeight() != m_nHeight)
		return;

	Vertex2Normal(pCLFrame->GetVertexBuffer(), pCLFrame->GetDepthThreshold());
	TransformBuffers(pCLFrame->GetVertexBuffer(), pCLCamera->GetCameraTransformBuffer());
}


void NuiKinfuOpenCLFeedbackFrame::Vertex2Normal(cl_mem verticesCL, float depth_threshold)
{
	if(!verticesCL)
		return;

	// Get the kernel
	cl_kernel normalEstKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_ESTIMATE_NORMALS_SIMPLE);
	assert(normalEstKernel);
	if (!normalEstKernel)
	{
		NUI_ERROR("Get kernel 'E_ESTIMATE_NORMALS_SIMPLE' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(normalEstKernel, idx++, sizeof(cl_mem), &verticesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(normalEstKernel, idx++, sizeof(cl_mem), &m_normalsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(normalEstKernel, idx++, sizeof(float), &depth_threshold);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { m_nWidth, m_nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		normalEstKernel,
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


void NuiKinfuOpenCLFeedbackFrame::TransformBuffers(cl_mem verticesCL, cl_mem transformCL)
{
	if(!verticesCL || !transformCL)
		return;

	// Get the kernel
	cl_kernel transformKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_TRANSFORM_MAPS);
	assert(transformKernel);
	if (!transformKernel)
	{
		NUI_ERROR("Get kernel 'E_TRANSFORM_MAPS' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	cl_uint idx = 0;
	err = clSetKernelArg(transformKernel, idx++, sizeof(cl_mem), &verticesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(transformKernel, idx++, sizeof(cl_mem), &m_normalsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(transformKernel, idx++, sizeof(cl_mem), &m_verticesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(transformKernel, idx++, sizeof(cl_mem), &m_normalsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(transformKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[2] = { m_nWidth, m_nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		transformKernel,
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


bool NuiKinfuOpenCLFeedbackFrame::VerticesToMappablePosition(NuiCLMappableData* pMappableData)
{
	assert(pMappableData);
	if(!pMappableData)
		return false;

	if(!m_verticesCL)
		return false;

	const UINT nPointsNum = m_nWidth * m_nHeight;

	pMappableData->SetBoundingBox(SgVec3f(-256.0f / 370.0f, -212.0f / 370.0f, 0.4f),
		SgVec3f((m_nWidth-256.0f) / 370.0f, (m_nHeight-212.0f) / 370.0f, 4.0f));

	NuiMappableAccessor::asVectorImpl(pMappableData->TriangleIndices())->data().clear();
	NuiMappableAccessor::asVectorImpl(pMappableData->WireframeIndices())->data().clear();

	std::vector<unsigned int>& clPointIndices =
		NuiMappableAccessor::asVectorImpl(pMappableData->PointIndices())->data();
	if(clPointIndices.size() != nPointsNum)
	{
		clPointIndices.resize(nPointsNum);
		for (UINT i = 0; i < nPointsNum; ++i)
		{
			clPointIndices[i] = i;
		}
		pMappableData->SetIndexingDirty(true);
	}

	if( nPointsNum != pMappableData->PositionStream().size() )
	{
		NuiMappableAccessor::asVectorImpl(pMappableData->PositionStream())->data().resize(nPointsNum);
	}

	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	// 
	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	cl_mem positionsGL = NuiOpenCLBufferFactory::asPosition3fBufferCL(pMappableData->PositionStream());
	// Acquire OpenGL objects before use
	cl_mem glObjs[] = {
		positionsGL
	};

	openclutil::enqueueAcquireHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

	err = clEnqueueCopyBuffer(
		queue,
		m_verticesCL,
		positionsGL,
		0,
		0,
		nPointsNum * 3 * sizeof(float),
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	// Release OpenGL objects
	openclutil::enqueueReleaseHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

	pMappableData->SetStreamDirty(true);

	return true;
}

bool	NuiKinfuOpenCLFeedbackFrame::BufferToMappableTexture(NuiCLMappableData* pMappableData, TrackerBufferType bufferType)
{
	assert(pMappableData);
	if(!pMappableData)
		return false;

	cl_mem bufferCL = m_verticesCL;
	switch (bufferType)
	{
	case eTracker_Vertices:
		bufferCL = m_verticesCL;
		break;
	case eTracker_Normals:
		bufferCL = m_normalsCL;
		break;
	default:
		break;
	}

	if(!bufferCL)
		return false;

	// Get the kernel
	cl_kernel rgbaKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_FLOAT3_TO_TEXTURE);
	assert(rgbaKernel);
	if (!rgbaKernel)
	{
		NUI_ERROR("Get kernel 'E_FLOAT3_TO_RGBA' failed!\n");
		return false;
	}

	if( m_nWidth != pMappableData->ColorTex().width() || m_nHeight != pMappableData->ColorTex().height())
	{
		NuiTextureMappableAccessor::updateImpl(
			pMappableData->ColorTex(),
			m_nWidth,
			m_nHeight,
			NULL
			);
	}
	cl_mem texGL = NuiOpenCLBufferFactory::asTexture2DCL(pMappableData->ColorTex());

	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	// 
	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	// Acquire OpenGL objects before use
	cl_mem glObjs[] = {
		texGL
	};

	openclutil::enqueueAcquireHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(rgbaKernel, idx++, sizeof(cl_mem), &bufferCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(rgbaKernel, idx++, sizeof(cl_mem), &texGL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { m_nWidth, m_nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		rgbaKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	// Release OpenGL objects
	openclutil::enqueueReleaseHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

	return true;
}

