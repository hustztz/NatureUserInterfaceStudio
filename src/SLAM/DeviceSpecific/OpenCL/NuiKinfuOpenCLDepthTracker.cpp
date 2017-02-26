#include "NuiKinfuOpenCLDepthTracker.h"

#include "NuiKinfuOpenCLFrame.h"
#include "NuiKinfuOpenCLScene.h"
#include "NuiKinfuOpenCLCameraState.h"

#include "Foundation/NuiDebugMacro.h"
#include "Shape/NuiCLMappableData.h"

#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

#include <iostream>
#include <boost/smart_ptr.hpp>

#define KINFU_ICP_CORESPS_NUM 29
#define WORK_GROUP_SIZE 128

using Eigen::AngleAxisf;

NuiKinfuOpenCLDepthTracker::NuiKinfuOpenCLDepthTracker(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight)
	: m_configuration(config)
	, m_gaussianCL(NULL)
	, m_corespsCL(NULL)
	, m_corespsBlocksCL(NULL)
	, m_nWidth(nWidth)
	, m_nHeight(nHeight)
	, m_error(0.0f)
	, m_count(0.0f)
{
	m_iterations = config.iterations;
	m_depthsArrCL.resize(m_iterations.size());
	m_verticesArrCL.resize(m_iterations.size());
	m_normalsArrCL.resize(m_iterations.size());
	m_verticesPrevArrCL.resize(m_iterations.size());
	m_normalsPrevArrCL.resize(m_iterations.size());
	for (UINT i = 0; i < m_iterations.size(); ++i)
	{
		m_depthsArrCL[i] = NULL;
		m_verticesArrCL[i] = NULL;
		m_normalsArrCL[i] = NULL;
		m_verticesPrevArrCL[i] = NULL;
		m_normalsPrevArrCL[i] = NULL;
	}

	AcquireBuffers();
}

NuiKinfuOpenCLDepthTracker::~NuiKinfuOpenCLDepthTracker()
{
	ReleaseBuffers();
}

void NuiKinfuOpenCLDepthTracker::AcquireBuffers()
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	for (UINT i = 0; i < m_iterations.size(); ++i)
	{
		m_depthsArrCL[i] = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_nWidth*m_nHeight*sizeof(cl_float), NULL, &err);
		NUI_CHECK_CL_ERR(err);
		m_verticesArrCL[i] = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, (m_nWidth>>i)*(m_nHeight>>i)*3*sizeof(cl_float), NULL, &err);
		NUI_CHECK_CL_ERR(err);
		m_normalsArrCL[i] = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, (m_nWidth>>i)*(m_nHeight>>i)*3*sizeof(cl_float), NULL, &err);
		NUI_CHECK_CL_ERR(err);
		m_verticesPrevArrCL[i] = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, (m_nWidth>>i)*(m_nHeight>>i)*3*sizeof(cl_float), NULL, &err);
		NUI_CHECK_CL_ERR(err);
		m_normalsPrevArrCL[i] = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, (m_nWidth>>i)*(m_nHeight>>i)*3*sizeof(cl_float), NULL, &err);
		NUI_CHECK_CL_ERR(err);
	}
	UINT nblocks = (UINT)std::ceil( (float)(m_nWidth*m_nHeight) / WORK_GROUP_SIZE);
	m_corespsBlocksCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nblocks*KINFU_ICP_CORESPS_NUM*sizeof(cl_float), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	UINT nItems = (UINT)std::ceil( (float)(nblocks) / WORK_GROUP_SIZE);
	m_corespsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nItems*KINFU_ICP_CORESPS_NUM*sizeof(cl_float), NULL, &err);
	NUI_CHECK_CL_ERR(err);

	GenerateGaussianBuffer();
}

void NuiKinfuOpenCLDepthTracker::ReleaseBuffers()
{
	if (m_gaussianCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_gaussianCL);
		NUI_CHECK_CL_ERR(err);
		m_gaussianCL = NULL;
	}
	for (UINT i = 0; i < m_iterations.size(); ++i)
	{
		if (m_depthsArrCL[i]) {
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_depthsArrCL[i]);
			NUI_CHECK_CL_ERR(err);
			m_depthsArrCL[i] = NULL;
		}
		if (m_verticesArrCL[i]) {
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_verticesArrCL[i]);
			NUI_CHECK_CL_ERR(err);
			m_verticesArrCL[i] = NULL;
		}
		if (m_normalsArrCL[i]) {
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_normalsArrCL[i]);
			NUI_CHECK_CL_ERR(err);
			m_normalsArrCL[i] = NULL;
		}
		if (m_verticesPrevArrCL[i]) {
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_verticesPrevArrCL[i]);
			NUI_CHECK_CL_ERR(err);
			m_verticesPrevArrCL[i] = NULL;
		}
		if (m_normalsPrevArrCL[i]) {
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_normalsPrevArrCL[i]);
			NUI_CHECK_CL_ERR(err);
			m_normalsPrevArrCL[i] = NULL;
		}
	}
	if (m_corespsBlocksCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_corespsBlocksCL);
		NUI_CHECK_CL_ERR(err);
		m_corespsBlocksCL = NULL;
	}
	if (m_corespsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_corespsCL);
		NUI_CHECK_CL_ERR(err);
		m_corespsCL = NULL;
	}
}

bool NuiKinfuOpenCLDepthTracker::log(const std::string& fileName) const
{
	return m_configuration.log(fileName);
}

bool NuiKinfuOpenCLDepthTracker::EvaluateFrame(NuiKinfuFrame* pFrame, NuiKinfuCameraState* pCameraState)
{
	// half sample the input depth maps into the pyramid levels
	if(!pFrame)
		return false;
	NuiKinfuOpenCLFrame* pCLFrame = dynamic_cast<NuiKinfuOpenCLFrame*>(pFrame);
	if(!pCLFrame)
		return false;
	// filter the input depth map
	SmoothDepths(pCLFrame->GetDepthsBuffer());
	SubSampleDepths();

	if(!pCameraState)
		return false;
	NuiKinfuOpenCLCameraState* pCLCamera = dynamic_cast<NuiKinfuOpenCLCameraState*>(pCameraState->GetDeviceCache());
	if(!pCLCamera)
		return false;

	Depth2vertex(pCLCamera->GetCameraParamsBuffer());
	Vertex2Normal();
	pCLFrame->SetNormalsBuffer(&m_normalsArrCL[0]);
	return true;
}

bool NuiKinfuOpenCLDepthTracker::EstimatePose(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint)
{
	return IterativeClosestPoint(pCameraState, hint);
}

void NuiKinfuOpenCLDepthTracker::FeedbackPose(NuiKinfuCameraState* pCameraState)
{
	if(!pCameraState)
		return;
	NuiKinfuOpenCLCameraState* pCLCamera = dynamic_cast<NuiKinfuOpenCLCameraState*>(pCameraState->GetDeviceCache());
	if(!pCLCamera)
		return;
	cl_mem transformCL = pCLCamera->GetCameraTransformBuffer();
	if(!transformCL)
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

	for (UINT i = 0; i < m_iterations.size(); ++i)
	{
		cl_uint idx = 0;
		err = clSetKernelArg(transformKernel, idx++, sizeof(cl_mem), &m_verticesArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(transformKernel, idx++, sizeof(cl_mem), &m_normalsArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(transformKernel, idx++, sizeof(cl_mem), &m_verticesPrevArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(transformKernel, idx++, sizeof(cl_mem), &m_normalsPrevArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(transformKernel, idx++, sizeof(cl_mem), &transformCL);
		NUI_CHECK_CL_ERR(err);

		// Run kernel to calculate
		size_t kernelGlobalSize[2] = { m_nWidth >> i, m_nHeight >> i };
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
}

void	NuiKinfuOpenCLDepthTracker::FeedbackPose(NuiKinfuCameraState* pCameraState, NuiKinfuScene* pScene)
{
	if(!pScene)
		return;
	NuiKinfuOpenCLScene* pCLScene = dynamic_cast<NuiKinfuOpenCLScene*>(pScene);
	if(!pCLScene)
		return;

	if(!pCameraState)
		return;
	NuiKinfuOpenCLCameraState* pCLCamera = dynamic_cast<NuiKinfuOpenCLCameraState*>(pCameraState->GetDeviceCache());
	if(!pCLCamera)
		return;
	const NuiCameraPos& cameraPos = pCameraState->GetCameraPos();
	cl_mem transformCL = pCLCamera->GetCameraTransformBuffer();
	cl_mem cameraParamsCL = pCLCamera->GetCameraParamsBuffer();

	pCLScene->raycastRender(
		m_verticesPrevArrCL[0],
		m_normalsPrevArrCL[0],
		NULL,
		cameraParamsCL,
		transformCL,
		m_nWidth, m_nHeight,
		cameraPos.getSensorDepthMin(), cameraPos.getSensorDepthMax()
		);

	resizePrevsMaps();
}

void NuiKinfuOpenCLDepthTracker::resizePrevsMaps()
{
	// half sample the input depth maps into the pyramid levels
	// Get the kernel
	cl_kernel resizeKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_RESIZE_MAPS);
	assert(resizeKernel);
	if (!resizeKernel)
	{
		NUI_ERROR("Get kernel 'E_RESIZE_MAPS' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	
	// Set kernel arguments
	cl_uint idx = 0;
	size_t kernelGlobalSize[2] = { m_nWidth, m_nHeight };

	for (UINT i = 1; i < m_iterations.size(); ++i)
	{
		// Vertices
		idx = 0;
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), &m_verticesPrevArrCL[i-1]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), &m_normalsPrevArrCL[i-1]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), NULL);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), &m_verticesPrevArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), &m_normalsPrevArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), NULL);
		NUI_CHECK_CL_ERR(err);

		// Run kernel to calculate
		kernelGlobalSize[0] = m_nWidth >> i;
		kernelGlobalSize[1] = m_nHeight >> i;
		err = clEnqueueNDRangeKernel(
			queue,
			resizeKernel,
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
}

void NuiKinfuOpenCLDepthTracker::copyPrevsFrame()
{
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	for (UINT i = 0; i < m_iterations.size(); ++i)
	{
		const UINT nPointsNum = (m_nWidth >> i) * (m_nHeight >> i);

		err = clEnqueueCopyBuffer(
			queue,
			m_verticesArrCL[i],
			m_verticesPrevArrCL[i],
			0,
			0,
			nPointsNum * 3 * sizeof(float),
			0,
			NULL,
			NULL
			);
		NUI_CHECK_CL_ERR(err);

		err = clEnqueueCopyBuffer(
			queue,
			m_normalsArrCL[i],
			m_normalsPrevArrCL[i],
			0,
			0,
			nPointsNum * 3 * sizeof(float),
			0,
			NULL,
			NULL
			);
		NUI_CHECK_CL_ERR(err);
	}
}

void NuiKinfuOpenCLDepthTracker::GenerateGaussianBuffer()
{
	// Get the kernel
	cl_kernel gaussianKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_GENERATE_GAUSSIAN);
	assert(gaussianKernel);
	if (!gaussianKernel)
	{
		NUI_ERROR("Get kernel 'E_GENERATE_GAUSSIAN' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	int gaussianSize = m_configuration.filter_radius*2+1;
	float* guassian_table = new float[gaussianSize];
	for (int i = 0; i < gaussianSize; ++i)
	{
		int x = i - (int)m_configuration.filter_radius;
		guassian_table[i] = exp(-(x * x) * m_configuration.sigma_space2_inv_half);
	}

	m_gaussianCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, gaussianSize*sizeof(float), guassian_table, &err);
	NUI_CHECK_CL_ERR(err);

	SafeDeleteArray(guassian_table);

	// Set kernel arguments
	//cl_uint idx = 0;
	//err = clSetKernelArg(gaussianKernel, idx++, sizeof(cl_mem), &m_gaussianCL);
	//NUI_CHECK_CL_ERR(err);
	//err = clSetKernelArg(gaussianKernel, idx++, sizeof(float), &m_configuration.sigma_space2_inv_half);
	//NUI_CHECK_CL_ERR(err);
	//err = clSetKernelArg(gaussianKernel, idx++, sizeof(UINT), &m_configuration.filter_radius);
	//NUI_CHECK_CL_ERR(err);

	//// Run kernel to calculate 
	//size_t kernelGlobalSize = m_configuration.filter_radius*2+1;
	//err = clEnqueueNDRangeKernel(
	//	queue,
	//	gaussianKernel,
	//	1,
	//	nullptr,
	//	&kernelGlobalSize,
	//	nullptr,
	//	0,
	//	NULL,
	//	NULL
	//	);
	//NUI_CHECK_CL_ERR(err);
}

void NuiKinfuOpenCLDepthTracker::SmoothDepths(cl_mem floatDepthsCL)
{
	assert(m_gaussianCL);
	if(!m_gaussianCL)
		return;

	// Get the kernel
	cl_kernel smoothKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_BILATERAL_FILTER_DEPTH);
	assert(smoothKernel);
	if (!smoothKernel)
	{
		NUI_ERROR("Get kernel 'E_BILATERAL_FILTER' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(smoothKernel, idx++, sizeof(cl_mem), &floatDepthsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(smoothKernel, idx++, sizeof(cl_mem), &m_depthsArrCL[0]);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(smoothKernel, idx++, sizeof(cl_mem), &m_gaussianCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(smoothKernel, idx++, sizeof(UINT), &m_configuration.filter_radius);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(smoothKernel, idx++, sizeof(float), &m_configuration.sigma_depth2_inv_half);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(smoothKernel, idx++, sizeof(float), &m_configuration.depth_threshold);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { m_nWidth, m_nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		smoothKernel,
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

void NuiKinfuOpenCLDepthTracker::SubSampleDepths()
{
	// half sample the input depth maps into the pyramid levels
	// Get the kernel
	cl_kernel pyrDownKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_ESTIMATE_HALF_SAMPLE);
	assert(pyrDownKernel);
	if (!pyrDownKernel)
	{
		NUI_ERROR("Get kernel 'E_ESTIMATE_HALF_SAMPLE' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	const UINT subSampleRadius = 1;
	for (UINT i = 1; i < m_iterations.size(); ++i)
	{
		// Set kernel arguments
		cl_uint idx = 0;
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(cl_mem), &m_depthsArrCL[i-1]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(cl_mem), &m_depthsArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(UINT), &subSampleRadius);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(float), &m_configuration.depth_threshold);
		NUI_CHECK_CL_ERR(err);

		// Run kernel to calculate 
		size_t kernelGlobalSize[2] = { m_nWidth>>i, m_nHeight>>i };
		err = clEnqueueNDRangeKernel(
			queue,
			pyrDownKernel,
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
}

void NuiKinfuOpenCLDepthTracker::Depth2vertex(cl_mem cameraParamsCL)
{
	// Get the kernel
	cl_kernel depth2vertexKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_DEPTH2VERTEX);
	assert(depth2vertexKernel);
	if (!depth2vertexKernel)
	{
		NUI_ERROR("Get kernel 'E_DEPTH2VERTEX' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	for (UINT i = 0; i < m_iterations.size(); ++i)
	{
		int div = 1 << i; 
		// Set kernel arguments
		cl_uint idx = 0;
		err = clSetKernelArg(depth2vertexKernel, idx++, sizeof(cl_mem), &m_depthsArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(depth2vertexKernel, idx++, sizeof(cl_mem), &m_verticesArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(depth2vertexKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(depth2vertexKernel, idx++, sizeof(int), &div);
		NUI_CHECK_CL_ERR(err);

		// Run kernel to calculate 
		size_t kernelGlobalSize[2] = { m_nWidth >> i, m_nHeight >> i };
		err = clEnqueueNDRangeKernel(
			queue,
			depth2vertexKernel,
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
}

void NuiKinfuOpenCLDepthTracker::Vertex2Normal()
{
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

	for (UINT i = 0; i < m_iterations.size(); ++i)
	{
		///////////////////////////////////////
		// vertex2normal
		// Set kernel arguments
		cl_uint idx = 0;
		err = clSetKernelArg(normalEstKernel, idx++, sizeof(cl_mem), &m_verticesArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(normalEstKernel, idx++, sizeof(cl_mem), &m_normalsArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(normalEstKernel, idx++, sizeof(float), &m_configuration.depth_threshold);
		NUI_CHECK_CL_ERR(err);
		
		// Run kernel to calculate 
		size_t kernelGlobalSize[2] = { m_nWidth >> i, m_nHeight >> i };
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
}

///////////////////////////////////////////////////////////////////////////////////////////
// Iterative Closest Point
bool NuiKinfuOpenCLDepthTracker::IterativeClosestPoint(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint)
{
	if(!pCameraState)
		return false;
	NuiKinfuOpenCLCameraState* pCLCamera = dynamic_cast<NuiKinfuOpenCLCameraState*>(pCameraState->GetDeviceCache());
	if(!pCLCamera)
		return false;

	cl_mem cameraParamsCL = pCLCamera->GetCameraParamsBuffer();
	if(!cameraParamsCL)
		return false;

	// Get the kernel
	cl_kernel icpKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_ICP_BLOCK);
	assert(icpKernel);
	if (!icpKernel)
	{
		NUI_ERROR("Get kernel 'E_ICP_BLOCK' failed!\n");
		return false;
	}

	cl_kernel sumKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_ICP_SUM);
	assert(sumKernel);
	if (!sumKernel)
	{
		NUI_ERROR("Get kernel 'E_ICP_SUM' failed!\n");
		return false;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	cl_uint idx = 0;
	size_t kernelGlobalSize[1] = {m_nWidth * m_nHeight};
	size_t local_ws[1] = {WORK_GROUP_SIZE};
	boost::scoped_array<float> corespResult(new float[kernelGlobalSize[0] * KINFU_ICP_CORESPS_NUM / WORK_GROUP_SIZE]);

	Matrix3frm Rcurr;
	Vector3f tcurr;
	if(hint)
	{
		Rcurr = hint->rotation().matrix();
		tcurr = hint->translation().matrix();
	}
	else
	{
		const NuiCameraPos& cameraPos = pCameraState->GetCameraPos();
		Rcurr = cameraPos.getRotation(); // tranform to global coo for ith camera pose
		tcurr = cameraPos.getTranslation();
	}
	cl_mem previousTransform = pCLCamera->GetCameraTransformBuffer();

	/** \brief array with IPC iteration numbers for each pyramid level */
	int LEVELS = (int)m_iterations.size();

	//ScopeTime time("icp-all");
	for (int level_index = LEVELS-1; level_index>=0; --level_index)
	{
		int div = 1 << level_index;
		int iter_num = m_iterations[level_index];
		for (int iter = 0; iter < iter_num; ++iter)
		{
			idx = 0;
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_int), &div);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &m_verticesArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &m_normalsArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(float)*8, Rcurr.data());
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(float), Rcurr.data()+8);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(float)*4, tcurr.data());
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &m_verticesPrevArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &m_normalsPrevArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &previousTransform);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(float), &m_configuration.dist_threshold);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(float), &m_configuration.normal_threshold);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &m_corespsBlocksCL);
			NUI_CHECK_CL_ERR(err);

			// Run kernel to calculate
			kernelGlobalSize[0] = (m_nWidth >> level_index) * (m_nHeight >> level_index);
			err = clEnqueueNDRangeKernel(
				queue,
				icpKernel,
				1,
				nullptr,
				kernelGlobalSize,
				local_ws,
				0,
				NULL,
				NULL
			);
			NUI_CHECK_CL_ERR(err);

			UINT size = (UINT)(std::ceil( (float)kernelGlobalSize[0] / (float)local_ws[0] ));

			idx = 0;
			err = clSetKernelArg(sumKernel, idx++, sizeof(cl_mem), &m_corespsBlocksCL);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(sumKernel, idx++, sizeof(cl_mem), &m_corespsCL);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(sumKernel, idx++, sizeof(cl_uint), &size);
			NUI_CHECK_CL_ERR(err);

			// Run kernel to calculate
			UINT nblocks = (UINT)( std::ceil( (float)size / (float)WORK_GROUP_SIZE) );
			kernelGlobalSize[0] = nblocks * WORK_GROUP_SIZE;
			err = clEnqueueNDRangeKernel(
				queue,
				sumKernel,
				1,
				nullptr,
				kernelGlobalSize,
				local_ws,
				0,
				NULL,
				NULL
				);
			NUI_CHECK_CL_ERR(err);

			err = clEnqueueReadBuffer(
				queue,
				m_corespsCL,
				CL_TRUE,//blocking
				0,
				nblocks * KINFU_ICP_CORESPS_NUM * sizeof(cl_float),
				corespResult.get(),
				0,
				NULL,
				NULL
			);
			NUI_CHECK_CL_ERR(err);

			m_error = 0.0f;
			m_count = 0.0f;
			for(UINT n = 0; n < nblocks; ++n)
			{
				UINT stride = n * KINFU_ICP_CORESPS_NUM;
				m_error += corespResult[stride + KINFU_ICP_CORESPS_NUM-2];
				m_count += corespResult[stride + KINFU_ICP_CORESPS_NUM-1];
			}
#ifdef _DEBUG
			//For debug
			//std::cout << "icpcount:" << icpCount << "\t" << "icperror:" << sqrt(icpError) / icpCount << std::endl;
#endif
			m_error = (m_count > 0.f) ? (sqrt(m_error) / m_count) : std::numeric_limits<float>::max();
			if((m_count < 1.f) || m_error < 1e-5f)
			{
				break;
			}
			/*else
			{
				std::cout << "icpcount:" << icpCount << "\t" << "icperror:" << sqrt(icpError) / icpCount << std::endl;
			}*/

			Eigen::Matrix<double, 6, 6, Eigen::RowMajor> A;
			Eigen::Matrix<double, 6, 1> b;

			int shift = 0;
			for (int i = 0; i < 6; ++i)  //rows
			{
				for (int j = i; j < 7; ++j)    // cols + b
				{
					float value = 0.0f;
					for(UINT n = 0; n < nblocks; ++n)
					{
						UINT stride = n * KINFU_ICP_CORESPS_NUM;
						value +=  corespResult[stride + shift];
					}
					if (j == 6)       // vector b
						b[i] = value;
					else
						A(j,i) = A(i,j) = value;
					shift++;
				}
			}

			//checking nullspace
			double det = A.determinant ();

			if (fabs (det) < 1e-15 || _isnan (det))
			{
				if (_isnan (det)) std::cout << "qnan" << std::endl;

				return (false);
			}
			//float maxc = A.maxCoeff();

			Eigen::Matrix<float, 6, 1> result = A.llt ().solve (b).cast<float>();
			//Eigen::Matrix<float, 6, 1> result = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

			float alpha = result (0);
			float beta  = result (1);
			float gamma = result (2);

			Eigen::Matrix3f Rinc = (Eigen::Matrix3f)AngleAxisf (gamma, Vector3f::UnitZ ()) * AngleAxisf (beta, Vector3f::UnitY ()) * AngleAxisf (alpha, Vector3f::UnitX ());
			Vector3f tinc = result.tail<3> ();

			//compose
			tcurr = Rinc * tcurr + tinc;
			Rcurr = Rinc * Rcurr;
		}
	}

	pCameraState->UpdateCameraTransform(Rcurr, tcurr);

#ifdef _DEBUG
	//For debug
	//std::cout << "t:" << tcurr[0] << "\t" << tcurr[1] << "\t" << tcurr[2] << std::endl;
#endif

	return true;
}


bool NuiKinfuOpenCLDepthTracker::previousBufferToData(NuiCLMappableData* pMappableData)
{
	assert(pMappableData);
	if(!pMappableData)
		return false;

	if(!m_verticesArrCL[0])
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
		m_verticesArrCL[0],
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

bool	NuiKinfuOpenCLDepthTracker::previousNormalImageToData(NuiCLMappableData* pMappableData)
{
	assert(pMappableData);
	if(!pMappableData)
		return false;

	if(!m_verticesArrCL[0])
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
	err = clSetKernelArg(rgbaKernel, idx++, sizeof(cl_mem), &m_verticesArrCL[0]);
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

