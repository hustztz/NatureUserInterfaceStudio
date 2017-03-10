#include "NuiKinfuOpenCLDepthTracker.h"

#include "NuiKinfuOpenCLFrame.h"
#include "NuiKinfuOpenCLFeedbackFrame.h"
#include "NuiKinfuOpenCLCameraState.h"

#include "Foundation/NuiDebugMacro.h"

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
	, m_corespsCL(NULL)
	, m_corespsBlocksCL(NULL)
	, m_nWidth(nWidth)
	, m_nHeight(nHeight)
	, m_error(0.0f)
	, m_count(0.0f)
{
	const NuiTrackerConfig::ITERATION_CLASS& iterations = m_configuration.iterations;
	UINT nBufferSize = (iterations.size() > 0) ? iterations.size() - 1 : 0;
	m_depthsHierarchyCL.resize(nBufferSize);
	m_verticesHierarchyCL.resize(nBufferSize);
	for (UINT i = 0; i < nBufferSize; ++i)
	{
		m_depthsHierarchyCL[i] = NULL;
		m_verticesHierarchyCL[i] = NULL;
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

	const NuiTrackerConfig::ITERATION_CLASS& iterations = m_configuration.iterations;
	UINT nBufferSize = (iterations.size() > 0) ? iterations.size() - 1 : 0;
	for (UINT i = 0; i < nBufferSize; ++i)
	{
		m_depthsHierarchyCL[i] = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, (m_nWidth>>(i+1))*(m_nHeight>>(i+1))*sizeof(cl_float), NULL, &err);
		NUI_CHECK_CL_ERR(err);
		m_verticesHierarchyCL[i] = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, (m_nWidth>>(i+1))*(m_nHeight>>(i+1))*3*sizeof(cl_float), NULL, &err);
		NUI_CHECK_CL_ERR(err);
	}

	UINT nblocks = (UINT)std::ceil( (float)(m_nWidth*m_nHeight) / WORK_GROUP_SIZE);
	m_corespsBlocksCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nblocks*KINFU_ICP_CORESPS_NUM*sizeof(cl_float), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	UINT nItems = (UINT)std::ceil( (float)(nblocks) / WORK_GROUP_SIZE);
	m_corespsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nItems*KINFU_ICP_CORESPS_NUM*sizeof(cl_float), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}

void NuiKinfuOpenCLDepthTracker::ReleaseBuffers()
{
	for (UINT i = 0; i < m_depthsHierarchyCL.size(); ++i)
	{
		if (m_depthsHierarchyCL[i]) {
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_depthsHierarchyCL[i]);
			NUI_CHECK_CL_ERR(err);
			m_depthsHierarchyCL[i] = NULL;
		}
		if (m_verticesHierarchyCL[i]) {
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_verticesHierarchyCL[i]);
			NUI_CHECK_CL_ERR(err);
			m_verticesHierarchyCL[i] = NULL;
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

bool NuiKinfuOpenCLDepthTracker::EstimatePose(
	NuiKinfuFrame* pFrame,
	NuiKinfuFeedbackFrame* pFeedbackFrame,
	NuiKinfuCameraState* pCameraState,
	Eigen::Affine3f *hint
	)
{
	if(!pFrame)
		return false;
	NuiKinfuOpenCLFrame* pCLFrame = dynamic_cast<NuiKinfuOpenCLFrame*>(pFrame);
	if(!pCLFrame)
		return false;
	// filter the input depth map
	SubSampleDepths(pCLFrame->GetFilteredDepthBuffer());

	if(!pCameraState)
		return false;
	NuiKinfuOpenCLCameraState* pCLCamera = dynamic_cast<NuiKinfuOpenCLCameraState*>(pCameraState);
	if(!pCLCamera)
		return false;

	HierarchyDepth2vertex(pCLCamera->GetCameraParamsBuffer());

	if(!pFeedbackFrame)
		return false;
	NuiKinfuOpenCLFeedbackFrame* pCLFeedbackFrame = dynamic_cast<NuiKinfuOpenCLFeedbackFrame*>(pFeedbackFrame);
	if(!pCLFeedbackFrame)
		return false;

	return IterativeClosestPoint(
		pCLFrame->GetVertexBuffer(),
		pCLFeedbackFrame->GetVertexBuffer(),
		pCLFeedbackFrame->GetNormalBuffer(),
		pCameraState,
		hint);
}

void NuiKinfuOpenCLDepthTracker::SubSampleDepths(cl_mem filteredDepths)
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
	UINT nBufferSize = m_depthsHierarchyCL.size();
	for (UINT i = 0; i < nBufferSize; ++i)
	{
		cl_mem srcBuffer = (i > 0) ? m_depthsHierarchyCL[i-1] : filteredDepths;
		cl_mem dstBuffer = m_depthsHierarchyCL[i];
		// Set kernel arguments
		cl_uint idx = 0;
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(cl_mem), &srcBuffer);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(cl_mem), &dstBuffer);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(UINT), &subSampleRadius);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(float), &m_configuration.depth_threshold);
		NUI_CHECK_CL_ERR(err);

		// Run kernel to calculate 
		size_t kernelGlobalSize[2] = { m_nWidth>>(i+1), m_nHeight>>(i+1) };
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

void NuiKinfuOpenCLDepthTracker::HierarchyDepth2vertex(cl_mem cameraParamsCL)
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

	for (UINT i = 0; i < m_depthsHierarchyCL.size(); ++i)
	{
		int div = 1 << (i+1); 
		// Set kernel arguments
		cl_uint idx = 0;
		err = clSetKernelArg(depth2vertexKernel, idx++, sizeof(cl_mem), &m_depthsHierarchyCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(depth2vertexKernel, idx++, sizeof(cl_mem), &m_verticesHierarchyCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(depth2vertexKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(depth2vertexKernel, idx++, sizeof(int), &div);
		NUI_CHECK_CL_ERR(err);

		// Run kernel to calculate 
		size_t kernelGlobalSize[2] = { m_nWidth >> (i+1), m_nHeight >> (i+1) };
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

///////////////////////////////////////////////////////////////////////////////////////////
// Iterative Closest Point
bool NuiKinfuOpenCLDepthTracker::IterativeClosestPoint(
	cl_mem verticesCL,
	cl_mem verticesPrevCL,
	cl_mem normalsPrevCL,
	NuiKinfuCameraState* pCameraState,
	Eigen::Affine3f *hint)
{
	if(!verticesCL || !verticesPrevCL || !normalsPrevCL || !pCameraState)
		return false;
	NuiKinfuOpenCLCameraState* pCLCamera = dynamic_cast<NuiKinfuOpenCLCameraState*>(pCameraState);
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
	float goodCores[KINFU_ICP_CORESPS_NUM];

	Matrix3frm Rcurr;
	Vector3f tcurr;
	if(hint)
	{
		Rcurr = hint->rotation();
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
	const NuiTrackerConfig::ITERATION_CLASS& iterations = m_configuration.iterations;
	int LEVELS = (int)iterations.size();
	float distThreshStep = m_configuration.dist_threshold / LEVELS;
	float distThresh = m_configuration.dist_threshold + distThreshStep;

	//ScopeTime time("icp-all");
	for (int level_index = LEVELS-1; level_index>=0; --level_index)
	{
		const UINT numValidThreshould = (kernelGlobalSize[0] >> (2*level_index)) * 0.1;
		float lambda = 1.0;
		Matrix3frm Rgood = Rcurr;
		Vector3f tgood = tcurr;
		float errorOld = 1e20f;
		//distThresh = distThresh - distThreshStep;
		cl_mem srcVertices = (0 == level_index) ? verticesCL : m_verticesHierarchyCL[level_index-1];
		int iter_num = iterations[level_index].m_num;
		for (int iter = 0; iter < iter_num; ++iter)
		{
			idx = 0;
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &srcVertices);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(float)*8, Rcurr.data());
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(float), Rcurr.data()+8);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(float)*4, tcurr.data());
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &verticesPrevCL);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &normalsPrevCL);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &previousTransform);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(float), &distThresh);
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

			m_error = (m_count > 0.0f) ? sqrt(m_error) / m_count : std::numeric_limits<float>::max();
			/*if(newError < 1e-5f)
			{
				break;
			}*/

			if(m_count < (float)numValidThreshould || m_error > errorOld)
			{
				lambda *= 10.0f;
				Rcurr = Rgood;
				tcurr = tgood;
				if(0 == iter)
					break;
#ifdef _DEBUG
				//For debug
				std::cout << "icp reverse due to no enough valid point count." << std::endl;
#endif	
			}
			else
			{
				lambda /= 10.0f;
				Rgood = Rcurr;
				tgood = tcurr;
				errorOld = m_error;

				for (int i = 0, shift = 0; i < 6; ++i)  //rows
				{
					for (int j = i; j < 7; ++j)    // cols + b
					{
						goodCores[shift] = 0.0f;
						for(UINT n = 0; n < nblocks; ++n)
						{
							UINT stride = n * KINFU_ICP_CORESPS_NUM;
							goodCores[shift] +=  corespResult[stride + shift];
						}
						shift++;
					}
				}
#ifdef _DEBUG
				//For debug
				std::cout << "icp:" << level_index << ". iter:" << iter << ". count:" << m_count << ". error:" << m_error << std::endl;
#endif
			}

			Eigen::Matrix<double, 6, 6, Eigen::RowMajor> A;
			Eigen::Matrix<double, 6, 1> b;

			for (int i = 0, shift = 0; i < 6; ++i)  //rows
			{
				for (int j = i; j < 7; ++j)    // cols + b
				{
					if (j == 6)       // vector b
						b[i] = goodCores[shift] / m_count;
					else
						A(j,i) = A(i,j) = goodCores[shift] / m_count;
					shift++;
				}
				A(i,i) *= 1.0f + lambda;
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

			float stepLength = alpha * alpha + beta * beta + gamma * gamma;
			if (stepLength < 1e-6f)
				break;

			Eigen::Matrix3f Rinc = (Eigen::Matrix3f)AngleAxisf (gamma, Vector3f::UnitZ ()) * AngleAxisf (beta, Vector3f::UnitY ()) * AngleAxisf (alpha, Vector3f::UnitX ());
			Eigen::Matrix3f RincInv = Rinc.inverse();
			Vector3f tinc = result.tail<3> ();

			//compose
			tcurr = RincInv * tcurr + tinc;
			Rcurr =  RincInv * Rcurr;
		}
	}

	pCameraState->UpdateCameraTransform(Rcurr, tcurr);

#ifdef _DEBUG
	//For debug
	//std::cout << "t:" << tcurr[0] << "\t" << tcurr[1] << "\t" << tcurr[2] << std::endl;
#endif

	return true;
}
