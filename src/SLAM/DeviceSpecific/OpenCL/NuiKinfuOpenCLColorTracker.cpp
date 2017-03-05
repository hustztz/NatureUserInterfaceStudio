#include "NuiKinfuOpenCLColorTracker.h"

#include "NuiKinfuOpenCLFrame.h"
#include "NuiKinfuOpenCLFeedbackFrame.h"
#include "NuiKinfuOpenCLCameraState.h"

#include <cmath>
#include "Foundation/NuiDebugMacro.h"
#include "Shape/NuiCLMappableData.h"

#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

#include <iostream>
#include <boost/smart_ptr.hpp>

#define KINFU_COLOR_ICP_CORESPS_NUM 28+1
#define WORK_GROUP_SIZE 128

using Eigen::AngleAxisf;

NuiKinfuOpenCLColorTracker::NuiKinfuOpenCLColorTracker(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight)
	: m_configuration(config)
	, m_colorDiffCL(NULL)
	, m_numValidCL(NULL)
	, m_corespsCL(NULL)
	, m_corespsBlocksCL(NULL)
	, m_nWidth(nWidth)
	, m_nHeight(nHeight)
	, m_count(0.0f)
{
	const NuiTrackerConfig::ITERATION_CLASS& iterations = m_configuration.iterations;
	m_colorsArrCL.resize(iterations.size());
	m_gradientXArrCL.resize(iterations.size());
	m_gradientYArrCL.resize(iterations.size());
	for (UINT i = 0; i < iterations.size(); ++i)
	{
		m_colorsArrCL[i] = NULL;
		m_gradientXArrCL[i] = NULL;
		m_gradientYArrCL[i] = NULL;
	}

	AcquireBuffers();
}

NuiKinfuOpenCLColorTracker::~NuiKinfuOpenCLColorTracker()
{
	ReleaseBuffers();
}

void NuiKinfuOpenCLColorTracker::AcquireBuffers()
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	const NuiTrackerConfig::ITERATION_CLASS& iterations = m_configuration.iterations;
	for (UINT i = 0; i < iterations.size(); ++i)
	{
		m_colorsArrCL[i] = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, (m_nWidth>>i)*(m_nHeight>>i)*sizeof(BGRQUAD), NULL, &err);
		NUI_CHECK_CL_ERR(err);
		m_gradientXArrCL[i] = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, (m_nWidth>>i)*(m_nHeight>>i)*sizeof(BGRQUAD), NULL, &err);
		NUI_CHECK_CL_ERR(err);
		m_gradientYArrCL[i] = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, (m_nWidth>>i)*(m_nHeight>>i)*sizeof(BGRQUAD), NULL, &err);
		NUI_CHECK_CL_ERR(err);
	}
	UINT nblocks = (UINT)std::ceil( (float)(m_nWidth*m_nHeight) / WORK_GROUP_SIZE);
	m_corespsBlocksCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nblocks*KINFU_COLOR_ICP_CORESPS_NUM*sizeof(cl_float), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	UINT nItems = (UINT)std::ceil( (float)(nblocks) / WORK_GROUP_SIZE);
	m_corespsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nItems*KINFU_COLOR_ICP_CORESPS_NUM*sizeof(cl_float), NULL, &err);
	NUI_CHECK_CL_ERR(err);

	m_colorDiffCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(cl_float), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_numValidCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(cl_float), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}

void NuiKinfuOpenCLColorTracker::ReleaseBuffers()
{
	const UINT nBufferSize = m_colorsArrCL.size();
	for (UINT i = 0; i < nBufferSize; ++i)
	{
		if (m_colorsArrCL[i]) {
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_colorsArrCL[i]);
			NUI_CHECK_CL_ERR(err);
			m_colorsArrCL[i] = NULL;
		}
		if (m_gradientXArrCL[i]) {
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_gradientXArrCL[i]);
			NUI_CHECK_CL_ERR(err);
			m_gradientXArrCL[i] = NULL;
		}
		if (m_gradientYArrCL[i]) {
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_gradientYArrCL[i]);
			NUI_CHECK_CL_ERR(err);
			m_gradientYArrCL[i] = NULL;
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
	if (m_colorDiffCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_colorDiffCL);
		NUI_CHECK_CL_ERR(err);
		m_colorDiffCL = NULL;
	}
	if (m_numValidCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_numValidCL);
		NUI_CHECK_CL_ERR(err);
		m_numValidCL = NULL;
	}
}

void	NuiKinfuOpenCLColorTracker::SubSampleColors(cl_mem colorsCL)
{
	if(!colorsCL)
		return;

	// half sample the input depth maps into the pyramid levels
	// Get the kernel
	cl_kernel pyrDownKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_ESTIMATE_HALF_BGRA_SAMPLE);
	assert(pyrDownKernel);
	if (!pyrDownKernel)
	{
		NUI_ERROR("Get kernel 'E_ESTIMATE_HALF_SAMPLE' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Copy
	err = clEnqueueCopyBuffer(
		queue,
		colorsCL,
		m_colorsArrCL[0],
		0,
		0,
		m_nWidth * m_nHeight * sizeof(BGRQUAD),
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	const UINT filter_radius = 1;

	// Set kernel arguments
	cl_uint idx = 0;
	size_t kernelGlobalSize[2];
	const UINT nBufferSize = m_colorsArrCL.size();
	for (UINT i = 1; i < nBufferSize; ++i)
	{
		// Set kernel arguments
		idx = 0;
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(cl_mem), &m_colorsArrCL[i-1]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(cl_mem), &m_colorsArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(UINT), &filter_radius);
		NUI_CHECK_CL_ERR(err);

		// Run kernel to calculate
		kernelGlobalSize[0] = m_nWidth>>i;
		kernelGlobalSize[1] = m_nHeight>>i;
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

void	NuiKinfuOpenCLColorTracker::GradientBuffers()
{
	// half sample the input depth maps into the pyramid levels
	// Get the kernel
	cl_kernel gradientKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_BGRA_GRADIENT);
	assert(gradientKernel);
	if (!gradientKernel)
	{
		NUI_ERROR("Get kernel 'E_BGRA_GRADIENT' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	size_t kernelGlobalSize[2];
	const UINT nBufferSize = m_colorsArrCL.size();
	for (UINT i = 0; i < nBufferSize; ++i)
	{
		// Set kernel arguments
		idx = 0;
		err = clSetKernelArg(gradientKernel, idx++, sizeof(cl_mem), &m_colorsArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(gradientKernel, idx++, sizeof(cl_mem), &m_gradientXArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(gradientKernel, idx++, sizeof(cl_mem), &m_gradientYArrCL[i]);
		NUI_CHECK_CL_ERR(err);

		// Run kernel to calculate
		kernelGlobalSize[0] = m_nWidth>>i;
		kernelGlobalSize[1] = m_nHeight>>i;
		err = clEnqueueNDRangeKernel(
			queue,
			gradientKernel,
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

float	NuiKinfuOpenCLColorTracker::GetColorsDifference(
	int level_index,
	cl_mem vertices,
	cl_mem colors,
	cl_mem level_colors,
	cl_mem cameraParamsCL,
	const Matrix3frm& rot,
	const Vector3f& trans)
{
	// Get the kernel
	cl_kernel colorDiffKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_COLOR_DIFFERENCE);
	assert(colorDiffKernel);
	if (!colorDiffKernel)
	{
		NUI_ERROR("Get kernel 'E_COLOR_DIFFERENCE' failed!\n");
		return 0.0;
	}

	int div = 1 << level_index;

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	cl_uint idx = 0;
	err = clSetKernelArg(colorDiffKernel, idx++, sizeof(cl_int), &div);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(colorDiffKernel, idx++, sizeof(cl_mem), &vertices);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(colorDiffKernel, idx++, sizeof(cl_mem), &colors);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(colorDiffKernel, idx++, sizeof(cl_mem), &level_colors);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(colorDiffKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(colorDiffKernel, idx++, sizeof(float)*8, rot.data());
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(colorDiffKernel, idx++, sizeof(float), rot.data()+8);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(colorDiffKernel, idx++, sizeof(float)*4, trans.data());
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(colorDiffKernel, idx++, sizeof(cl_mem), &m_colorDiffCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(colorDiffKernel, idx++, sizeof(cl_mem), &m_numValidCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[1] = { m_nWidth * m_nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		colorDiffKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	float numValid = 0.0;
	err = clEnqueueReadBuffer(
		queue,
		m_numValidCL,
		CL_TRUE,//blocking
		0,
		sizeof(float),
		&numValid,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	if(numValid <= 0.0f)
		return -1.0f;

	float colorDiff = 0.0;
	err = clEnqueueReadBuffer(
		queue,
		m_colorDiffCL,
		CL_TRUE,//blocking
		0,
		sizeof(float),
		&colorDiff,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	float scaleForOcclusions = (float)(m_nWidth * m_nHeight) / numValid;
	return colorDiff * scaleForOcclusions;
}

bool NuiKinfuOpenCLColorTracker::ColorIterativeClosestPoint(
	cl_mem verticesPrevCL,
	cl_mem colorsPrevCL,
	NuiKinfuCameraState* pCameraState,
	Eigen::Affine3f *hint)
{
	if(!pCameraState)
		return false;
	NuiKinfuOpenCLCameraState* pCLCamera = dynamic_cast<NuiKinfuOpenCLCameraState*>(pCameraState);
	if(!pCLCamera)
		return false;

	cl_mem cameraParamsCL = pCLCamera->GetCameraParamsBuffer();
	if(!cameraParamsCL)
		return false;

	// Get the kernel
	cl_kernel colorIcpKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_COLOR_ICP_BLOCK);
	assert(colorIcpKernel);
	if (!colorIcpKernel)
	{
		NUI_ERROR("Get kernel 'E_COLOR_ICP_BLOCK' failed!\n");
		return false;
	}

	cl_kernel sumKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_COLOR_ICP_SUM);
	assert(sumKernel);
	if (!sumKernel)
	{
		NUI_ERROR("Get kernel 'E_COLOR_ICP_SUM' failed!\n");
		return false;
	}

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

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	cl_uint idx = 0;
	cl_short bBackgroundMask = false;
	size_t kernelGlobalSize[1] = {m_nWidth * m_nHeight};
	size_t local_ws[1] = {WORK_GROUP_SIZE};
	boost::scoped_array<float> corespResult(new float[kernelGlobalSize[0] / WORK_GROUP_SIZE * KINFU_COLOR_ICP_CORESPS_NUM]);

	/** \brief array with IPC iteration numbers for each pyramid level */
	const NuiTrackerConfig::ITERATION_CLASS& iterations = m_configuration.iterations;
	int LEVELS = (int)iterations.size();

	//ScopeTime time("icp-all");
	for (int level_index = LEVELS-1; level_index>=0; --level_index)
	{
		int div = 1 << level_index;
		int iter_num = iterations[level_index].m_num;
		NuiTrackerConfig::TrackerIterationType iter_type = iterations[level_index].m_type;
		bool rotationOnly = (iter_type == NuiTrackerConfig::eTracker_Iteration_Rotation);
		int numPara = rotationOnly ? 3 : 6, startPara = rotationOnly ? 3 : 0, numParaSQ = rotationOnly ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

		for (int iter = 0; iter < iter_num; ++iter)
		{
			// Compute euler angles
			const NuiCameraPos& cameraPos = pCameraState->GetCameraPos();
			Matrix3frm Rdelta = cameraPos.getRotation().inverse() * Rcurr;
			Vector3f eulerAngles = Rdelta.eulerAngles(2, 1, 0);

			idx = 0;
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_int), &div);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &verticesPrevCL);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &colorsPrevCL);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_colorsArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_gradientXArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_gradientYArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float)*8, Rcurr.data());
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float), Rcurr.data()+8);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float)*4, tcurr.data());
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_int), &numPara);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_int), &startPara);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_corespsBlocksCL);
			NUI_CHECK_CL_ERR(err);

			// Run kernel to calculate
			err = clEnqueueNDRangeKernel(
				queue,
				colorIcpKernel,
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
				nblocks * KINFU_COLOR_ICP_CORESPS_NUM * sizeof(float),
				corespResult.get(),
				0,
				NULL,
				NULL
				);
			NUI_CHECK_CL_ERR(err);

			m_count = 0.0f;
			for(UINT n = 0; n < nblocks; ++n)
			{
				UINT stride = n * KINFU_COLOR_ICP_CORESPS_NUM;
				m_count += corespResult[stride + KINFU_COLOR_ICP_CORESPS_NUM-1];
			}
			if(m_count < 1.f)
			{
				break;
			}
			float scaleForOcclusions = (float)(m_nWidth * m_nHeight) / m_count;

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
						UINT stride = n * KINFU_COLOR_ICP_CORESPS_NUM;
						value +=  corespResult[stride + shift];
					}
					if (j == 6)       // vector b
						b[i] = value * scaleForOcclusions;
					else
						A(j,i) = A(i,j) = value * scaleForOcclusions;
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

	return true;
}

bool NuiKinfuOpenCLColorTracker::EstimatePose(
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

	SubSampleColors(pCLFrame->GetColorBuffer());
	GradientBuffers();

	if(!pFeedbackFrame)
		return false;
	NuiKinfuOpenCLFeedbackFrame* pCLFeedbackFrame = dynamic_cast<NuiKinfuOpenCLFeedbackFrame*>(pFeedbackFrame);
	if(!pCLFeedbackFrame)
		return false;

	return ColorIterativeClosestPoint(
		pCLFeedbackFrame->GetVertexBuffer(),
		pCLFrame->GetColorBuffer(), //pCLFeedbackFrame->GetColorBuffer(),
		pCameraState,
		hint);
}
