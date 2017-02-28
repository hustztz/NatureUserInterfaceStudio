#include "NuiKinfuOpenCLIntensityTracker.h"

#include "NuiKinfuOpenCLFrame.h"
#include "NuiKinfuOpenCLScene.h"
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

#define KINFU_ICP_CORESPS_NUM 29
#define WORK_GROUP_SIZE 128

using Eigen::AngleAxisf;

NuiKinfuOpenCLIntensityTracker::NuiKinfuOpenCLIntensityTracker(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight)
	: NuiKinfuOpenCLDepthTracker(config, nWidth, nHeight)
{
	m_iterations = config.iterations;
	m_intensitiesArrCL.resize(m_iterations.size());
	m_intensitiesPrevArrCL.resize(m_iterations.size());
	m_intensityDerivsPrevArrCL.resize(m_iterations.size());
	for (UINT i = 0; i < m_iterations.size(); ++i)
	{
		m_intensitiesArrCL[i] = NULL;
		m_intensitiesPrevArrCL[i] = NULL;
		m_intensityDerivsPrevArrCL[i] = NULL;
	}

	AcquireBuffers();
}

NuiKinfuOpenCLIntensityTracker::~NuiKinfuOpenCLIntensityTracker()
{
	ReleaseBuffers();
}

void NuiKinfuOpenCLIntensityTracker::AcquireBuffers()
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	for (UINT i = 0; i < m_iterations.size(); ++i)
	{
		m_intensitiesArrCL[i] = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, (m_nWidth>>i)*(m_nHeight>>i)*sizeof(cl_float), NULL, &err);
		NUI_CHECK_CL_ERR(err);
		m_intensitiesPrevArrCL[i] = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, (m_nWidth>>i)*(m_nHeight>>i)*sizeof(cl_float), NULL, &err);
		NUI_CHECK_CL_ERR(err);
		m_intensityDerivsPrevArrCL[i] = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, (m_nWidth>>i)*(m_nHeight>>i)*2*sizeof(cl_float), NULL, &err);
		NUI_CHECK_CL_ERR(err);
	}
}

void NuiKinfuOpenCLIntensityTracker::ReleaseBuffers()
{
	for (UINT i = 0; i < m_iterations.size(); ++i)
	{
		if (m_intensitiesArrCL[i]) {
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_intensitiesArrCL[i]);
			NUI_CHECK_CL_ERR(err);
			m_intensitiesArrCL[i] = NULL;
		}
		if (m_intensitiesPrevArrCL[i]) {
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_intensitiesPrevArrCL[i]);
			NUI_CHECK_CL_ERR(err);
			m_intensitiesPrevArrCL[i] = NULL;
		}
		if (m_intensityDerivsPrevArrCL[i]) {
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_intensityDerivsPrevArrCL[i]);
			NUI_CHECK_CL_ERR(err);
			m_intensityDerivsPrevArrCL[i] = NULL;
		}
	}
}

bool NuiKinfuOpenCLIntensityTracker::EvaluateFrame(NuiKinfuFrame* pFrame, NuiKinfuCameraState* pCameraState)
{
	if(!pFrame)
		return false;
	NuiKinfuOpenCLFrame* pCLFrame = dynamic_cast<NuiKinfuOpenCLFrame*>(pFrame);
	if(!pCLFrame)
		return false;

	cl_mem colorsCL = pCLFrame->GetColorsBuffer();
	ColorsToIntensity(colorsCL);

	return NuiKinfuOpenCLDepthTracker::EvaluateFrame(pFrame, pCameraState);
}

bool NuiKinfuOpenCLIntensityTracker::EstimatePose(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint)
{
	return IntensityIterativeClosestPoint(pCameraState, hint);
}

void NuiKinfuOpenCLIntensityTracker::FeedbackPose(NuiKinfuCameraState* pCameraState)
{
	NuiKinfuOpenCLDepthTracker::FeedbackPose(pCameraState);
	CopyPrevIntensityMaps();
}

void NuiKinfuOpenCLIntensityTracker::resizePrevsMaps()
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

	cl_kernel pyrDownKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_ESTIMATE_HALF_SAMPLE);
	assert(pyrDownKernel);
	if (!pyrDownKernel)
	{
		NUI_ERROR("Get kernel 'E_ESTIMATE_HALF_SAMPLE' failed!\n");
		return;
	}
	// Get the kernel
	cl_kernel derivativesKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_INTENSITY_DERIVATIVES);
	assert(derivativesKernel);
	if (!derivativesKernel)
	{
		NUI_ERROR("Get kernel 'E_INTENSITY_DERIVATIVES' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	
	// Set kernel arguments
	cl_uint idx = 0;
	size_t kernelGlobalSize[2] = { m_nWidth, m_nHeight };

	if(m_intensitiesArrCL[0] && m_intensityDerivsPrevArrCL[0])
	{
		err = clSetKernelArg(derivativesKernel, idx++, sizeof(cl_mem), &m_intensitiesArrCL[0]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(derivativesKernel, idx++, sizeof(cl_mem), &m_intensityDerivsPrevArrCL[0]);
		NUI_CHECK_CL_ERR(err);

		// Run kernel to calculate 
		err = clEnqueueNDRangeKernel(
			queue,
			derivativesKernel,
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

	for (UINT i = 1; i < m_iterations.size(); ++i)
	{
		// Vertices
		idx = 0;
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), &m_verticesPrevArrCL[i-1]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), &m_normalsPrevArrCL[i-1]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), &m_intensitiesArrCL[i-1]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), &m_verticesPrevArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), &m_normalsPrevArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), &m_intensitiesArrCL[i]);
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

		// Intensity Derivative
		if(m_intensitiesArrCL[i] && m_intensityDerivsPrevArrCL[i])
		{
			// Set kernel arguments
			idx = 0;
			err = clSetKernelArg(derivativesKernel, idx++, sizeof(cl_mem), &m_intensitiesArrCL[i]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(derivativesKernel, idx++, sizeof(cl_mem), &m_intensityDerivsPrevArrCL[i]);
			NUI_CHECK_CL_ERR(err);

			// Run kernel to calculate 
			err = clEnqueueNDRangeKernel(
				queue,
				derivativesKernel,
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
}

void NuiKinfuOpenCLIntensityTracker::ColorsToIntensity(cl_mem colorsCL)
{
	if(!colorsCL || !m_intensitiesArrCL[0])
		return;

	// Get the kernel
	cl_kernel convertKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_BGRA_TO_INTENSITY);
	assert(convertKernel);
	if (!convertKernel)
	{
		NUI_ERROR("Get kernel 'E_BGRA_TO_INTENSITY' failed!\n");
		return;
	}
	
	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(convertKernel, idx++, sizeof(cl_mem), &colorsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(convertKernel, idx++, sizeof(cl_mem), &m_intensitiesArrCL[0]);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSizeOneDegree[1] = { m_nWidth * m_nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		convertKernel,
		1,
		nullptr,
		kernelGlobalSizeOneDegree,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

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

	size_t kernelGlobalSize[2];
	for (UINT i = 1; i < m_iterations.size(); ++i)
	{
		// Set kernel arguments
		idx = 0;
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(cl_mem), &m_intensitiesArrCL[i-1]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(cl_mem), &m_intensitiesArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(UINT), &m_configuration.filter_radius);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(float), &m_configuration.depth_threshold);
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

void NuiKinfuOpenCLIntensityTracker::CopyPrevIntensityMaps()
{
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	for (UINT i = 0; i < m_iterations.size(); ++i)
	{
		if(!m_intensitiesArrCL[i] || !m_intensitiesPrevArrCL[i])
			continue;

		const UINT nPointsNum = (m_nWidth >> i) * (m_nHeight >> i);

		err = clEnqueueCopyBuffer(
			queue,
			m_intensitiesArrCL[i],
			m_intensitiesPrevArrCL[i],
			0,
			0,
			nPointsNum * sizeof(cl_float),
			0,
			NULL,
			NULL
			);
		NUI_CHECK_CL_ERR(err);
	}
}

bool NuiKinfuOpenCLIntensityTracker::IntensityIterativeClosestPoint(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint)
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
	cl_kernel colorIcpKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_INTENSITY_ICP_BLOCK);
	assert(colorIcpKernel);
	if (!colorIcpKernel)
	{
		NUI_ERROR("Get kernel 'E_INTENSITY_ICP_BLOCK' failed!\n");
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
	boost::scoped_array<float> corespResult(new float[kernelGlobalSize[0] / WORK_GROUP_SIZE * KINFU_ICP_CORESPS_NUM]);

	/** \brief array with IPC iteration numbers for each pyramid level */
	int LEVELS = (int)m_iterations.size();

	//ScopeTime time("icp-all");
	for (int level_index = LEVELS-1; level_index>=0; --level_index)
	{
		int div = 1 << level_index;
		int iter_num = m_iterations[level_index];
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
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_verticesHierarchyCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_normalsHierarchyCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_intensitiesArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float)*8, Rcurr.data());
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float), Rcurr.data()+8);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float)*4, tcurr.data());
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_verticesPrevArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_normalsPrevArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_intensitiesPrevArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_intensityDerivsPrevArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &previousTransform);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float)*4, eulerAngles.data());
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float), &m_configuration.dist_threshold);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float), &m_configuration.normal_threshold);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float), &m_configuration.color_dist_threshold);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float), &m_configuration.color_gradiant_min);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_corespsBlocksCL);
			NUI_CHECK_CL_ERR(err);

			// Run kernel to calculate
			kernelGlobalSize[0] = (m_nWidth >> level_index) * (m_nHeight >> level_index);
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
				nblocks * KINFU_ICP_CORESPS_NUM * sizeof(float),
				corespResult.get(),
				0,
				NULL,
				NULL
				);
			NUI_CHECK_CL_ERR(err);

			float icpError = 0.0f;
			float icpCount = 0.0f;
			for(UINT n = 0; n < nblocks; ++n)
			{
				UINT stride = n * KINFU_ICP_CORESPS_NUM;
				icpError += corespResult[stride + KINFU_ICP_CORESPS_NUM-2];
				icpCount += corespResult[stride + KINFU_ICP_CORESPS_NUM-1];
			}

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

	return true;
}

bool	NuiKinfuOpenCLIntensityTracker::VerticesToMappablePosition(NuiCLMappableData* pMappableData)
{
	assert(pMappableData);
	if(!pMappableData)
		return false;

	// Get the kernel
	cl_kernel intensityKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_INTENSITY_TO_FLOAT4);
	assert(intensityKernel);
	if (intensityKernel && m_intensitiesPrevArrCL[0])
	{
		const UINT nPointsNum = m_nWidth * m_nHeight;
		if( nPointsNum != pMappableData->ColorStream().size() )
		{
			NuiMappableAccessor::asVectorImpl(pMappableData->ColorStream())->data().resize(nPointsNum);
		}

		cl_int           err = CL_SUCCESS;
		cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
		// 
		err = clFinish(queue);
		NUI_CHECK_CL_ERR(err);

		cl_mem colorsGL = NuiOpenCLBufferFactory::asColor4fBufferCL(pMappableData->ColorStream());

		// Acquire OpenGL objects before use
		cl_mem glObjs[] = {
			colorsGL
		};
		openclutil::enqueueAcquireHWObjects(
			sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

		// Set kernel arguments
		cl_uint idx = 0;
		err = clSetKernelArg(intensityKernel, idx++, sizeof(cl_mem), &m_intensitiesPrevArrCL[0]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(intensityKernel, idx++, sizeof(cl_mem), &colorsGL);
		NUI_CHECK_CL_ERR(err);

		size_t kernelGlobalSize[1] = { nPointsNum };
		err = clEnqueueNDRangeKernel(
			queue,
			intensityKernel,
			1,
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

		pMappableData->SetStreamDirty(true);
	}
	else
	{
		NUI_ERROR("Get kernel 'E_INTENSITY_TO_FLOAT4' failed!\n");
	}

	return NuiKinfuOpenCLDepthTracker::previousBufferToData(pMappableData);
}
