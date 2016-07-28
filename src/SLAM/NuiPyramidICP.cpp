#include "NuiPyramidICP.h"

#include "NuiKinfuTransform.h"
#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

#include <iostream>
#include <boost/smart_ptr.hpp>

#define KINFU_ICP_CORESPS_NUM 29
#define WORK_GROUP_SIZE 128

using Eigen::AngleAxisf;

NuiPyramidICP::NuiPyramidICP(const NuiICPConfig& config, UINT nWidth, UINT nHeight)
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
	m_colorsArrCL.resize(m_iterations.size());
	m_verticesPrevArrCL.resize(m_iterations.size());
	m_normalsPrevArrCL.resize(m_iterations.size());
	m_colorsPrevArrCL.resize(m_iterations.size());
	for (UINT i = 0; i < m_iterations.size(); ++i)
	{
		m_depthsArrCL[i] = NULL;
		m_verticesArrCL[i] = NULL;
		m_normalsArrCL[i] = NULL;
		m_colorsArrCL[i] = NULL;
		m_verticesPrevArrCL[i] = NULL;
		m_normalsPrevArrCL[i] = NULL;
		m_colorsPrevArrCL[i] = NULL;
	}

	AcquireBuffers(true);
}

NuiPyramidICP::~NuiPyramidICP()
{
	ReleaseBuffers();
}

void NuiPyramidICP::AcquireBuffers(bool bHasColor)
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
		if(bHasColor)
		{
			m_colorsArrCL[i] = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, (m_nWidth>>i)*(m_nHeight>>i)*sizeof(BGRQUAD), NULL, &err);
			NUI_CHECK_CL_ERR(err);
			m_colorsPrevArrCL[i] = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, (m_nWidth>>i)*(m_nHeight>>i)*sizeof(BGRQUAD), NULL, &err);
			NUI_CHECK_CL_ERR(err);
		}
	}
	UINT nblocks = (UINT)std::ceil( (float)(m_nWidth*m_nHeight) / WORK_GROUP_SIZE);
	m_corespsBlocksCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nblocks*KINFU_ICP_CORESPS_NUM*sizeof(cl_float), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	UINT nItems = (UINT)std::ceil( (float)(nblocks) / WORK_GROUP_SIZE);
	m_corespsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nItems*KINFU_ICP_CORESPS_NUM*sizeof(cl_float), NULL, &err);
	NUI_CHECK_CL_ERR(err);

	GenerateGaussianBuffer();
}

void NuiPyramidICP::ReleaseBuffers()
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
		if (m_colorsArrCL[i]) {
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_colorsArrCL[i]);
			NUI_CHECK_CL_ERR(err);
			m_colorsArrCL[i] = NULL;
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
		if (m_colorsPrevArrCL[i]) {
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_colorsPrevArrCL[i]);
			NUI_CHECK_CL_ERR(err);
			m_colorsPrevArrCL[i] = NULL;
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

bool NuiPyramidICP::log(const std::string& fileName) const
{
	return m_configuration.log(fileName);
}

void NuiPyramidICP::input(cl_mem floatDepthsCL, cl_mem cameraParamsCL)
{
	// filter the input depth map
	SmoothDepths(floatDepthsCL);
	// half sample the input depth maps into the pyramid levels
	PyrDown();
	NormalEst(cameraParamsCL);
}

bool NuiPyramidICP::run(cl_mem cameraParamsCL, NuiKinfuTransform* pTransform, Eigen::Affine3f *hint)
{
	if(!cameraParamsCL || !pTransform)
		return false;

	return IterativeClosestPoint(cameraParamsCL, pTransform, hint);
}

void NuiPyramidICP::transformPrevs(cl_mem transformCL)
{
	TransformPrevMaps(transformCL);
}

void NuiPyramidICP::resizePrevs()
{
	ResizePrevMaps();
}

void NuiPyramidICP::copyPrevs()
{
	CopyPrevMaps();
}

void NuiPyramidICP::GenerateGaussianBuffer()
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

void NuiPyramidICP::SmoothDepths(cl_mem floatDepthsCL)
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

void NuiPyramidICP::PyrDown()
{
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

	for (UINT i = 1; i < m_iterations.size(); ++i)
	{
		// Set kernel arguments
		cl_uint idx = 0;
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(cl_mem), &m_depthsArrCL[i-1]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(cl_mem), &m_depthsArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(pyrDownKernel, idx++, sizeof(UINT), &m_configuration.filter_radius);
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

void NuiPyramidICP::NormalEst(cl_mem cameraParamsCL)
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

		///////////////////////////////////////
		// vertex2normal
		// Set kernel arguments
		idx = 0;
		err = clSetKernelArg(normalEstKernel, idx++, sizeof(cl_mem), &m_verticesArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(normalEstKernel, idx++, sizeof(cl_mem), &m_normalsArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(normalEstKernel, idx++, sizeof(float), &m_configuration.depth_threshold);
		NUI_CHECK_CL_ERR(err);
		
		// Run kernel to calculate 
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
bool NuiPyramidICP::IterativeClosestPoint(cl_mem cameraParamsCL, NuiKinfuTransform* pTransform, Eigen::Affine3f *hint)
{
	if(!cameraParamsCL || !pTransform)
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
		Rcurr = pTransform->getRotation(); // tranform to global coo for ith camera pose
		tcurr = pTransform->getTranslation();
	}
	cl_mem previousTransform = pTransform->getTransformCL();

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

	pTransform->setTransform(Rcurr, tcurr);

#ifdef _DEBUG
	//For debug
	//std::cout << "t:" << tcurr[0] << "\t" << tcurr[1] << "\t" << tcurr[2] << std::endl;
#endif

	return true;
}

void    NuiPyramidICP::ResizePrevMaps()
{
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
	cl_uint idx = 0;
	size_t kernelGlobalSize[2];

	for (UINT i = 1; i < m_iterations.size(); ++i)
	{
		idx = 0;
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), &m_verticesPrevArrCL[i-1]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), &m_normalsPrevArrCL[i-1]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), &m_verticesPrevArrCL[i]);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(resizeKernel, idx++, sizeof(cl_mem), &m_normalsPrevArrCL[i]);
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

void NuiPyramidICP::TransformPrevMaps(cl_mem transformCL)
{
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
	CopyPrevColorMaps();
}

void NuiPyramidICP::CopyPrevMaps()
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
	CopyPrevColorMaps();
}

void NuiPyramidICP::CopyPrevColorMaps()
{
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	for (UINT i = 0; i < m_iterations.size(); ++i)
	{
		if(!m_colorsArrCL[i] || !m_colorsPrevArrCL[i])
			continue;

		const UINT nPointsNum = (m_nWidth >> i) * (m_nHeight >> i);

		err = clEnqueueCopyBuffer(
			queue,
			m_colorsArrCL[i],
			m_colorsPrevArrCL[i],
			0,
			0,
			nPointsNum * 4 * sizeof(cl_uchar),
			0,
			NULL,
			NULL
			);
		NUI_CHECK_CL_ERR(err);
	}
}

//bool NuiPyramidICP::ColorIterativeClosestPoint(NuiCameraPos* pPos, Eigen::Affine3f *hint)
//{
//	// Get the kernel
//	cl_kernel colorIcpKernel =
//		NuiOpenCLKernelManager::instance().acquireKernel(E_COLOR_ICP_BLOCK);
//	assert(colorIcpKernel);
//	if (!colorIcpKernel)
//	{
//		NUI_ERROR("Get kernel 'E_COLOR_ICP_BLOCK' failed!\n");
//		return false;
//	}
//
//	cl_kernel sumKernel =
//		NuiOpenCLKernelManager::instance().acquireKernel(E_ICP_SUM);
//	assert(sumKernel);
//	if (!sumKernel)
//	{
//		NUI_ERROR("Get kernel 'E_ICP_SUM' failed!\n");
//		return false;
//	}
//
//	const NuiCameraIntrinsics& intri = pPos->getIntrinsics();
//	Matrix3frm Rprev_inv = pPos->getRotation().inverse (); //Rprev.t();
//
//	Matrix3frm Rcurr;
//	Vector3f tcurr;
//	if(hint)
//	{
//		Rcurr = hint->rotation().matrix();
//		tcurr = hint->translation().matrix();
//	}
//	else
//	{
//		Rcurr = pPos->getRotation(); // tranform to global coo for ith camera pose
//		tcurr = pPos->getTranslation();
//	}
//
//	// OpenCL command queue and device
//	cl_int           err = CL_SUCCESS;
//	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
//	cl_uint idx = 0;
//	cl_short bBackgroundMask = false;
//	size_t kernelGlobalSize[1] = {m_nWidth * m_nHeight};
//	size_t local_ws[1] = {WORK_GROUP_SIZE};
//	boost::scoped_array<float> corespResult(new float[kernelGlobalSize[0] / WORK_GROUP_SIZE * KINFU_ICP_CORESPS_NUM]);
//
//	/** \brief array with IPC iteration numbers for each pyramid level */
//	int LEVELS = (int)m_iterations.size();
//
//	//ScopeTime time("icp-all");
//	for (int level_index = LEVELS-1; level_index>=0; --level_index)
//	{
//		int div = 1 << level_index; 
//		float fx = intri.m_fx / div;
//		float fy = intri.m_fy / div;
//		float cx = intri.m_cx / div;
//		float cy = intri.m_cy / div;
//		cl_uint width = m_nWidth >> level_index;
//		cl_uint height = m_nHeight >> level_index;
//
//		int iter_num = m_iterations[level_index];
//		for (int iter = 0; iter < iter_num; ++iter)
//		{
//			idx = 0;
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_uint), &width);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_uint), &height);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_verticesArrCL[level_index]);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_normalsArrCL[level_index]);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float)*8, Rcurr.data());
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float), Rcurr.data()+8);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float)*4, tcurr.data());
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_verticesPrevArrCL[level_index]);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_normalsPrevArrCL[level_index]);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float)*8, Rprev_inv.data());
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float), Rprev_inv.data()+8);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float)*4, pPos->getTranslation().data());
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_short), &bBackgroundMask);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_depthsArrCL[level_index]);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float), &fx);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float), &fy);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float), &cx);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float), &cy);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float), &m_configuration.dist_threshold);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(float), &m_configuration.normal_threshold);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(colorIcpKernel, idx++, sizeof(cl_mem), &m_corespsBlocksCL);
//			NUI_CHECK_CL_ERR(err);
//
//			// Run kernel to calculate
//			kernelGlobalSize[0] = (m_nWidth >> level_index) * (m_nHeight >> level_index);
//			err = clEnqueueNDRangeKernel(
//				queue,
//				colorIcpKernel,
//				1,
//				nullptr,
//				kernelGlobalSize,
//				local_ws,
//				0,
//				NULL,
//				NULL
//				);
//			NUI_CHECK_CL_ERR(err);
//
//			UINT size = (UINT)(std::ceil( (float)kernelGlobalSize[0] / (float)local_ws[0] ));
//
//			idx = 0;
//			err = clSetKernelArg(sumKernel, idx++, sizeof(cl_mem), &m_corespsBlocksCL);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(sumKernel, idx++, sizeof(cl_mem), &m_corespsCL);
//			NUI_CHECK_CL_ERR(err);
//			err = clSetKernelArg(sumKernel, idx++, sizeof(cl_uint), &size);
//			NUI_CHECK_CL_ERR(err);
//
//			// Run kernel to calculate
//			UINT nblocks = (UINT)( std::ceil( (float)size / (float)WORK_GROUP_SIZE) );
//			kernelGlobalSize[0] = nblocks * WORK_GROUP_SIZE;
//			err = clEnqueueNDRangeKernel(
//				queue,
//				sumKernel,
//				1,
//				nullptr,
//				kernelGlobalSize,
//				local_ws,
//				0,
//				NULL,
//				NULL
//				);
//			NUI_CHECK_CL_ERR(err);
//
//			err = clEnqueueReadBuffer(
//				queue,
//				m_corespsCL,
//				CL_TRUE,//blocking
//				0,
//				nblocks * KINFU_ICP_CORESPS_NUM * sizeof(float),
//				corespResult.get(),
//				0,
//				NULL,
//				NULL
//				);
//			NUI_CHECK_CL_ERR(err);
//
//			clFinish(queue);
//
//			float icpError = 0.0f;
//			float icpCount = 0.0f;
//			for(UINT n = 0; n < nblocks; ++n)
//			{
//				UINT stride = n * KINFU_ICP_CORESPS_NUM;
//				icpError += corespResult[stride + KINFU_ICP_CORESPS_NUM-2];
//				icpCount += corespResult[stride + KINFU_ICP_CORESPS_NUM-1];
//			}
//#ifdef _DEBUG
//			std::cout << "icpcount:" << icpCount << "\t" << "icperror:" << sqrt(icpError) / icpCount << std::endl;
//#endif
//			if((0 == icpCount) || (sqrt(icpError) / icpCount) < 1e-5f)
//			{
//				if(bBackgroundMask && (0 != level_index))
//				{
//					break;
//				}
//				else
//				{
//					bBackgroundMask = true;
//					continue;
//				}
//			}
//			if((0 == level_index) && (iter == iter_num-2))
//			{
//				bBackgroundMask = true;
//			}
//
//			Eigen::Matrix<double, 6, 6, Eigen::RowMajor> A;
//			Eigen::Matrix<double, 6, 1> b;
//
//			int shift = 0;
//			for (int i = 0; i < 6; ++i)  //rows
//			{
//				for (int j = i; j < 7; ++j)    // cols + b
//				{
//					float value = 0.0f;
//					for(UINT n = 0; n < nblocks; ++n)
//					{
//						UINT stride = n * KINFU_ICP_CORESPS_NUM;
//						value +=  corespResult[stride + shift];
//					}
//					if (j == 6)       // vector b
//						b[i] = value;
//					else
//						A(j,i) = A(i,j) = value;
//					shift++;
//				}
//			}
//
//			//checking nullspace
//			double det = A.determinant ();
//
//			if (fabs (det) < 1e-15 || _isnan (det))
//			{
//				if (_isnan (det)) std::cout << "qnan" << std::endl;
//
//				return (false);
//			}
//			//float maxc = A.maxCoeff();
//
//			Eigen::Matrix<float, 6, 1> result = A.llt ().solve (b).cast<float>();
//			//Eigen::Matrix<float, 6, 1> result = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
//
//			float alpha = result (0);
//			float beta  = result (1);
//			float gamma = result (2);
//
//			Eigen::Matrix3f Rinc = (Eigen::Matrix3f)AngleAxisf (gamma, Vector3f::UnitZ ()) * AngleAxisf (beta, Vector3f::UnitY ()) * AngleAxisf (alpha, Vector3f::UnitX ());
//			Vector3f tinc = result.tail<3> ();
//
//			//compose
//			tcurr = Rinc * tcurr + tinc;
//			Rcurr = Rinc * Rcurr;
//		}
//	}
//
//	pPos->setRotation(Rcurr);
//	pPos->setTranslation(tcurr);
//
//	return true;
//}
