#include "NuiKinfuTracker.h"

#include "NuiKinfuVolume.h"
#include "NuiPyramidICP.h"

#include <cmath>
#include "Foundation/NuiDebugMacro.h"
#include "Shape/NuiCLMappableData.h"
#include "OpenCLUtilities/NuiMappable.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

#define KINFU_DEFAULT_DEPTH_FOCAL_X 370.f
#define KINFU_DEFAULT_DEPTH_FOCAL_Y 370.f

NuiKinfuTracker::NuiKinfuTracker(NuiICPConfig& icpConfig, UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight)
	: m_icp(NULL)
	, m_positionsGL(NULL)
	, m_rawDepthsCL(NULL)
	, m_floatDepthsCL(NULL)
	, m_colorUVsCL(NULL)
	, m_colorImageCL(NULL)
	, m_colorsCL(NULL)
	, m_nWidth(nWidth)
	, m_nHeight(nHeight)
	, m_nColorWidth(nColorWidth)
	, m_nColorHeight(nColorHeight)
	, m_lastIntegrationFrame(0)
	, m_integration_metric_threshold(0.15f)
{
	m_icp = new NuiPyramidICP(icpConfig, m_nWidth, m_nHeight);

	AcquireBuffers(true);
	reset( Vector3f::Zero() );
}

NuiKinfuTracker::NuiKinfuTracker()
	: m_icp(NULL)
	, m_positionsGL(NULL)
	, m_rawDepthsCL(NULL)
	, m_floatDepthsCL(NULL)
	, m_colorUVsCL(NULL)
	, m_colorImageCL(NULL)
	, m_colorsCL(NULL)
	, m_nWidth(0)
	, m_nHeight(0)
	, m_nColorWidth(0)
	, m_nColorHeight(0)
	, m_lastIntegrationFrame(0)
	, m_integration_metric_threshold(0.15f)
{
	reset( Vector3f::Zero() );
}

NuiKinfuTracker::~NuiKinfuTracker()
{
	SafeDelete(m_icp);

	ReleaseBuffers();
	ReleaseGLBuffer();
}

void NuiKinfuTracker::initialize(NuiICPConfig& icpConfig, UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight)
{
	if(nWidth != m_nWidth || nHeight != m_nHeight || nColorWidth != m_nColorWidth || nColorHeight != m_nColorHeight)
	{
		SafeDelete(m_icp);

		ReleaseBuffers();
		ReleaseGLBuffer();
	}
	if(!m_icp)
	{
		m_nWidth = nWidth;
		m_nHeight = nHeight;
		m_nColorWidth = nColorWidth;
		m_nColorHeight = nColorHeight;
		m_icp = new NuiPyramidICP(icpConfig, m_nWidth, m_nHeight);
		AcquireBuffers(true);
	}
}

void NuiKinfuTracker::reset(const Vector3f& translateBasis)
{
	if (m_frames.size() > 0)
		std::cout << "Reset" << std::endl;

	m_frames.clear ();
	m_frames.reserve (30000);

	m_currPos.setRotation(Matrix3frm::Identity());
	m_currPos.setTranslation( translateBasis );
	m_lastIntegrationFrame = 0;
}

float NuiKinfuTracker::getIcpError() const
{
	return m_icp ? m_icp->getError() : 0.0f;
}

float NuiKinfuTracker::getIcpCount() const
{
	return m_icp ? m_icp->getCount() : 0.0f;
}

const NuiCameraParams&	NuiKinfuTracker::getCameraPose (int time /*= -1*/) const
{
	if(0 == m_frames.size())
	{
		return m_currPos;
	}
	if (time > (int)m_frames.size () || time < 0)
		time = (int)m_frames.size () - 1;

	return m_frames[time];
}

void NuiKinfuTracker::AcquireBuffers(bool bHasColor)
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();
	m_rawDepthsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY, m_nWidth*m_nHeight*sizeof(UINT16), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_floatDepthsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_nWidth*m_nHeight*sizeof(cl_float), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	if(bHasColor)
	{
		m_colorUVsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY, m_nWidth*m_nHeight*sizeof(ColorSpacePoint), NULL, &err);
		NUI_CHECK_CL_ERR(err);
		/*cl_image_format colorImgFormat;
		colorImgFormat.image_channel_order = CL_BGRA;
		colorImgFormat.image_channel_data_type = CL_UNSIGNED_INT8;
		m_colorImageCL = NuiGPUMemManager::instance().CreateImage2DCL(context, CL_MEM_READ_ONLY, &colorImgFormat, m_nColorWidth, m_nColorHeight, 0, NULL, &err);
		NUI_CHECK_CL_ERR(err);*/
		m_colorImageCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY, m_nColorWidth*m_nColorHeight*sizeof(BGRQUAD), NULL, &err);
		NUI_CHECK_CL_ERR(err);
		m_colorsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_nWidth*m_nHeight*sizeof(BGRQUAD), NULL, &err);
		NUI_CHECK_CL_ERR(err);
	}
}

bool NuiKinfuTracker::AcquireGLBuffer(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return false;

	m_positionsGL =
		NuiOpenCLBufferFactory::asPosition3fBufferCL(pCLData->PositionStream());
	if (!m_positionsGL)
	{
		NUI_ERROR("Failed to get positions of the buffer\n");
		return false;
	}

	// Acquire OpenGL objects before use
	cl_mem glObjs[] = {
		m_positionsGL
	};
	openclutil::enqueueAcquireHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

	return true;
}

void NuiKinfuTracker::ReleaseBuffers()
{
	if (m_rawDepthsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_rawDepthsCL);
		NUI_CHECK_CL_ERR(err);
		m_rawDepthsCL = NULL;
	}
	if (m_floatDepthsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_floatDepthsCL);
		NUI_CHECK_CL_ERR(err);
		m_floatDepthsCL = NULL;
	}
	if (m_colorUVsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_colorUVsCL);
		NUI_CHECK_CL_ERR(err);
		m_colorUVsCL = NULL;
	}
	if (m_colorImageCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_colorImageCL);
		NUI_CHECK_CL_ERR(err);
		m_colorImageCL = NULL;
	}
	if (m_colorsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_colorsCL);
		NUI_CHECK_CL_ERR(err);
		m_colorsCL = NULL;
	}
}

void NuiKinfuTracker::ReleaseGLBuffer()
{
	if(!m_positionsGL)
		return;
	assert(m_positionsGL);

	cl_mem glObjs[] = {
		m_positionsGL
	};
	// Release OpenGL objects
	openclutil::enqueueReleaseHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

	m_positionsGL = NULL;
}

void NuiKinfuTracker::WriteDepths(UINT16* pDepths, UINT nPointsNum, UINT16 minDepth, UINT16 maxDepth)
{
	assert(m_nWidth*m_nHeight == nPointsNum);

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

#ifdef _GPU_PROFILER
	cl_event timing_event;
	cl_ulong time_start, time_end;
#endif
	clEnqueueWriteBuffer(
		queue,
		m_rawDepthsCL,
#ifdef _GPU_PROFILER
		CL_TRUE,
#else
		CL_FALSE,//blocking
#endif
		0,
		nPointsNum * sizeof(UINT16),
		pDepths,
		0,
		NULL,
#ifdef _GPU_PROFILER
		&timing_event
#else
		NULL
#endif
	);
	NUI_CHECK_CL_ERR(err);

#ifdef _GPU_PROFILER
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_START, sizeof(time_start), &time_start, NULL);
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_END, sizeof(time_end), &time_end, NULL);
	std::cout << "write depth:" << (time_end - time_start) << std::endl;
	clReleaseEvent(timing_event);
#endif

	// filter the input depth map
	PassingDepths((float)minDepth/1000.0f, (float)maxDepth/1000.0f);
}

void NuiKinfuTracker::WriteColors(ColorSpacePoint* pDepthToColor, const NuiColorImage& image, UINT nPointsNum)
{
	assert(m_nWidth*m_nHeight == nPointsNum);
	if(!pDepthToColor || !image.GetBuffer() || !m_colorUVsCL || !m_colorImageCL || !m_colorsCL)
		return;

	// Get the kernel
	cl_kernel uv2colorKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_UV2COLOR);
	assert(uv2colorKernel);
	if (!uv2colorKernel)
	{
		NUI_ERROR("Get kernel 'E_UV2COLOR' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

#ifdef _GPU_PROFILER
	cl_event timing_event;
	cl_ulong time_start, time_end;
#endif
	clEnqueueWriteBuffer(
		queue,
		m_colorUVsCL,
#ifdef _GPU_PROFILER
		CL_TRUE,
#else
		CL_FALSE,//blocking
#endif
		0,
		nPointsNum * sizeof(ColorSpacePoint),
		pDepthToColor,
		0,
		NULL,
#ifdef _GPU_PROFILER
		&timing_event
#else
		NULL
#endif
		);
	NUI_CHECK_CL_ERR(err);

#ifdef _GPU_PROFILER
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_START, sizeof(time_start), &time_start, NULL);
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_END, sizeof(time_end), &time_end, NULL);
	std::cout << "write color:" << (time_end - time_start) << std::endl;
	clReleaseEvent(timing_event);
#endif

	//////////////////////////
	// Write Color Image
#ifdef _GPU_PROFILER
	cl_event timing_event2;
#endif
	clEnqueueWriteBuffer(
		queue,
		m_colorImageCL,
#ifdef _GPU_PROFILER
		CL_TRUE,
#else
		CL_FALSE,//blocking
#endif
		0,
		image.GetWidth() * image.GetHeight() * sizeof(BGRQUAD),
		image.GetBuffer(),
		0,
		NULL,
#ifdef _GPU_PROFILER
		&timing_event2
#else
		NULL
#endif
		);
	NUI_CHECK_CL_ERR(err);

#ifdef _GPU_PROFILER
	clGetEventProfilingInfo(timing_event2, CL_PROFILING_COMMAND_START, sizeof(time_start), &time_start, NULL);
	clGetEventProfilingInfo(timing_event2, CL_PROFILING_COMMAND_END, sizeof(time_end), &time_end, NULL);
	std::cout << "write color image:" << (time_end - time_start) << std::endl;
	clReleaseEvent(timing_event2);
#endif

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(uv2colorKernel, idx++, sizeof(cl_mem), &m_colorUVsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(uv2colorKernel, idx++, sizeof(cl_mem), &m_colorImageCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(uv2colorKernel, idx++, sizeof(cl_int), &m_nColorWidth);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(uv2colorKernel, idx++, sizeof(cl_int), &m_nColorHeight);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(uv2colorKernel, idx++, sizeof(cl_mem), &m_colorsCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { m_nWidth, m_nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		uv2colorKernel,
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

void NuiKinfuTracker::PassingDepths(float nearPlane, float farPlane)
{
	// Get the kernel
	cl_kernel passingKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_PASSING_FILTER);
	assert(passingKernel);
	if (!passingKernel)
	{
		NUI_ERROR("Get kernel 'E_BILATERAL_FILTER' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(passingKernel, idx++, sizeof(cl_mem), &m_rawDepthsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(passingKernel, idx++, sizeof(cl_mem), &m_floatDepthsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(passingKernel, idx++, sizeof(float), &nearPlane);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(passingKernel, idx++, sizeof(float), &farPlane);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[1] = { m_nWidth * m_nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		passingKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
}

Vector3f rodrigues2(const Eigen::Matrix3f& matrix)
{
	Eigen::JacobiSVD<Eigen::Matrix3f> svd(matrix, Eigen::ComputeFullV | Eigen::ComputeFullU);    
	Eigen::Matrix3f R = svd.matrixU() * svd.matrixV().transpose();

	double rx = R(2, 1) - R(1, 2);
	double ry = R(0, 2) - R(2, 0);
	double rz = R(1, 0) - R(0, 1);

	double s = sqrt((rx*rx + ry*ry + rz*rz)*0.25);
	double c = (R.trace() - 1) * 0.5;
	c = c > 1. ? 1. : c < -1. ? -1. : c;

	double theta = acos(c);

	if( s < 1e-5 )
	{
		double t;

		if( c > 0 )
			rx = ry = rz = 0;
		else
		{
			t = (R(0, 0) + 1)*0.5;
			rx = sqrt( std::max(t, 0.0) );
			t = (R(1, 1) + 1)*0.5;
			ry = sqrt( std::max(t, 0.0) ) * (R(0, 1) < 0 ? -1.0 : 1.0);
			t = (R(2, 2) + 1)*0.5;
			rz = sqrt( std::max(t, 0.0) ) * (R(0, 2) < 0 ? -1.0 : 1.0);

			if( fabs(rx) < fabs(ry) && fabs(rx) < fabs(rz) && (R(1, 2) > 0) != (ry*rz > 0) )
				rz = -rz;
			theta /= sqrt(rx*rx + ry*ry + rz*rz);
			rx *= theta;
			ry *= theta;
			rz *= theta;
		}
	}
	else
	{
		double vth = 1/(2*s);
		vth *= theta;
		rx *= vth; ry *= vth; rz *= vth;
	}
	return Eigen::Vector3d(rx, ry, rz).cast<float>();
}

/** \brief Function that integrates volume if volume element contains: 2 bytes for round(tsdf*SHORT_MAX) and 2 bytes for integer weight.*/
void    NuiKinfuTracker::IntegrateTsdfVolume(NuiKinfuVolume*	pVolume)
{
	if(!pVolume || !m_icp)
		return;

	// Get the kernel
	cl_kernel scaleDepthsKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_SCALE_DEPTHS);
	assert(scaleDepthsKernel);
	if (!scaleDepthsKernel)
	{
		NUI_ERROR("Get kernel 'E_SCALE_DEPTHS' failed!\n");
		return;
	}

	cl_kernel integrateKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_INTEGRATE_TSDF_VOLUME);
	assert(integrateKernel);
	if (!integrateKernel)
	{
		NUI_ERROR("Get kernel 'E_INTEGRATE_TSDF_VOLUME' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	const NuiCameraIntrinsics& intri = m_currPos.getIntrinsics();
	float fx_inv = 1.f / intri.m_fx;
	float fy_inv = 1.f / intri.m_fy;

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(scaleDepthsKernel, idx++, sizeof(cl_mem), &m_floatDepthsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(scaleDepthsKernel, idx++, sizeof(cl_float), &fx_inv);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(scaleDepthsKernel, idx++, sizeof(cl_float), &fy_inv);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(scaleDepthsKernel, idx++, sizeof(cl_float), &intri.m_cx);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(scaleDepthsKernel, idx++, sizeof(cl_float), &intri.m_cy);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { m_nWidth, m_nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		scaleDepthsKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
	);
	NUI_CHECK_CL_ERR(err);

	Matrix3frm Rcurr_inv = m_currPos.getRotation().inverse();

	cl_mem currentNormals = m_icp->getNormals();
	const Vector3i&	resolutions = pVolume->getResolution();
	cl_mem volume_data = pVolume->data();
	cl_mem color_volume_data = pVolume->colorData();
	cl_mem volume_param_data = pVolume->paramData();
	cl_uchar color_max_weight = pVolume->getMaxColorWeight();
	Vector3i voxelWrap = pVolume->getVoxelWrap();

	// Set kernel arguments
	idx = 0;
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &m_floatDepthsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &currentNormals);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_int), &m_nWidth);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_int), &m_nHeight);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &m_colorsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_float)*8, Rcurr_inv.data());
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_float), Rcurr_inv.data()+8);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_float)*4, m_currPos.getTranslation().data());
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_float), &intri.m_fx);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_float), &intri.m_fy);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_float), &intri.m_cx);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_float), &intri.m_cy);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_int3), voxelWrap.data());
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &volume_data);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &volume_param_data);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &color_volume_data);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_uchar), &color_max_weight);
	NUI_CHECK_CL_ERR(err);

#ifdef _GPU_PROFILER
	cl_event timing_event;
	cl_ulong time_start, time_end;
#endif

	// Run kernel to calculate 
	kernelGlobalSize[0] = (size_t)resolutions[0];
	kernelGlobalSize[1] = (size_t)resolutions[1];
	err = clEnqueueNDRangeKernel(
		queue,
		integrateKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
#ifdef _GPU_PROFILER
		&timing_event
#else
		NULL
#endif
	);
	NUI_CHECK_CL_ERR(err);

#ifdef _GPU_PROFILER
	clFinish(queue);
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_START, sizeof(time_start), &time_start, NULL);
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_END, sizeof(time_end), &time_end, NULL);
	std::cout << "integration:" << (time_end - time_start) << std::endl;
	clReleaseEvent(timing_event);
#endif

}

void    NuiKinfuTracker::RayCast(NuiKinfuVolume*	pVolume)
{
	if(!pVolume || !m_icp)
		return;

	// Get the kernel
	cl_kernel raycastKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_RAY_CAST);
	assert(raycastKernel);
	if (!raycastKernel)
	{
		NUI_ERROR("Get kernel 'E_RAY_CAST' failed!\n");
		return;
	}

	cl_mem volume_data = pVolume->data();
	cl_mem volume_param_data = pVolume->paramData();
	
	const NuiCameraIntrinsics& intri = m_currPos.getIntrinsics();
	float fx_inv = 1.f / intri.m_fx;
	float fy_inv = 1.f / intri.m_fy;
	Vector3i voxelWrap = pVolume->getVoxelWrap();

	cl_mem prevVertices = m_icp->getPrevVertices();
	cl_mem prevNormals = m_icp->getPrevNormals();

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &volume_data);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &volume_param_data);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float)*8, m_currPos.getRotation().data());
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), m_currPos.getRotation().data()+8);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float)*4, m_currPos.getTranslation().data());
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), &fx_inv);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), &fy_inv);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), &intri.m_cx);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), &intri.m_cy);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_int3), voxelWrap.data());
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &prevVertices);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &prevNormals);
	NUI_CHECK_CL_ERR(err);

#ifdef _GPU_PROFILER
	cl_event timing_event;
	cl_ulong time_start, time_end;
#endif
	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { m_nWidth, m_nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		raycastKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
#ifdef _GPU_PROFILER
		&timing_event
#else
		NULL
#endif
	);
	NUI_CHECK_CL_ERR(err);

#ifdef _GPU_PROFILER
	clFinish(queue);
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_START, sizeof(time_start), &time_start, NULL);
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_END, sizeof(time_end), &time_end, NULL);
	std::cout << "raycast:" << (time_end - time_start) << std::endl;
	clReleaseEvent(timing_event);
#endif
}

bool	NuiKinfuTracker::RunTracking(
	UINT16* pDepths,
	UINT nPointNum,
	UINT16 minDepth,
	UINT16 maxDepth,
	ColorSpacePoint* pDepthToColor,
	const NuiColorImage& colorImage,
	NuiKinfuVolume*	pVolume,
	float intri_fx,
	float intri_fy,
	float intri_cx,
	float intri_cy)
{
	if(!m_icp)
		return true;

	if(!pDepths || 0 == nPointNum)
		return true;

	m_currPos.setIntrinsics(NuiCameraIntrinsics(
		intri_fx,
		intri_fy,
		intri_cx,
		intri_cy ));

	WriteDepths(pDepths, nPointNum, minDepth, maxDepth);
	// half sample the input depth maps into the pyramid levels
	m_icp->input(m_floatDepthsCL, m_currPos);

	UINT frameTime = (UINT)m_frames.size();
	if (frameTime == 0)
	{
		m_icp->transformPrevs(m_currPos);
		if( pVolume )
		{
			if( pVolume->hasColorData() )
				WriteColors(pDepthToColor, colorImage, nPointNum);
			IntegrateTsdfVolume(pVolume);
			m_lastIntegrationFrame = frameTime;
			pVolume->setDirty();
		}
	}
	else
	{
		if( !m_icp->run(&m_currPos, NULL) )
			return false;

		if( pVolume )
		{
			// Shift
			m_currPos.setTranslation( pVolume->shiftVolume(m_currPos.getTranslation()) );

			// Integrate
			if(m_lastIntegrationFrame < frameTime)
			{
				const Matrix3frm& Rprev = m_frames[m_lastIntegrationFrame].getRotation();
				const Vector3f&   tprev = m_frames[m_lastIntegrationFrame].getTranslation();

				///////////////////////////////////////////////////////////////////////////////////////////
				// Integration check - We do not integrate volume if camera does not move.  
				float rnorm = rodrigues2(m_currPos.getRotation().inverse() * Rprev).norm();
				float tnorm = (m_currPos.getTranslation() - tprev).norm();
				const float alpha = 1.f;
				bool integrate = (rnorm + alpha * tnorm)/2 >= m_integration_metric_threshold;

				if (integrate)
				{
					if( pVolume->hasColorData() )
						WriteColors(pDepthToColor, colorImage, nPointNum);
					IntegrateTsdfVolume(pVolume);
					m_lastIntegrationFrame = frameTime;
					pVolume->setDirty();
				}
			}
			RayCast(pVolume);
			m_icp->resizePrevs();
		}
		else
		{
			// Only ICP
			m_icp->transformPrevs(m_currPos);
		}
	}

	//save tranform
	m_frames.push_back( m_currPos );

	return true;
}

bool NuiKinfuTracker::PreviousBuffer(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return false;

	if(!m_icp)
		return false;

	/*if ( !AcquireGLBuffer(pCLData) )
	{
		return false;
	}*/

	const UINT nPointsNum = m_nWidth * m_nHeight;

	std::vector<unsigned int>& clTriangleIndices =
		NuiMappableAccessor::asVectorImpl(pCLData->TriangleIndices())->data();
	clTriangleIndices.clear();

	std::vector<unsigned int>& clWireframeIndices =
		NuiMappableAccessor::asVectorImpl(pCLData->WireframeIndices())->data();
	clWireframeIndices.clear();

	std::vector<unsigned int>& clPointIndices =
		NuiMappableAccessor::asVectorImpl(pCLData->PointIndices())->data();
	if(clPointIndices.size() != nPointsNum)
	{
		clPointIndices.resize(nPointsNum);
		for (UINT i = 0; i < nPointsNum; ++i)
		{
			clPointIndices[i] = i;
		}
		pCLData->SetIndexingDirty(true);
	}
	
	// OpenCL command queue and device
	/*cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	err = clEnqueueCopyBuffer(
		queue,
		m_verticesArrCL[0],
		m_positionsGL,
		0,
		0,
		nPointsNum * 3 * sizeof(float),
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
	pCLData->SetStreamDirty(false);*/

	std::vector<SgVec3f>& positions = NuiMappableAccessor::asVectorImpl(pCLData->PositionStream())->data();
	if(positions.size() != nPointsNum)
		positions.resize(nPointsNum);

	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	cl_mem prevVertices = m_icp->getPrevVertices();

	clEnqueueReadBuffer(
		queue,
		prevVertices,
		CL_TRUE,//blocking
		0,
		nPointsNum * 3 * sizeof(float),
		positions.data(),
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	pCLData->SetStreamDirty(true);

	//ReleaseGLBuffer();

	return true;
}


