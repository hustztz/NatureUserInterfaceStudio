#include "NuiKinfuTracker.h"

#include "NuiKinfuTSDFVolume.h"
#include "NuiPyramidICP.h"

#include <cmath>
#include "Foundation/NuiMatrixUtilities.h"
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
	, m_rawDepthsCL(NULL)
	, m_floatDepthsCL(NULL)
	, m_colorUVsCL(NULL)
	, m_colorImageCL(NULL)
	, m_colorsCL(NULL)
	, m_cameraParamsCL(NULL)
	, m_outputColorImageCL(NULL)
	, m_integration_metric_threshold(0.15f)
	, m_nWidth(nWidth)
	, m_nHeight(nHeight)
	, m_nColorWidth(nColorWidth)
	, m_nColorHeight(nColorHeight)
{
	m_icp = new NuiPyramidICP(icpConfig, m_nWidth, m_nHeight);

	AcquireBuffers(true);
	reset( Vector3f::Zero() );
}

NuiKinfuTracker::NuiKinfuTracker()
	: m_icp(NULL)
	, m_rawDepthsCL(NULL)
	, m_floatDepthsCL(NULL)
	, m_colorUVsCL(NULL)
	, m_colorImageCL(NULL)
	, m_colorsCL(NULL)
	, m_cameraParamsCL(NULL)
	, m_outputColorImageCL(NULL)
	, m_integration_metric_threshold(0.15f)
	, m_nWidth(0)
	, m_nHeight(0)
	, m_nColorWidth(0)
	, m_nColorHeight(0)
{
	reset( Vector3f::Zero() );
}

NuiKinfuTracker::~NuiKinfuTracker()
{
	SafeDelete(m_icp);

	ReleaseBuffers();
}

void NuiKinfuTracker::initialize(const NuiICPConfig& icpConfig, UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight)
{
	if(nWidth != m_nWidth || nHeight != m_nHeight || nColorWidth != m_nColorWidth || nColorHeight != m_nColorHeight)
	{
		SafeDelete(m_icp);

		ReleaseBuffers();
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

	m_transform.setTransform(Matrix3frm::Identity(), translateBasis);
	m_lastIntegrationPos.setRotation(m_transform.getRotation());
	m_lastIntegrationPos.setTranslation(m_transform.getTranslation());
}

bool NuiKinfuTracker::log(const std::string& fileName) const
{
	return m_icp ? m_icp->log(fileName) : false;
}

float NuiKinfuTracker::getIcpError() const
{
	return m_icp ? m_icp->getError() : 0.0f;
}

float NuiKinfuTracker::getIcpCount() const
{
	return m_icp ? m_icp->getCount() : 0.0f;
}

const NuiCameraPos&	NuiKinfuTracker::getCameraPose (int time /*= -1*/) const
{
	if(m_frames.size() == 0)
		return m_lastIntegrationPos;
	
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
	m_cameraParamsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY, sizeof(NuiCLCameraParams), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	// Output
	m_outputColorImageCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_nWidth*m_nHeight*sizeof(BGRQUAD), NULL, &err);
	NUI_CHECK_CL_ERR(err);
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
	if (m_cameraParamsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_cameraParamsCL);
		NUI_CHECK_CL_ERR(err);
		m_cameraParamsCL = NULL;
	}
	if (m_outputColorImageCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_outputColorImageCL);
		NUI_CHECK_CL_ERR(err);
		m_outputColorImageCL = NULL;
	}
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

void NuiKinfuTracker::WriteCameraParams(const NuiCLCameraParams& camIntri)
{
	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	clEnqueueWriteBuffer(
		queue,
		m_cameraParamsCL,
		CL_FALSE,//blocking
		0,
		sizeof(NuiCLCameraParams),
		&camIntri,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
}

void NuiKinfuTracker::WriteColors(ColorSpacePoint* pDepthToColor, const NuiColorImage& image, UINT nPointsNum, cl_mem colorsCL)
{
	assert(m_nWidth*m_nHeight == nPointsNum);
	if(!pDepthToColor || !image.GetBuffer() || !m_colorUVsCL || !m_colorImageCL || !colorsCL)
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
	err = clSetKernelArg(uv2colorKernel, idx++, sizeof(cl_mem), &colorsCL);
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

	NuiCLCameraParams camIntri;
	camIntri.fx = intri_fx;
	camIntri.fx_inv = 1 / intri_fx;
	camIntri.fy = intri_fy;
	camIntri.fy_inv = 1 / intri_fy;
	camIntri.cx = intri_cx;
	camIntri.cy = intri_cy;
	camIntri.depthImageWidth = m_nWidth;
	camIntri.depthImageHeight = m_nHeight;
	camIntri.sensorDepthWorldMin = (float)minDepth / 1000.0f;
	camIntri.sensorDepthWorldMax = (float)maxDepth / 1000.0f;
	WriteCameraParams(camIntri);
	WriteDepths(pDepths, nPointNum, minDepth, maxDepth);
	if( m_icp->getColorsCL() )
		WriteColors(pDepthToColor, colorImage, nPointNum, m_icp->getColorsCL());

	// half sample the input depth maps into the pyramid levels
	m_icp->input(m_floatDepthsCL, m_cameraParamsCL);

	UINT frameTime = (UINT)m_frames.size();
	if (frameTime == 0)
	{
		m_icp->transformPrevs(m_transform.getTransformCL());
		if( pVolume )
		{
			cl_mem colorsCL = NULL;
			if( pVolume->hasColorData() )
			{
				if( m_icp->getColorsCL() )
				{
					colorsCL = m_icp->getColorsCL();
				}
				else
				{
					WriteColors(pDepthToColor, colorImage, nPointNum, m_colorsCL);
					colorsCL = m_colorsCL;
				}
			}
			pVolume->integrateVolume(m_floatDepthsCL, m_icp->getNormalsCL(), colorsCL, m_cameraParamsCL, m_transform.getTransformCL(), m_nWidth, m_nHeight);
		}
	}
	else
	{
		if( !m_icp->run(m_cameraParamsCL, &m_transform, NULL) )
			return false;
		//m_transform.setTransform(Matrix3frm::Identity(), Vector3f::Zero());
		if( pVolume )
		{
			///////////////////////////////////////////////////////////////////////////////////////////
			// Integration check - We do not integrate volume if camera does not move.  
			float rnorm = NuiMatrixUtilities::rodrigues2(m_transform.getRotation().inverse() * m_lastIntegrationPos.getRotation()).norm();
			float tnorm = (m_transform.getTranslation() - m_lastIntegrationPos.getTranslation()).norm();
			const float alpha = 1.f;
			bool integrate = (rnorm + alpha * tnorm)/2 >= m_integration_metric_threshold;
			// Integrate
			if (integrate)
			{
				cl_mem colorsCL = NULL;
				if( pVolume->hasColorData() )
				{
					if( m_icp->getColorsCL() )
					{
						colorsCL = m_icp->getColorsCL();
					}
					else
					{
						WriteColors(pDepthToColor, colorImage, nPointNum, m_colorsCL);
						colorsCL = m_colorsCL;
					}
				}
				pVolume->integrateVolume(m_floatDepthsCL, m_icp->getNormalsCL(), colorsCL, m_cameraParamsCL, m_transform.getTransformCL(), m_nWidth, m_nHeight);
				m_lastIntegrationPos.setRotation(m_transform.getRotation());
				m_lastIntegrationPos.setTranslation(m_transform.getTranslation());
			}
			pVolume->raycastRender(m_icp->getPrevVerticesCL(), m_icp->getPrevNormalsCL(), m_icp->getPrevColorsCL(), m_cameraParamsCL, m_transform.getTransformCL(), m_nWidth, m_nHeight);
			m_icp->resizePrevs();
		}
		else
		{
			// Only ICP
			m_icp->transformPrevs(m_transform.getTransformCL());
		}
	}

	//save tranform
	NuiCameraPos camPos;
	camPos.setIntrinsics(NuiCameraIntrinsics(
		intri_fx,
		intri_fy,
		intri_cx,
		intri_cy ));
	camPos.setRotation(m_transform.getRotation());
	camPos.setTranslation(m_transform.getTranslation());
	m_frames.push_back( camPos );

	return true;
}

bool NuiKinfuTracker::previousBufferToData(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return false;

	if(!m_icp)
		return false;

	const UINT nPointsNum = m_nWidth * m_nHeight;

	pCLData->SetBoundingBox(SgVec3f(-256.0f / 370.0f, -212.0f / 370.0f, 0.4f),
		SgVec3f((m_nWidth-256.0f) / 370.0f, (m_nHeight-212.0f) / 370.0f, 4.0f));

	NuiMappableAccessor::asVectorImpl(pCLData->TriangleIndices())->data().clear();
	NuiMappableAccessor::asVectorImpl(pCLData->WireframeIndices())->data().clear();

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
	
	if( nPointsNum != pCLData->PositionStream().size() )
	{
		NuiMappableAccessor::asVectorImpl(pCLData->PositionStream())->data().resize(nPointsNum);
	}

	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	// 
	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	cl_mem prevVertices = m_icp->getPrevVerticesCL();
	cl_mem positionsGL = NuiOpenCLBufferFactory::asPosition3fBufferCL(pCLData->PositionStream());
	// Acquire OpenGL objects before use
	cl_mem glObjs[] = {
		positionsGL
	};

	openclutil::enqueueAcquireHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

	err = clEnqueueCopyBuffer(
		queue,
		prevVertices,
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

	// Output colors
	cl_mem prevColorsCL = m_icp->getPrevColorsCL();
	if(prevColorsCL)
	{
		if( nPointsNum != pCLData->ColorStream().size() )
		{
			NuiMappableAccessor::asVectorImpl(pCLData->ColorStream())->data().resize(nPointsNum);
		}

		// Get the kernel
		cl_kernel rgbaKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_RGBA_TO_FLOAT4);
		assert(rgbaKernel);
		if (rgbaKernel && prevColorsCL)
		{
			cl_mem colorsGL = NuiOpenCLBufferFactory::asColor4fBufferCL(pCLData->ColorStream());

			// Acquire OpenGL objects before use
			glObjs[0] = colorsGL;
			openclutil::enqueueAcquireHWObjects(
				sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

			// Set kernel arguments
			cl_uint idx = 0;
			err = clSetKernelArg(rgbaKernel, idx++, sizeof(cl_mem), &prevColorsCL);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(rgbaKernel, idx++, sizeof(cl_mem), &colorsGL);
			NUI_CHECK_CL_ERR(err);

			size_t kernelGlobalSize[1] = { nPointsNum };
			err = clEnqueueNDRangeKernel(
				queue,
				rgbaKernel,
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
		}
		else
		{
			NUI_ERROR("Get kernel 'E_RGBA_TO_FLOAT4' failed!\n");
		}
	}

	pCLData->SetStreamDirty(true);

	return true;
}

bool	NuiKinfuTracker::previousNormalImageToData(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return false;

	if(!m_icp)
		return false;

	// Get the kernel
	cl_kernel rgbaKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_FLOAT3_TO_RGBA);
	assert(rgbaKernel);
	if (!rgbaKernel)
	{
		NUI_ERROR("Get kernel 'E_FLOAT3_TO_RGBA' failed!\n");
		return false;
	}

	cl_mem prevNormals = m_icp->getPrevNormalsCL();

	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	// 
	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(rgbaKernel, idx++, sizeof(cl_mem), &prevNormals);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(rgbaKernel, idx++, sizeof(cl_mem), &m_outputColorImageCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[1] = { m_nWidth * m_nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		rgbaKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	BGRQUAD* normalBuffer = pCLData->GetColorImage().AllocateBuffer(m_nWidth, m_nHeight);
	clEnqueueReadBuffer(
		queue,
		m_outputColorImageCL,
		CL_FALSE,//blocking
		0,
		pCLData->GetColorImage().GetBufferSize(),
		normalBuffer,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	return true;
}
