#include "NuiKinfuTrackingEngine.h"

#include "NuiKinfuTSDFVolume.h"
#include "NuiPyramidICP.h"

#include <cmath>
#include "Foundation/NuiMatrixUtilities.h"
#include "Foundation/NuiDebugMacro.h"
#include "Shape/NuiCameraParams.h"

#include "Shape/NuiCLMappableData.h"
#include "DeviceSpecific/NuiKinfuOpenCLFrame.h"
#include "DeviceSpecific/NuiKinfuOpenCLDepthTracker.h"

#include "OpenCLUtilities/NuiMappable.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

#define KINFU_DEFAULT_DEPTH_FOCAL_X 370.f
#define KINFU_DEFAULT_DEPTH_FOCAL_Y 370.f

NuiKinfuTrackingEngine::NuiKinfuTrackingEngine(NuiTrackerConfig& tracerConfig, UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight)
	: m_tracker(NULL)
	, m_frame(NULL)
	, m_integration_metric_threshold(0.15f)
{
	m_frame = new NuiKinfuOpenCLFrame();
	m_frame->AcquireBuffers(nWidth, nHeight,nColorWidth, nColorHeight);

	m_tracker = new NuiKinfuOpenCLDepthTracker(tracerConfig, nWidth, nHeight);
	reset( Vector3f::Zero() );
}

NuiKinfuTrackingEngine::NuiKinfuTrackingEngine()
	: m_tracker(NULL)
	, m_frame(NULL)
	, m_integration_metric_threshold(0.15f)
{
	reset( Vector3f::Zero() );
}

NuiKinfuTrackingEngine::~NuiKinfuTrackingEngine()
{
	SafeDelete(m_tracker);
	SafeDelete(m_frame);
}

void NuiKinfuTrackingEngine::initialize(const NuiTrackerConfig& tracerConfig, UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight)
{
	if(!m_frame)
		m_frame = new NuiKinfuOpenCLFrame();
	m_frame->AcquireBuffers(nWidth, nHeight, nColorWidth, nColorHeight);

	if(!m_tracker)
		m_tracker = new NuiKinfuOpenCLDepthTracker(tracerConfig, nWidth, nHeight);
}

void NuiKinfuTrackingEngine::reset(const Vector3f& translateBasis)
{
	if (m_poses.size() > 0)
		std::cout << "Reset" << std::endl;

	m_poses.clear ();
	m_poses.reserve (30000);

	m_transform.setTransform(Matrix3frm::Identity(), translateBasis);
	m_lastIntegrationPos.setRotation(m_transform.getRotation());
	m_lastIntegrationPos.setTranslation(m_transform.getTranslation());
}

bool NuiKinfuTrackingEngine::log(const std::string& fileName) const
{
	return m_tracker ? m_tracker->log(fileName) : false;
}

float NuiKinfuTrackingEngine::getTrackerError() const
{
	return m_tracker ? m_tracker->getError() : 0.0f;
}

float NuiKinfuTrackingEngine::getTrackerCount() const
{
	return m_tracker ? m_tracker->getCount() : 0.0f;
}

const NuiCameraPos&	NuiKinfuTrackingEngine::getCameraPose (int time /*= -1*/) const
{
	if(m_poses.size() == 0)
		return m_lastIntegrationPos;
	
	if (time > (int)m_poses.size () || time < 0)
		time = (int)m_poses.size () - 1;

	return m_poses[time];
}

bool	NuiKinfuTrackingEngine::RunTracking(
	UINT16* pDepths,
	ColorSpacePoint* pDepthToColor,
	UINT nPointNum,
	const NuiColorImage& colorImage,
	NuiKinfuVolume*	pVolume,
	const NuiCameraParams& cameraParams)
{
	if(!m_tracker || !m_frame)
		return true;

	if(!pDepths || 0 == nPointNum)
		return true;

	// Build the frame buffers
	m_frame->UpdateCameraParams(cameraParams);
	m_frame->UpdateDepthBuffers(pDepths, nPointNum, cameraParams.m_sensorDepthMin, cameraParams.m_sensorDepthMax);
	if( m_tracker->hasColorData() ||
		(pVolume && pVolume->hasColorData()))
	{
		m_frame->UpdateColorBuffers(pDepthToColor, nPointNum, colorImage);
	}
	
	// Tracking
	UINT frameTime = (UINT)m_poses.size();
	if (frameTime == 0)
	{
		if( !m_tracker->readFrame(m_frame) )
			return false;

		m_tracker->transformPrevsFrame(&m_transform);
		if( pVolume )
		{
			pVolume->integrateVolume(m_floatDepthsCL, m_tracker->getNormalsCL(), colorsCL, m_cameraParamsCL, m_transform.getTransformCL(), m_nWidth, m_nHeight);
		}
	}
	else
	{
		if( !m_tracker->trackFrame(m_frame, &m_transform, NULL) )
			return false;
		// Debug
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
				pVolume->integrateVolume(m_floatDepthsCL, m_tracker->getNormalsCL(), colorsCL, m_cameraParamsCL, m_transform.getTransformCL(), m_nWidth, m_nHeight);
				m_lastIntegrationPos.setRotation(m_transform.getRotation());
				m_lastIntegrationPos.setTranslation(m_transform.getTranslation());
			}
			pVolume->raycastRender(m_tracker->getPrevVerticesCL(), m_tracker->getPrevNormalsCL(), m_tracker->getPrevIntensitiesCL(), m_cameraParamsCL, m_transform.getTransformCL(), m_nWidth, m_nHeight, camIntri.sensorDepthWorldMin, camIntri.sensorDepthWorldMax);
			m_tracker->resizePrevsFrame();
		}
		else
		{
			// Only ICP
			m_tracker->transformPrevsFrame(&m_transform);
		}
	}

	//save tranform
	NuiCameraPos camPos;
	camPos.setIntrinsics(cameraParams.m_intrinsics);
	camPos.setRotation(m_transform.getRotation());
	camPos.setTranslation(m_transform.getTranslation());
	m_poses.push_back( camPos );

	return true;
}

bool NuiKinfuTrackingEngine::previousBufferToData(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return false;

	if(!m_tracker)
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

	cl_mem prevVertices = m_tracker->getPrevVerticesCL();
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
	cl_mem prevIntensitiesCL = m_tracker->getPrevIntensitiesCL();
	if(prevIntensitiesCL)
	{
		if( nPointsNum != pCLData->ColorStream().size() )
		{
			NuiMappableAccessor::asVectorImpl(pCLData->ColorStream())->data().resize(nPointsNum);
		}

		// Get the kernel
		cl_kernel intensityKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_INTENSITY_TO_FLOAT4);
		assert(intensityKernel);
		if (intensityKernel && prevIntensitiesCL)
		{
			cl_mem colorsGL = NuiOpenCLBufferFactory::asColor4fBufferCL(pCLData->ColorStream());

			// Acquire OpenGL objects before use
			glObjs[0] = colorsGL;
			openclutil::enqueueAcquireHWObjects(
				sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

			// Set kernel arguments
			cl_uint idx = 0;
			err = clSetKernelArg(intensityKernel, idx++, sizeof(cl_mem), &prevIntensitiesCL);
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
		}
		else
		{
			NUI_ERROR("Get kernel 'E_INTENSITY_TO_FLOAT4' failed!\n");
		}
	}

	pCLData->SetStreamDirty(true);

	return true;
}

bool	NuiKinfuTrackingEngine::previousNormalImageToData(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return false;

	if(!m_tracker)
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

	cl_mem prevData = m_tracker->getPrevVerticesCL();
	if( m_nWidth != pCLData->ColorTex().width() || m_nHeight != pCLData->ColorTex().height())
	{
		NuiTextureMappableAccessor::updateImpl(
			pCLData->ColorTex(),
			m_nWidth,
			m_nHeight,
			NULL
			);
	}
	cl_mem texGL = NuiOpenCLBufferFactory::asTexture2DCL(pCLData->ColorTex());

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
	err = clSetKernelArg(rgbaKernel, idx++, sizeof(cl_mem), &prevData);
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
