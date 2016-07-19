#include "NuiKinfuVolume.h"

#include "NuiKinfuTransform.h"
#include "Foundation/NuiDebugMacro.h"
#include "Foundation/NuiTimeLog.h"
#include "Foundation/NuiMatrixUtilities.h"
#include "NuiMarchingCubeTable.h"
#include "Shape/NuiCLMappableData.h"
#include "Shape\NuiMeshShape.h"

#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

static const std::string sVolume2Vertices("Volume2Vertices");

NuiKinfuVolume::NuiKinfuVolume()
	: m_volumeOutputVerticesCL(NULL)
	, m_volumeOutputColorsCL(NULL)
	, m_mutexCL(NULL)
	, m_vertexSumCL(NULL)
	, m_max_output_vertex_size(3000000)
	, m_dirty(true)
	, m_integration_metric_threshold(0.15f)
{
	AcquireBuffer(true);
	reset();
}

NuiKinfuVolume::~NuiKinfuVolume()
{
	ReleaseBuffer();
}

void NuiKinfuVolume::reset()
{
	m_lastIntegrationRotation.setIdentity();
	m_lastIntegrationTranslation = Vector3f::Zero();

	m_cachedPointCloud.clear();
	m_dirty = true;
}


void NuiKinfuVolume::AcquireBuffer(bool bHas_color_volume)
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();
	
	m_volumeOutputVerticesCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_WRITE_ONLY, m_max_output_vertex_size*3*sizeof(float), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	if(bHas_color_volume)
	{
		m_volumeOutputColorsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_WRITE_ONLY, m_max_output_vertex_size*4*sizeof(float), NULL, &err);
		NUI_CHECK_CL_ERR(err);
	}
	m_mutexCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_vertexSumCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}


void NuiKinfuVolume::ReleaseBuffer()
{
	if (m_volumeOutputVerticesCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_volumeOutputVerticesCL);
		NUI_CHECK_CL_ERR(err);
		m_volumeOutputVerticesCL = NULL;
	}
	if (m_volumeOutputColorsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_volumeOutputColorsCL);
		NUI_CHECK_CL_ERR(err);
		m_volumeOutputColorsCL = NULL;
	}
	if (m_mutexCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_mutexCL);
		NUI_CHECK_CL_ERR(err);
		m_mutexCL = NULL;
	}
	if (m_vertexSumCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_vertexSumCL);
		NUI_CHECK_CL_ERR(err);
		m_vertexSumCL = NULL;
	}
}
