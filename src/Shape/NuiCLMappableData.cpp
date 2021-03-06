#include "NuiCLMappableData.h"

NuiCLMappableData::NuiCLMappableData()
	: m_pointIndices("pointIndex")
	, m_triangleIndices("triangleIndex")
	, m_wireframeIndices("wireframeIndex")
	, m_positionStream("position")
	, m_colorStream("color")
	, m_normalStream("normal")
	, m_patchUVStream("UV")
	, m_streamDirty(true)
	, m_indexDirty(true)
{
}

NuiCLMappableData::~NuiCLMappableData()
{
	Clear();
}

void NuiCLMappableData::Clear()
{
	std::shared_ptr<NuiVectorMappableImplui> pointIndices = NuiMappableAccessor::asVectorImpl(PointIndices());
	pointIndices->data().clear();
	NuiMappableAccessor::reset(m_pointIndices);
	std::shared_ptr<NuiVectorMappableImplui> triangleIndices = NuiMappableAccessor::asVectorImpl(TriangleIndices());
	triangleIndices->data().clear();
	NuiMappableAccessor::reset(m_triangleIndices);
	std::shared_ptr<NuiVectorMappableImplui> wireframeIndices = NuiMappableAccessor::asVectorImpl(WireframeIndices());
	wireframeIndices->data().clear();
	NuiMappableAccessor::reset(m_wireframeIndices);
	std::shared_ptr<NuiVectorMappableImpl3f> positions = NuiMappableAccessor::asVectorImpl(PositionStream());
	positions->data().clear();
	NuiMappableAccessor::reset(m_positionStream);
	std::shared_ptr<NuiVectorMappableImpl4f> colors = NuiMappableAccessor::asVectorImpl(ColorStream());
	colors->data().clear();
	NuiMappableAccessor::reset(m_colorStream);
	std::shared_ptr<NuiVectorMappableImpl3f> normals = NuiMappableAccessor::asVectorImpl(NormalStream());
	normals->data().clear();
	NuiMappableAccessor::reset(m_normalStream);
	std::shared_ptr<NuiVectorMappableImpl2f> uvs = NuiMappableAccessor::asVectorImpl(PatchUVStream());
	uvs->data().clear();
	NuiMappableAccessor::reset(m_patchUVStream);

	NuiTextureMappableAccessor::reset(ColorTex());
	NuiTextureMappableAccessor::reset(FeedbackTex());

	m_boundingBoxMin.setValue(0.0f, 0.0f, 0.0f);
	m_boundingBoxMax.setValue(0.0f, 0.0f, 0.0f);

	m_streamDirty = true;
	m_indexDirty = true;
}

void NuiCLMappableData::DeepCopy (const NuiCLMappableData& other)
{
	m_nWidthStep = other.m_nWidthStep;
	m_pointIndices = other.m_pointIndices;
	m_triangleIndices = other.m_triangleIndices;
	m_wireframeIndices = other.m_wireframeIndices;
	m_positionStream = other.m_positionStream;
	m_colorStream = other.m_colorStream;
	m_normalStream = other.m_normalStream;
	m_patchUVStream = other.m_patchUVStream;

	m_colorTex = other.m_colorTex;
	m_feedbackTex = other.m_feedbackTex;
	m_camPos = other.m_camPos;

	m_boundingBoxMin = other.m_boundingBoxMin;
	m_boundingBoxMax = other.m_boundingBoxMax;

	//m_mesh = other.m_mesh;

	m_streamDirty = other.m_streamDirty;
	m_indexDirty = other.m_indexDirty;
}

void		NuiCLMappableData::SetCameraParams(const NuiCameraParams& params)
{
	m_camPos.setIntrinsics(params.m_intrinsics);
	m_camPos.setSensorDepthRange(params.m_sensorDepthMin, params.m_sensorDepthMax);
}

const float*	NuiCLMappableData::GetPositionValue(UINT idx) const
{
	NuiScopedMapConstBuffer<SgVec3f> positions(m_positionStream);
	if(idx >= positions.size())
		return NULL;

	return positions[idx].getValue();
}

bool	NuiCLMappableData::SetPositionValue(UINT idx, float x, float y, float z)
{
	NuiScopedMapBuffer<SgVec3f> positions(m_positionStream);
	if(idx >= positions.size())
		return false;

	positions[idx].setValue(x, y, z);
	return true;
}

const float*	NuiCLMappableData::GetNormalValue(UINT idx) const
{
	NuiScopedMapConstBuffer<SgVec3f> normals(m_normalStream);
	if(idx >= normals.size())
		return NULL;

	return normals[idx].getValue();
}

bool	NuiCLMappableData::SetNormalValue(UINT idx, float x, float y, float z)
{
	NuiScopedMapBuffer<SgVec3f> normals(m_normalStream);
	if(idx >= normals.size())
		return false;

	normals[idx].setValue(x, y, z);
	return true;
}

bool	NuiCLMappableData::IsVertexValid(UINT idx) const
{
	NuiScopedMapConstBuffer<SgVec3f> positions(m_positionStream);
	if(idx >= positions.size())
		return false;

	return (positions[idx][2] > 0.0f);
}

void NuiCLMappableData::relaxToCPU()
{
	NuiMappableAccessor::relaxToCPU(m_positionStream);
	//

	NuiTextureMappableAccessor::relaxToCPU(ColorTex());
	NuiTextureMappableAccessor::relaxToCPU(FeedbackTex());
}

