#include "NuiKinfuCPUFeedbackFrame.h"

#include "NuiKinfuCPUUtilities.h"
#include "NuiKinfuCPUFrame.h"

#include "../../NuiKinfuCameraState.h"
#include "Foundation/NuiDebugMacro.h"
#include "Shape/NuiCLMappableData.h"
#include "assert.h"

NuiKinfuCPUFeedbackFrame::NuiKinfuCPUFeedbackFrame(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight)
{
	AcquireBuffers(nWidth, nHeight);
}

NuiKinfuCPUFeedbackFrame::~NuiKinfuCPUFeedbackFrame()
{
	ReleaseBuffers();
}

void	NuiKinfuCPUFeedbackFrame::AcquireBuffers(UINT nWidth, UINT nHeight)
{
	if(nWidth == m_vertices.GetWidth() && nHeight == m_vertices.GetHeight())
	{
		return;
	}

	ReleaseBuffers();

	m_vertices.AllocateBuffer(nWidth, nHeight);
	m_normals.AllocateBuffer(nWidth, nHeight);
}

void	NuiKinfuCPUFeedbackFrame::ReleaseBuffers()
{
	m_vertices.Clear();
	m_normals.Clear();
}

void	NuiKinfuCPUFeedbackFrame::Vertex2Normal(Vector3f* verticesBuffer, float depth_threshold)
{
	Vector3f* normalsBuffer = m_normals.GetBuffer();
	if(!normalsBuffer || !verticesBuffer)
		return;

	UINT nWidth = m_normals.GetWidth();
	UINT nHeight = m_normals.GetHeight();
	for (UINT y = 0; y < nHeight; y++)
	{
		for (UINT x = 0; x < nWidth; x++)
		{
			const UINT centerId = y * nWidth + x;
			Vector3f center = verticesBuffer[centerId];
			if(!_IsNan(center))
			{
				Vector3f left = center;
				if(x > 0)
				{
					left = verticesBuffer[centerId-1];
					if(_IsNan(left) || fabs(center[2] - left[2]) > depth_threshold)
						left = center;
				}
				Vector3f right = center;
				if(x < nWidth-1)
				{
					right = verticesBuffer[centerId+1];
					if(_IsNan(right) || fabs(center[2] - right[2]) > depth_threshold)
						right = center;
				}
				Vector3f up = center;
				if(y > 0)
				{
					up = verticesBuffer[centerId-nWidth];
					if(_IsNan(up) || fabs(center[2] - up[2]) > depth_threshold)
						up = center;
				}
				Vector3f down = center;
				if(y < nHeight-1)
				{
					down = verticesBuffer[centerId+nWidth];
					if(_IsNan(down) || fabs(center[2] - down[2]) > depth_threshold)
						down = center;
				}

				// gradients x and y
				Vector3f diff_x = right - left;
				Vector3f diff_y = down - up;

				// cross product
				Vector3f outNormal = diff_x.cross(diff_y);
				/*outNormal.x = (diff_x.y * diff_y.z - diff_x.z*diff_y.y);
				outNormal.y = (diff_x.z * diff_y.x - diff_x.x*diff_y.z);
				outNormal.z = (diff_x.x * diff_y.y - diff_x.y*diff_y.x);*/
				if(outNormal[0] == 0.0f && outNormal[1] == 0.0f && outNormal[2] == 0.0f)
				{
					normalsBuffer[centerId] = Vector3f(NAN_FLOAT, NAN_FLOAT, NAN_FLOAT);
				}
				//float norm = 1.0f / sqrt((outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z));
				normalsBuffer[centerId] = outNormal.normalized();
			}
			else
			{
				normalsBuffer[centerId] = Vector3f(NAN_FLOAT, NAN_FLOAT, NAN_FLOAT);
			}
		}
	}
}

void NuiKinfuCPUFeedbackFrame::TransformBuffers(Vector3f* verticesBuffer, const NuiCameraPos& cameraPos)
{
	if(!verticesBuffer)
		return;

	const Matrix3frm& rot = cameraPos.getRotation();
	const Vector3f& trans = cameraPos.getTranslation();
	
	Vector3f* dstBuffer = m_vertices.GetBuffer();
	Vector3f* normalsBuffer = m_normals.GetBuffer();
	if(!normalsBuffer || !dstBuffer)
		return;

	UINT nWidth = m_normals.GetWidth();
	UINT nHeight = m_normals.GetHeight();
	for (UINT y = 0; y < nHeight; y++)
	{
		for (UINT x = 0; x < nWidth; x++)
		{
			const UINT id = y * nWidth + x;
			Vector3f vert_src = verticesBuffer[id];
			if(_IsNan(vert_src))
			{
				dstBuffer[id] = Vector3f(NAN_FLOAT, NAN_FLOAT, NAN_FLOAT);
				normalsBuffer[id] = Vector3f(NAN_FLOAT, NAN_FLOAT, NAN_FLOAT);
				continue;
			}
			dstBuffer[id] = rot * vert_src + trans;
			Vector3f norm_src = normalsBuffer[id];
			if(_IsNan(vert_src))
			{
				normalsBuffer[id] = Vector3f(NAN_FLOAT, NAN_FLOAT, NAN_FLOAT);
				continue;
			}
			normalsBuffer[id] = rot * norm_src;
		}
	}
}


void	NuiKinfuCPUFeedbackFrame::UpdateBuffers(NuiKinfuFrame* pFrame, NuiKinfuCameraState* pCameraState)
{
	if(!pFrame)
		return;
	NuiKinfuCPUFrame* pCPUFrame = dynamic_cast<NuiKinfuCPUFrame*>(pFrame);
	if(!pCPUFrame)
		return;

	if(!pCameraState)
		return;

	if(pCPUFrame->GetWidth() != GetWidth() || pCPUFrame->GetHeight() != GetHeight())
		return;

	Vertex2Normal(pCPUFrame->GetVertexBuffer(), pCPUFrame->GetDepthThreshold());
	TransformBuffers(pCPUFrame->GetVertexBuffer(), pCameraState->GetCameraPos());
}

bool NuiKinfuCPUFeedbackFrame::VerticesToMappablePosition(NuiCLMappableData* pMappableData)
{
	assert(pMappableData);
	if(!pMappableData)
		return false;

	Vector3f* verticesBuffer = m_vertices.GetBuffer();
	if(!verticesBuffer)
		return false;

	const UINT nWidth = m_vertices.GetWidth();
	const UINT nHeight = m_vertices.GetHeight();

	pMappableData->SetBoundingBox(SgVec3f(-256.0f / 370.0f, -212.0f / 370.0f, 0.4f),
		SgVec3f((nWidth-256.0f) / 370.0f, (nHeight-212.0f) / 370.0f, 4.0f));

	NuiMappableAccessor::asVectorImpl(pMappableData->TriangleIndices())->data().clear();
	NuiMappableAccessor::asVectorImpl(pMappableData->WireframeIndices())->data().clear();

	std::vector<unsigned int>& clPointIndices =
		NuiMappableAccessor::asVectorImpl(pMappableData->PointIndices())->data();
	const UINT nPointsNum = nWidth * nHeight;
	if(clPointIndices.size() != nPointsNum)
	{
		clPointIndices.resize(nPointsNum);
		for (UINT i = 0; i < nPointsNum; ++i)
		{
			clPointIndices[i] = i;
		}
		pMappableData->SetIndexingDirty(true);
	}

	std::vector<SgVec3f>& clVertices =
		NuiMappableAccessor::asVectorImpl(pMappableData->PositionStream())->data();
	if( nPointsNum != clVertices.size() )
	{
		clVertices.resize(nPointsNum);
	}

	for (UINT i = 0; i < nPointsNum; ++i)
	{
		clVertices[i][0] = verticesBuffer[i][0];
		clVertices[i][1] = verticesBuffer[i][1];
		clVertices[i][2] = verticesBuffer[i][2];
	}
	pMappableData->SetStreamDirty(true);

	return true;
}

bool	NuiKinfuCPUFeedbackFrame::BufferToMappableTexture(NuiCLMappableData* pMappableData, TrackerBufferType bufferType)
{
	assert(pMappableData);
	if(!pMappableData)
		return false;

	Vector3f* buffer = NULL;
	switch (bufferType)
	{
	case eTracker_Vertices:
		buffer = m_vertices.GetBuffer();
		break;
	case eTracker_Normals:
		buffer = m_normals.GetBuffer();
		break;
	default:
		break;
	}

	if(!buffer)
		return false;

	NuiTextureMappableAccessor::updateImpl(
		pMappableData->ColorTex(),
		m_vertices.GetWidth(),
		m_vertices.GetHeight(),
		buffer
		);

	return true;
}

