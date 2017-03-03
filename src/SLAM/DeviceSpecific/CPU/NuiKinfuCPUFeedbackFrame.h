#pragma once

#include "../NuiKinfuFeedbackFrame.h"
#include "Shape\NuiImageBuffer.h"

typedef Eigen::Vector3f Vector3f;
struct NuiTrackerConfig;
class NuiCameraPos;

class NuiKinfuCPUFeedbackFrame : public NuiKinfuFeedbackFrame
{
public:
	NuiKinfuCPUFeedbackFrame(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight);
	virtual ~NuiKinfuCPUFeedbackFrame();

	virtual void	UpdateBuffers(NuiKinfuFrame* pFrame, NuiKinfuCameraState* pCameraState) override;

	virtual bool	VerticesToMappablePosition(NuiCLMappableData* pMappableData) override;
	virtual bool	BufferToMappableTexture(NuiCLMappableData* pMappableData, TrackerBufferType bufferType) override;

	Vector3f*		GetVertexBuffer() const { return m_vertices.GetBuffer(); }
	Vector3f*		GetNormalsBuffer() const { return m_normals.GetBuffer(); }

protected:
	void	AcquireBuffers(UINT nWidth, UINT nHeight);
	void	ReleaseBuffers();

	void	Vertex2Normal(Vector3f* verticesBuffer, float depth_threshold);
	void	TransformBuffers(Vector3f* verticesBuffer, const NuiCameraPos& cameraPos);

private:
	NuiFloat3Image	m_vertices;
	NuiFloat3Image	m_normals;
};