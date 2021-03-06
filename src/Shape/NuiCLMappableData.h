#pragma once

#include "stdafx.h"
#include "Shape/NuiCameraPos.h"
#include "Shape/NuiPolygonMesh.h"
#include "OpenCLUtilities/NuiMappable.h"
#include "OpenCLUtilities/NuiTextureMappable.h"

class NuiCLMappableData
{
public:
	enum eMappableDataIndexFlags
	{
		E_MappableData_Point = (0x1 << 0),
		E_MappableData_Triangle = (0x1 << 1),
		E_MappableData_Wireframe = (0x1 << 2),

		E_MappableData_LAST_BIT = (0x1 << 3),
	};
public:
	NuiCLMappableData();
	~NuiCLMappableData();

	void				Clear();
	void			DeepCopy (const NuiCLMappableData& other);
	//NuiCLMappableData (const NuiCLMappableData& other) { DeepCopy(other); }
	NuiCLMappableData& operator = (const NuiCLMappableData& other) {	DeepCopy(other); return *this; }
	
	NuiMappableui&		PointIndices() { return m_pointIndices; }
	NuiMappableui&		TriangleIndices() { return m_triangleIndices; }
	NuiMappableui&		WireframeIndices() { return m_wireframeIndices; }

	NuiMappable3f&		PositionStream() { return m_positionStream; }
	NuiMappable4f&		ColorStream() { return m_colorStream; }
	NuiMappable3f&		NormalStream() { return m_normalStream; }
	NuiMappable2f&		PatchUVStream() { return m_patchUVStream; }

	NuiTextureMappable&	ColorTex() { return m_colorTex; }
	NuiTextureMappable&	FeedbackTex() { return m_feedbackTex; }

	UINT				GetPositionNum() const { return (UINT)(m_positionStream.size()); }
	const float*		GetPositionValue(UINT idx) const;
	bool				SetPositionValue(UINT idx, float x, float y, float z);
	const float*		GetNormalValue(UINT idx) const;
	bool				SetNormalValue(UINT idx, float x, float y, float z);
	bool				IsVertexValid(UINT idx) const;

	UINT				WidthStep() const { return m_nWidthStep; }
	void				WidthStep(UINT w) { m_nWidthStep = w; }

	void				SetCameraParams(const NuiCameraParams& params);
	void				SetCameraPos(const NuiCameraPos& cam) { m_camPos = cam; }
	const NuiCameraPos&	GetCameraPos() const { return m_camPos; }

	NuiPolygonMesh*		GetPolygonMesh() { return &m_mesh; }

	void				SetBoundingBox(const SgVec3f& min, const SgVec3f& max) { m_boundingBoxMin = min; m_boundingBoxMax = max; }
	const SgVec3f&		GetBoundingBoxMin() const { return m_boundingBoxMin; }
	const SgVec3f&		GetBoundingBoxMax() const { return m_boundingBoxMax; }

	void				SetStreamDirty(bool isDirty) { m_streamDirty = isDirty; }
	bool				IsStreamDirty() const { return m_streamDirty; }
	void				SetIndexingDirty(bool isDirty) { m_indexDirty = isDirty; }
	bool				IsIndexingDirty() const { return m_indexDirty; }
	
	// Enable relaxing GPU data to CPU
	void relaxToCPU();

private:
	UINT				m_nWidthStep;
	NuiMappableui		m_pointIndices;
	NuiMappableui		m_triangleIndices;
	NuiMappableui		m_wireframeIndices;

	NuiMappable3f		m_positionStream;
	NuiMappable4f		m_colorStream;
	NuiMappable3f		m_normalStream;
	NuiMappable2f		m_patchUVStream;

	NuiTextureMappable	m_colorTex;
	NuiTextureMappable	m_feedbackTex;
	NuiCameraPos		m_camPos;

	SgVec3f				m_boundingBoxMin;
	SgVec3f				m_boundingBoxMax;

	NuiPolygonMesh		m_mesh;

	bool				m_streamDirty;
	bool				m_indexDirty;
};