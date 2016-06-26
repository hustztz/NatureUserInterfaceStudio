#pragma once
///////////////////////////////////////////////////////////////////////////////
//
// pointCloudGeometryOverride.h
//
// Handles vertex data preparation for drawing the user defined shape in
// Viewport 2.0.
//
////////////////////////////////////////////////////////////////////////////////

#include <maya/MPxGeometryOverride.h>
#include <maya/MIntArray.h>
#include <maya/MStateManager.h>

#include <vector>

class NuiMayaPointCloudShape;
namespace MHWRender
{
	class MIndexBuffer;
}

class NuiMayaPointCloudGeometryOverride : public MHWRender::MPxGeometryOverride
{
public:
	static MPxGeometryOverride* Creator(const MObject& obj)
	{
		return new NuiMayaPointCloudGeometryOverride(obj);
	}

	virtual ~NuiMayaPointCloudGeometryOverride();

	virtual MHWRender::DrawAPI supportedDrawAPIs() const;

	virtual void updateDG();
	virtual void updateRenderItems(
		const MDagPath& path,
		MHWRender::MRenderItemList& list);
	virtual void populateGeometry(
		const MHWRender::MGeometryRequirements& requirements,
		const MHWRender::MRenderItemList& renderItems,
		MHWRender::MGeometry& data);
	virtual void cleanUp();
	virtual bool isIndexingDirty(
		const MHWRender::MRenderItem& item);
	virtual bool isStreamDirty(
		const MHWRender::MVertexBufferDescriptor& desc);

protected:
	NuiMayaPointCloudGeometryOverride(const MObject& obj);

	void setSolidPointSize(MHWRender::MShaderInstance* shaderInstance, float value);
	void setSolidColor(MHWRender::MShaderInstance* shaderInstance, const float *value);
	bool enableActiveComponentDisplay(const MDagPath &path) const;

	// Render item handling methods
	void updateSelectedBoundingBoxRenderItem(const MDagPath& path, MHWRender::MRenderItemList& list);
	void updatePointCloudItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr);
	void updateShadedPointCloudItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr);
	void updateTexturedPointCloudItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr);
	void updateActiveVerticesItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr);
	void updateVertexNormalsItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr);
	void updateWireframeItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr);
	void updateShadedItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr);
	void updateTexturedItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr);
	
	// Data stream (geometry requirements) handling
	void updateGeometryRequirements(const MHWRender::MGeometryRequirements& requirements,
		MHWRender::MGeometry& data,
		bool debugPopulateGeometry);

	// Indexing for render item handling methods
	void updateIndexingForActiveVertices(const MHWRender::MRenderItem* item, MHWRender::MGeometry& data);

	NuiMayaPointCloudShape* fShape;
	std::vector<unsigned int> fActiveVertexIndices;

	// Render item names
	static const MString sBoundingBoxItemName;
	static const MString sPointCloudItemName;
	static const MString sShadedPointCloudItemName;
	static const MString sTexturedPointCloudItemName;
	static const MString sVertexNormalItemName;
	static const MString sActiveVertexItemName;
	static const MString sWireframeItemName;
	static const MString sShadedItemName;
	static const MString sTexturedItemName;

public:
	static void PreDrawShadedCallback(
		MHWRender::MDrawContext&            context,
		const MHWRender::MRenderItemList&   renderItemList,
		MHWRender::MShaderInstance*         shaderInstance
		);
};