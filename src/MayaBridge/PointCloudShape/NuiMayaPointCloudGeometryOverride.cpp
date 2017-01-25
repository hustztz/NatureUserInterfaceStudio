///////////////////////////////////////////////////////////////////////////////
//
// pointCloudGeometryOverride.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include "NuiMayaPointCloudGeometryOverride.h"
#include "NuiMayaPointCloudShape.h"
#include "Shape/NuiCLMappableData.h"

#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "../HardwareUtilities/NuiMayaHWMappable.h"

#include <maya/MFnDagNode.h>
#include <maya/MFnSingleIndexedComponent.h>
#include <maya/MHWGeometry.h>
#include <maya/MHWGeometryUtilities.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MObjectArray.h>
#include <maya/MShaderManager.h>
#include <maya/MDistance.h>
#include <maya/MGlobal.h>

const MString NuiMayaPointCloudGeometryOverride::sBoundingBoxItemName = "BoundingBoxItem";
const MString NuiMayaPointCloudGeometryOverride::sPointCloudItemName = "pointCloudVertices";
const MString NuiMayaPointCloudGeometryOverride::sShadedPointCloudItemName = "shadedPointCloudVertices";
const MString NuiMayaPointCloudGeometryOverride::sTexturedPointCloudItemName = "texturedPointCloudVertices";
const MString NuiMayaPointCloudGeometryOverride::sVertexNormalItemName = "pointCloudVertexNormals";
const MString NuiMayaPointCloudGeometryOverride::sActiveVertexItemName = "pointCloudActiveVertices";
const MString NuiMayaPointCloudGeometryOverride::sWireframeItemName = "pointCloudWireframe";
const MString NuiMayaPointCloudGeometryOverride::sShadedItemName = "pointCloudShaded";
const MString NuiMayaPointCloudGeometryOverride::sTexturedItemName = "pointCloudTextured";

void NuiMayaPointCloudGeometryOverride::PreDrawShadedCallback(
	MHWRender::MDrawContext&            context,
	const MHWRender::MRenderItemList&   renderItemList,
	MHWRender::MShaderInstance*         shaderInstance
	)
{
	// Need to guarantee all the opencl work is done before rendering
	cl_int err = clFinish(NuiOpenCLGlobal::instance().clQueue());
	NUI_CHECK_CL_ERR(err);
}

NuiMayaPointCloudGeometryOverride::NuiMayaPointCloudGeometryOverride(const MObject& obj)
	: MPxGeometryOverride(obj)
	, fShape(NULL)
{
	// get the real apiMesh object from the MObject
	MStatus status;
	MFnDependencyNode node(obj, &status);
	if (status)
	{
		fShape = dynamic_cast<NuiMayaPointCloudShape*>(node.userNode());
	}
}

NuiMayaPointCloudGeometryOverride::~NuiMayaPointCloudGeometryOverride()
{
	fShape = NULL;
}

MHWRender::DrawAPI NuiMayaPointCloudGeometryOverride::supportedDrawAPIs() const
{
	return (MHWRender::kAllDevices);
}

void NuiMayaPointCloudGeometryOverride::updateDG()
{
	if(!fShape)
		return;

	// Pull the actual outMesh from the shape, as well
	// as any active components
	MIntArray activeVertices;
	if (fShape)
	{
		if (fShape->hasActiveComponents())
		{
			MObjectArray clist = fShape->activeComponents();
			if (clist.length())
			{
				MFnSingleIndexedComponent fnComponent( clist[0] );
				if (fnComponent.elementCount())
				{
					fnComponent.getElements( activeVertices );

					for(unsigned int i = 0; i < activeVertices.length(); ++i)
					{
						fActiveVertexIndices.push_back(i);
					}
				}
			}
		}
	}
}

/*
	Test to see if active components should be enabled.
	Based on active vertices + non-template state
*/
bool NuiMayaPointCloudGeometryOverride::enableActiveComponentDisplay(const MDagPath &path) const
{
	bool enable = true;

	// If no active components then disable the active
	// component render item
	if (fActiveVertexIndices.size() == 0)
	{
		enable = false;
	}
	else
	{
		// If there are components then we need to check
		// either the display status of the object, or
		// in the case of a templated object make sure
		// to hide components to be consistent with how
		// internal objects behave
		//
		MHWRender::DisplayStatus displayStatus =
			MHWRender::MGeometryUtilities::displayStatus(path);
		if (displayStatus == MHWRender::kTemplate ||
			displayStatus == MHWRender::kActiveTemplate)
		{
			enable = false;
		}
		else
		{
			// Do an explicit path test for templated
			// since display status does not indicate this.
			if (path.isTemplated())
				enable = false;
		}
	}
	return enable;
}

/*
	Set the point size for solid color shaders
*/
void NuiMayaPointCloudGeometryOverride::setSolidPointSize(MHWRender::MShaderInstance* shaderInstance, float pointSize)
{
	if (!shaderInstance)
		return;

	float pointSizeArray[2] = {0};
	pointSizeArray[0] = pointSize;
	pointSizeArray[1] = pointSize;

	const MString pointSizeParameterName = "pointSize";
	shaderInstance->setParameter( pointSizeParameterName, pointSizeArray );
}

void NuiMayaPointCloudGeometryOverride::updateSelectedBoundingBoxRenderItem(
	const MDagPath& path, MHWRender::MRenderItemList& list)
{
	// Update render item
	int index = list.indexOf(sBoundingBoxItemName);
	if(index < 0)
		return;

	MHWRender::MRenderItem* boundingBoxItem = list.itemAt(index);
	if( !boundingBoxItem ) return;

	boundingBoxItem->setDrawMode(MHWRender::MGeometry::kAll);
	MHWRender::DisplayStatus displayStatus =
		MHWRender::MGeometryUtilities::displayStatus(path);
	switch (displayStatus) {
		case MHWRender::kActive:
		case MHWRender::kActiveAffected:
		case MHWRender::kLead:
		case MHWRender::kTemplate:
		case MHWRender::kActiveTemplate:
		case MHWRender::kIntermediateObject:
			boundingBoxItem->enable(true);
			break;
		default:
			boundingBoxItem->enable(false);
			break;
	}
}

/*
	Create a render item for dormant vertices if it does not exist. Updating
	shading parameters as necessary.
*/
void NuiMayaPointCloudGeometryOverride::updatePointCloudItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr)
{
	NuiCLMappableData* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	MHWRender::MRenderItem* pointCloudItem = NULL;
	int index = list.indexOf(sPointCloudItemName);
	if (index < 0)
	{
		pointCloudItem = MHWRender::MRenderItem::Create(
			sPointCloudItemName,
			MHWRender::MRenderItem::DecorationItem,
			MHWRender::MGeometry::kPoints);
		pointCloudItem->setDrawMode((MHWRender::MGeometry::DrawMode)(MHWRender::MGeometry::kWireframe));
		// Set depth priority higher than wireframe and shaded render items,
		// but lower than active points.
		// Raising higher than wireframe will make them not seem embedded into the surface
		pointCloudItem->depthPriority( MHWRender::MRenderItem::sDormantWireDepthPriority );

		MHWRender::MShaderInstance* shader = shaderMgr->getStockShader(
			MHWRender::MShaderManager::k3dFatPointShader );
		if (shader)
		{
			// assign shader
			pointCloudItem->setShader(shader);

			// once assigned, no need to hold on to shader instance
			shaderMgr->releaseShader(shader);
		}

		list.append(pointCloudItem);
	}
	else
	{
		pointCloudItem = list.itemAt(index);
	}

	if (pointCloudItem)
	{
		if(geomPtr->PointIndices().size() <= 0)
		{
			pointCloudItem->enable(false);
			return;
		}

		MHWRender::MShaderInstance* shader = pointCloudItem->getShader();
		if (shader)
		{
			// Set the point size parameter
			static const float pointSize = 3.0f;
			setSolidPointSize( shader, pointSize );

			MColor wireColor = MHWRender::MGeometryUtilities::wireframeColor(path);
			setSolidColor( shader, &(wireColor.r));
		}

		MHWRender::DisplayStatus displayStatus =
			MHWRender::MGeometryUtilities::displayStatus(path);

		// Generally if the display status is hilite then we
		// draw components.
		if (displayStatus == MHWRender::kHilite)
		{
			// In case the object is templated
			// we will hide the components to be consistent
			// with how internal objects behave.
			if (path.isTemplated())
				pointCloudItem->enable(false);
			else
				pointCloudItem->enable(true);
		}
		else
		{
			pointCloudItem->enable(true);
		}
	}
}

void NuiMayaPointCloudGeometryOverride::updateShadedPointCloudItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr)
{
	NuiCLMappableData* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	MHWRender::MRenderItem* pointCloudItem = NULL;
	int index = list.indexOf(sShadedPointCloudItemName);
	if (index < 0)
	{
		pointCloudItem = MHWRender::MRenderItem::Create(
			sShadedPointCloudItemName,
			MHWRender::MRenderItem::MaterialSceneItem,
			MHWRender::MGeometry::kPoints);
		pointCloudItem->setDrawMode((MHWRender::MGeometry::DrawMode)(MHWRender::MGeometry::kShaded));
		list.append(pointCloudItem);
	}
	else
	{
		pointCloudItem = list.itemAt(index);
	}

	if (pointCloudItem)
	{
		if(geomPtr->PointIndices().size() <= 0)
		{
			pointCloudItem->enable(false);
			return;
		}

		bool useLighting = fShape->getBooleanValue(NuiMayaPointCloudShape::aUseLighting);
		MHWRender::MShaderInstance* shader = useLighting ? 
			shaderMgr->getStockShader(MHWRender::MShaderManager::k3dDepthFalseColorMaterialPointCloudShader, PreDrawShadedCallback) :
			shaderMgr->getStockShader(MHWRender::MShaderManager::k3dDepthFalseColorPointCloudShader);
		if (shader)
		{
			// Set the point size parameter
			static const float pointSize = 3.0f;
			setSolidPointSize( shader, pointSize );

			// assign shader
			pointCloudItem->setShader(shader);

			// once assigned, no need to hold on to shader instance
			shaderMgr->releaseShader(shader);
		}

		MHWRender::DisplayStatus displayStatus =
			MHWRender::MGeometryUtilities::displayStatus(path);

		// Generally if the display status is hilite then we
		// draw components.
		if (displayStatus == MHWRender::kHilite)
		{
			// In case the object is templated
			// we will hide the components to be consistent
			// with how internal objects behave.
			if (path.isTemplated())
				pointCloudItem->enable(false);
			else
				pointCloudItem->enable(true);
		}
		else
		{
			pointCloudItem->enable(true);
		}
	}
}

void NuiMayaPointCloudGeometryOverride::updateTexturedPointCloudItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr)
{
	NuiCLMappableData* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	MHWRender::MRenderItem* pointCloudItem = NULL;
	int index = list.indexOf(sTexturedPointCloudItemName);
	if (index < 0)
	{
		pointCloudItem = MHWRender::MRenderItem::Create(
			sTexturedPointCloudItemName,
			MHWRender::MRenderItem::MaterialSceneItem,
			MHWRender::MGeometry::kPoints);
		pointCloudItem->setDrawMode((MHWRender::MGeometry::DrawMode)(MHWRender::MGeometry::kTextured));
		list.append(pointCloudItem);
	}
	else
	{
		pointCloudItem = list.itemAt(index);
	}

	if (pointCloudItem)
	{
		if(geomPtr->PointIndices().size() <= 0)
		{
			pointCloudItem->enable(false);
			return;
		}

		MHWRender::MShaderInstance* shader = NULL;
		bool bUV2Color = (geomPtr->ColorStream().size() == 0);
		bool useLighting = fShape->getBooleanValue(NuiMayaPointCloudShape::aUseLighting);
		if(bUV2Color)
		{
			shader = useLighting ?
				shaderMgr->getStockShader(MHWRender::MShaderManager::k3dTextureMaterialPointCloudShader, PreDrawShadedCallback) :
				shaderMgr->getStockShader(MHWRender::MShaderManager::k3dTexturePointCloudShader );
			if (shader)
			{
				MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
				MHWRender::MTextureManager* textureMgr = renderer->getTextureManager();
				if(textureMgr)
				{
					NuiCLMappableData* geomPtr = fShape->meshGeom();
					if(geomPtr)
					{
						const NuiTextureMappable& colorTex = geomPtr->ColorTex();
						MHWRender::MTextureDescription textureDesc;
						textureDesc.setToDefault2DTexture();
						textureDesc.fWidth = colorTex.width();
						textureDesc.fHeight = colorTex.height();
						textureDesc.fDepth = 1;
						textureDesc.fBytesPerSlice = colorTex.width() * colorTex.height() * sizeof(BGRQUAD);
						textureDesc.fBytesPerRow = colorTex.width() * sizeof(BGRQUAD);
						textureDesc.fMipmaps = 1;
						textureDesc.fArraySlices = 1;
						textureDesc.fTextureType = MHWRender::kImage2D;
						textureDesc.fFormat = MHWRender::kB8G8R8A8;
						MHWRender::MTexture *colorTexture =
							textureMgr->acquireTexture("", textureDesc, colorTex.data(), false);

						if(colorTexture)
						{
							MHWRender::MTextureAssignment texAssignment;
							texAssignment.texture = colorTexture;
							shader->setParameter("map", texAssignment);
						}
						textureMgr->releaseTexture(colorTexture);
					}
				}
			}
		}
		else
		{
			shader = shaderMgr->getStockShader(MHWRender::MShaderManager::k3dCPVFatPointShader );
		}
		if (shader)
		{
			// Set the point size parameter
			static const float pointSize = 3.0f;
			setSolidPointSize( shader, pointSize );

			// assign shader
			pointCloudItem->setShader(shader);

			// once assigned, no need to hold on to shader instance
			shaderMgr->releaseShader(shader);
		}

		MHWRender::DisplayStatus displayStatus =
			MHWRender::MGeometryUtilities::displayStatus(path);

		// Generally if the display status is hilite then we
		// draw components.
		if (displayStatus == MHWRender::kHilite)
		{
			// In case the object is templated
			// we will hide the components to be consistent
			// with how internal objects behave.
			if (path.isTemplated())
				pointCloudItem->enable(false);
			else
				pointCloudItem->enable(true);
		}
		else
		{
			pointCloudItem->enable(true);
		}
	}
}

/*
	Create a render item for active vertices if it does not exist. Updating
	shading parameters as necessary.
*/
void NuiMayaPointCloudGeometryOverride::updateActiveVerticesItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr)
{
	bool enableActiveDisplay = enableActiveComponentDisplay(path);

	MHWRender::MRenderItem* activeItem = NULL;
	int index = list.indexOf(sActiveVertexItemName);
	if (index < 0)
	{
		activeItem = MHWRender::MRenderItem::Create(
			sActiveVertexItemName,
			MHWRender::MRenderItem::DecorationItem,
			MHWRender::MGeometry::kPoints);
		activeItem->setDrawMode((MHWRender::MGeometry::DrawMode)(MHWRender::MGeometry::kWireframe | MHWRender::MGeometry::kShaded | MHWRender::MGeometry::kTextured));
		// Set depth priority to be active point. This should offset it
		// to be visible above items with "dormant point" priority.
		activeItem->depthPriority( MHWRender::MRenderItem::sActivePointDepthPriority );
		list.append(activeItem);

		MHWRender::MShaderInstance* shader = shaderMgr->getStockShader(MHWRender::MShaderManager::k3dFatPointShader );
		if (shader)
		{
			// Set the point size parameter. Make it slightly larger for active vertices
			static const float pointSize = 5.0f;
			setSolidPointSize( shader, pointSize );

			// Assign shader. Use a named stream if we want to supply a different
			// set of "shared" vertices for drawing active vertices
			activeItem->setShader(shader);

			// once assigned, no need to hold on to shader instance
			shaderMgr->releaseShader(shader);
		}
	}
	else
	{
		activeItem = list.itemAt(index);
	}

	if (activeItem)
	{
		MHWRender::MShaderInstance* shader = activeItem->getShader();
		if (shader)
		{
			// Set active color
			static const float theColor[] = { 1.0f, 1.0f, 0.0f, 1.0f };
			shader->setParameter("solidColor", theColor);
		}

		activeItem->enable( enableActiveDisplay );
	}
}

/*
	Create a render item for vertex normals if it does not exist. Updating
	shading parameters as necessary.
*/
void NuiMayaPointCloudGeometryOverride::updateVertexNormalsItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr)
{
	NuiCLMappableData* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	MHWRender::MRenderItem* vertexNormalItem = NULL;
	int index = list.indexOf(sVertexNormalItemName);
	if (index < 0)
	{
		vertexNormalItem = MHWRender::MRenderItem::Create(
			sVertexNormalItemName,
			MHWRender::MRenderItem::DecorationItem,
			MHWRender::MGeometry::kPoints);
		vertexNormalItem->setDrawMode((MHWRender::MGeometry::DrawMode)(MHWRender::MGeometry::kWireframe | MHWRender::MGeometry::kShaded | MHWRender::MGeometry::kTextured));
		// Set depth priority higher than wireframe and shaded render items,
		// but lower than active points.
		// Raising higher than wireframe will make them not seem embedded into the surface
		vertexNormalItem->depthPriority( MHWRender::MRenderItem::sDormantWireDepthPriority );
		list.append(vertexNormalItem);

		MHWRender::MShaderInstance* shader = shaderMgr->getStockShader(
			MHWRender::MShaderManager::k3dPointNormalShader, PreDrawShadedCallback);
		if (shader)
		{
			// Set the point size parameter
			static const float pointSize = 3.0f;
			setSolidPointSize( shader, pointSize );

			// assign shader
			vertexNormalItem->setShader(shader);

			// once assigned, no need to hold on to shader instance
			shaderMgr->releaseShader(shader);
		}
	}
	else
	{
		vertexNormalItem = list.itemAt(index);
	}

	if (vertexNormalItem)
	{
		if(!fShape->getBooleanValue(NuiMayaPointCloudShape::aShowNormals) || (geomPtr->PointIndices().size() <= 0))
		{
			vertexNormalItem->enable(false);
			return;
		}
		// Set parameters
		MHWRender::MShaderInstance* shader = vertexNormalItem->getShader();
		if (shader)
		{
			float scale = fShape->getFloatValue(NuiMayaPointCloudShape::aNormalLength);
			const MString vectorSclaeParameterName = "vectorScale";
			shader->setParameter( vectorSclaeParameterName, scale );

			static const float theColor[] = { 0.0f, 0.0f, 1.0f, 1.0f };
			const MString colorParameterName = "solidColor";
			shader->setParameter( colorParameterName, theColor );
		}
		// Enable
		vertexNormalItem->enable(true);
	}
}

void NuiMayaPointCloudGeometryOverride::setSolidColor(MHWRender::MShaderInstance* shaderInstance, const float *value)
{
	if (!shaderInstance)
		return;

	const MString colorParameterName = "solidColor";
	shaderInstance->setParameter(colorParameterName, value);
}

void NuiMayaPointCloudGeometryOverride::updateWireframeItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr)
{
	NuiCLMappableData* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	MHWRender::MRenderItem* wireframeItem = NULL;
	int index = list.indexOf(sWireframeItemName);
	if (index < 0)
	{
		wireframeItem = MHWRender::MRenderItem::Create(
			sWireframeItemName,
			MHWRender::MRenderItem::DecorationItem,
			MHWRender::MGeometry::kLines);
		wireframeItem->setDrawMode((MHWRender::MGeometry::DrawMode)(MHWRender::MGeometry::kWireframe));
		wireframeItem->depthPriority( MHWRender::MRenderItem::sDormantWireDepthPriority );
		list.append(wireframeItem);

		MHWRender::MShaderInstance* shader = shaderMgr->getStockShader(
			MHWRender::MShaderManager::k3dSolidShader);
		if (shader)
		{
			// assign shader
			wireframeItem->setShader(shader);

			// once assigned, no need to hold on to shader instance
			shaderMgr->releaseShader(shader);
		}
	}
	else
	{
		wireframeItem = list.itemAt(index);
	}

	if (wireframeItem)
	{
		// Enable
		bool bHasWireframe = (geomPtr->WireframeIndices().size() > 0);
		wireframeItem->enable(bHasWireframe);
		if(!bHasWireframe)
			return;

		MHWRender::MShaderInstance* shader = wireframeItem->getShader();
		MColor wireColor = MHWRender::MGeometryUtilities::wireframeColor(path);
		setSolidColor( shader, &(wireColor.r));
	}
}

void NuiMayaPointCloudGeometryOverride::updateShadedItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr)
{
	NuiCLMappableData* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	MHWRender::MRenderItem* shadedItem = NULL;
	int index = list.indexOf(sShadedItemName);
	if (index < 0)
	{
		shadedItem = MHWRender::MRenderItem::Create(
			sShadedItemName,
			MHWRender::MRenderItem::MaterialSceneItem,
			MHWRender::MGeometry::kTriangles);
		shadedItem->setDrawMode((MHWRender::MGeometry::DrawMode)(MHWRender::MGeometry::kShaded));
		list.append(shadedItem);
	}
	else
	{
		shadedItem = list.itemAt(index);
	}

	if (shadedItem)
	{
		bool useLighting = fShape->getBooleanValue(NuiMayaPointCloudShape::aUseLighting);
		MHWRender::MShaderInstance* shader = useLighting ?
			shaderMgr->getFragmentShader("mayaLambertSurface", "outSurfaceFinal", true) :
			shaderMgr->getStockShader(MHWRender::MShaderManager::k3dDepthFalseColorShader);
		if (shader)
		{
			shader->setIsTransparent(false);

			// assign shader
			shadedItem->setShader(shader);

			// once assigned, no need to hold on to shader instance
			shaderMgr->releaseShader(shader);
		}
		// Enable
		bool bHasShaded = (geomPtr->TriangleIndices().size() > 0);
		shadedItem->enable(bHasShaded);
	}
}

void NuiMayaPointCloudGeometryOverride::updateTexturedItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr)
{
	NuiCLMappableData* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	MHWRender::MRenderItem* texturedItem = NULL;
	int index = list.indexOf(sTexturedItemName);
	if (index < 0)
	{
		texturedItem = MHWRender::MRenderItem::Create(
			sTexturedItemName,
			MHWRender::MRenderItem::MaterialSceneItem,
			MHWRender::MGeometry::kTriangles);
		texturedItem->setDrawMode((MHWRender::MGeometry::DrawMode)(MHWRender::MGeometry::kTextured));
		list.append(texturedItem);		
	}
	else
	{
		texturedItem = list.itemAt(index);
	}

	if (texturedItem)
	{
		// Enable
		bool bHasTextured = (geomPtr->TriangleIndices().size() > 0);
		texturedItem->enable(bHasTextured);
		if(!bHasTextured)
			return;

		MHWRender::MShaderInstance* shader = NULL;
		bool bUV2Color = (geomPtr->ColorStream().size() == 0);
		bool useLighting = fShape->getBooleanValue(NuiMayaPointCloudShape::aUseLighting);
		if(bUV2Color)
		{
			shader = useLighting ?
				shaderMgr->getStockShader( MHWRender::MShaderManager::k3dTextureMaterialShader, PreDrawShadedCallback) :
				shaderMgr->getStockShader( MHWRender::MShaderManager::k3dTextureSurfaceShader );
			if (shader)
			{
				MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
				MHWRender::MTextureManager* textureMgr = renderer->getTextureManager();
				if(textureMgr)
				{
					NuiCLMappableData* geomPtr = fShape->meshGeom();
					if(geomPtr)
					{
						const NuiTextureMappable& colorTex = geomPtr->ColorTex();
						MHWRender::MTextureDescription textureDesc;
						textureDesc.setToDefault2DTexture();
						textureDesc.fWidth = colorTex.width();
						textureDesc.fHeight = colorTex.height();
						textureDesc.fDepth = 1;
						textureDesc.fBytesPerSlice = colorTex.width() * colorTex.height() * sizeof(BGRQUAD);
						textureDesc.fBytesPerRow = colorTex.width() * sizeof(BGRQUAD);
						textureDesc.fMipmaps = 1;
						textureDesc.fArraySlices = 1;
						textureDesc.fTextureType = MHWRender::kImage2D;
						textureDesc.fFormat = MHWRender::kB8G8R8A8;
						MHWRender::MTexture *colorTexture =
							textureMgr->acquireTexture("", textureDesc, colorTex.data(), false);

						if(colorTexture)
						{
							MHWRender::MTextureAssignment texAssignment;
							texAssignment.texture = colorTexture;
							shader->setParameter("map", texAssignment);
						}
						//if (!fColorSampler)
						//{
						//	MHWRender::MSamplerStateDesc samplerDesc;
						//	samplerDesc.addressU = MHWRender::MSamplerState::kTexClamp;
						//	samplerDesc.addressV = MHWRender::MSamplerState::kTexClamp;
						//	samplerDesc.addressW = MHWRender::MSamplerState::kTexClamp;
						//	samplerDesc.filter = MHWRender::MSamplerState::kMinMagMipPoint;
						//	fColorSampler = MHWRender::MStateManager::acquireSamplerState(samplerDesc);
						//}

						//if (fColorRemapTexture && fColorSampler)
						//{
						//	// Set up the ramp lookup
						//	MHWRender::MTextureAssignment texAssignment;
						//	texAssignment.texture = fColorRemapTexture;
						//	shader->setParameter("map", texAssignment);
						//	shader->setParameter("samp", *fColorSampler);

						//	// No remapping. The initial data created in the range 0...1
						//	//
						//	MFloatVector rampValueRange(0.0f, 1.0f);
						//	shader->setParameter("UVRange", (float*)&rampValueRange);
						//}
						textureMgr->releaseTexture(colorTexture);
					}
				}
			}
		}
		else
		{
			shader = shaderMgr->getStockShader( MHWRender::MShaderManager::k3dCPVSolidShader );
		}
		if (shader)
		{
			shader->setIsTransparent(false);

			// assign shader
			texturedItem->setShader(shader);

			// once assigned, no need to hold on to shader instance
			shaderMgr->releaseShader(shader);
		}
	}
}

/*
	Update render items. Shaded render item is provided so this
	method will be adding and updating UI render items only.
*/
void NuiMayaPointCloudGeometryOverride::updateRenderItems(
	const MDagPath& path,
	MHWRender::MRenderItemList& list)
{
	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if (!renderer) return;
	const MHWRender::MShaderManager* shaderMgr = renderer->getShaderManager();
	if (!shaderMgr) return;

	// Disable all the items
	for (int i = 0;i < list.length(); i++)
	{
		list.itemAt(i)->enable(false);
	}

	if(!fShape)
		return;
	NuiCLMappableData* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	int numVerts = geomPtr->GetPositionNum();
	if(0 == numVerts)
		return;

	// Update vertex render items
	updateSelectedBoundingBoxRenderItem(path, list);
	if( fShape->getBooleanValue(NuiMayaPointCloudShape::aShowNormals) )
	{
		updateVertexNormalsItem(path, list, shaderMgr);
	}
	if( geomPtr->TriangleIndices().size() > 0 )
	{
		updateWireframeItem(path, list, shaderMgr);
		updateShadedItem(path, list, shaderMgr);
		updateTexturedItem(path, list, shaderMgr);
	}
	else
	{
		updatePointCloudItem(path, list, shaderMgr);
		updateShadedPointCloudItem(path, list, shaderMgr);
		updateTexturedPointCloudItem(path, list, shaderMgr);
		updateActiveVerticesItem(path, list, shaderMgr);
	}
}

/*
	Examine the geometry requirements and create / update the
	appropriate data streams. As render items specify both named and
	unnamed data streams, both need to be handled here.
*/
void NuiMayaPointCloudGeometryOverride::updateGeometryRequirements(
		const MHWRender::MGeometryRequirements& requirements,
		MHWRender::MGeometry& data,
		bool debugPopulateGeometry)
{
	if(!fShape)
		return;
	NuiCLMappableData* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	bool bHasNormals = false;
	const MHWRender::MVertexBufferDescriptorList& descList =
		requirements.vertexRequirements();
	int numVertexReqs = descList.length();
	MHWRender::MVertexBufferDescriptor desc;
	for (int reqNum=0; reqNum<numVertexReqs; reqNum++)
	{
		if (!descList.getDescriptor(reqNum, desc))
		{
			continue;
		}

		// Fill vertex stream data used for dormant vertex, wireframe and shaded drawing.
		// Fill also for active vertices if (fDrawSharedActiveVertices=false)
		if (debugPopulateGeometry)
		{
			printf(">>> Fill in data for requirement[%d] with name %s. Semantic = %d\n",
				reqNum, desc.name().asChar(), desc.semantic() );
		}
		switch (desc.semantic())
		{
		case MHWRender::MGeometry::kPosition:
			{
				if (debugPopulateGeometry)
					printf("Acquire unnamed position buffer\n");
				// Copy to hardware buffer
				MHWRender::MVertexBuffer* positionBuffer = NuiMayaHWMappable::asHWVertexBuffer(desc, geomPtr->PositionStream());
				if(positionBuffer)
				{
					data.addVertexBuffer(positionBuffer);
				}
			}
			break;
		case MHWRender::MGeometry::kNormal:
			{
				MHWRender::MVertexBuffer* normalBuffer = NuiMayaHWMappable::asHWVertexBuffer(desc, geomPtr->NormalStream());
				if(normalBuffer)
				{
					data.addVertexBuffer(normalBuffer);
					bHasNormals = true;
				}
			}
			break;
		case MHWRender::MGeometry::kColor:
			{
				MHWRender::MVertexBuffer* cpvBuffer = NuiMayaHWMappable::asHWVertexBuffer(desc, geomPtr->ColorStream());
				if(cpvBuffer)
				{
					data.addVertexBuffer(cpvBuffer);
				}
			}
			break;
		case MHWRender::MGeometry::kTexture:
			{
				MHWRender::MVertexBuffer* uvBuffer = NuiMayaHWMappable::asHWVertexBuffer(desc, geomPtr->PatchUVStream());
				if(uvBuffer)
				{
					data.addVertexBuffer(uvBuffer);
				}
			}
			break;
		default:
			// do nothing for stuff we don't understand
			break;
		}
	}

	/*double distanceUnit = 1.0;
	MDistance::Unit currentUnit = MDistance::internalUnit();
	if(currentUnit != MDistance::kInvalid)
	{
		distanceUnit = MDistance(1.0, MDistance::kMeters).as(currentUnit);
	}
	else
	{
		MGlobal::displayError( " Invalid distance unit." );
		return;
	}*/
	geomPtr->SetStreamDirty(false);
}

/*
	Create / update indexing for render items which draw active vertices
*/
void NuiMayaPointCloudGeometryOverride::updateIndexingForActiveVertices(const MHWRender::MRenderItem* item,
															  MHWRender::MGeometry& data)
{
	unsigned int activeVertexCount = (unsigned int)fActiveVertexIndices.size();
	if(activeVertexCount == 0)
		return;

	MHWRender::MIndexBuffer* indexBuffer =
		data.createIndexBuffer(MHWRender::MGeometry::kUnsignedInt32);
	if (indexBuffer)
	{
		unsigned int* buffer = (unsigned int*)indexBuffer->acquire(activeVertexCount, true /*writeOnly - we don't need the current buffer values*/);
		if (buffer)
		{
			memcpy(buffer, fActiveVertexIndices.data(), activeVertexCount * sizeof(unsigned int));

			indexBuffer->commit(buffer);
			item->associateWithIndexBuffer(indexBuffer);
		}
	}
}


/*
	Fill in data and index streams based on the requirements passed in.
	Associate indexing with the render items passed in.

	Note that we leave both code paths to either draw shared or non-shared active vertices.
	The choice of which to use is up to the circumstances per plug-in.
	When drawing shared vertices, this requires an additional position buffer to be
	created so will use more memory. If drawing unshared vertices redundent extra
	vertices are drawn but will use less memory. The data member fDrawSharedActiveVertices
	can be set to decide on which implementation to use.
*/
void NuiMayaPointCloudGeometryOverride::populateGeometry(
	const MHWRender::MGeometryRequirements& requirements,
    const MHWRender::MRenderItemList& renderItems,
	MHWRender::MGeometry& data)
{
	NuiCLMappableData* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	static bool debugPopulateGeometry = false;
	if (debugPopulateGeometry)
		printf("> Begin populate geometry\n");

	/////////////////////////////////////////////////////////////////////
	// Update data streams based on geometry requirements
	/////////////////////////////////////////////////////////////////////
	updateGeometryRequirements(requirements, data,
		debugPopulateGeometry);

	int numItems = renderItems.length();
	for (int i=0; i<numItems; i++)
	{
        const MHWRender::MRenderItem* item = renderItems.itemAt(i);
		if (!item) continue;

		// Enable to debug vertex buffers that are associated with each render item.
		// Can also use to generate indexing better, but we don't need that here.
		// Also debugs custom data on the render item.
		static const bool debugStuff = false;
		if (debugStuff)
		{
			const MHWRender::MVertexBufferDescriptorList& itemBuffers =
				item->requiredVertexBuffers();
			int numBufs = itemBuffers.length();
			MHWRender::MVertexBufferDescriptor desc;
			for (int bufNum=0; bufNum<numBufs; bufNum++)
			{
				if (itemBuffers.getDescriptor(bufNum, desc))
				{
					printf("Buffer Required for Item #%d ('%s'):\n", i, item->name().asChar());
					printf("\tBufferName: %s\n", desc.name().asChar());
					printf("\tDataType: %s (dimension %d)\n", MHWRender::MGeometry::dataTypeString(desc.dataType()).asChar(), desc.dimension());
					printf("\tSemantic: %s\n", MHWRender::MGeometry::semanticString(desc.semantic()).asChar());
					printf("\n");
				}
			}

			// Just print a message for illustration purposes. Note that the custom data is also
			// accessible from the MRenderItem in MPxShaderOverride::draw().
			/*apiMeshUserData* myCustomData = dynamic_cast<apiMeshUserData*>(item->customData());
			if (myCustomData)
			{
				printf("Custom data on Item #%d: '%s', modified count='%d'\n\n", i, myCustomData->fMessage.asChar(), myCustomData->fNumModifications);
			}
			else
			{
				printf("No custom data on Item #%d\n\n", i);
			}*/
		}

		// Update indexing for active vertex item
		//
		if (item->name() == sActiveVertexItemName)
		{
			updateIndexingForActiveVertices( item, data);
		}

		// Create indexing for dormant vertex render items
		//
		else if (item->primitive() == MHWRender::MGeometry::kPoints)
		{
			MHWRender::MIndexBuffer* indexBuffer = NuiMayaHWMappable::asUInt32IndexBuffer(geomPtr->PointIndices());
			if (indexBuffer)
			{
				data.addIndexBuffer(indexBuffer);
				item->associateWithIndexBuffer(indexBuffer);
			}
		}
		else if (item->primitive() == MHWRender::MGeometry::kLines)
		{
			MHWRender::MIndexBuffer* indexBuffer = NuiMayaHWMappable::asUInt32IndexBuffer(geomPtr->WireframeIndices());
			if (indexBuffer)
			{
				data.addIndexBuffer(indexBuffer);
				item->associateWithIndexBuffer(indexBuffer);
			}
		}
		else if (item->primitive() == MHWRender::MGeometry::kTriangles)
		{
			MHWRender::MIndexBuffer* indexBuffer = NuiMayaHWMappable::asUInt32IndexBuffer(geomPtr->TriangleIndices());
			if (indexBuffer)
			{
				data.addIndexBuffer(indexBuffer);
				item->associateWithIndexBuffer(indexBuffer);
			}
		}
	}

	geomPtr->SetIndexingDirty(false);
	if (debugPopulateGeometry)
		printf("> End populate geometry\n");

}

bool NuiMayaPointCloudGeometryOverride::isIndexingDirty(const MHWRender::MRenderItem& item)
{
	if(!fShape)
		return false;
	NuiCLMappableData* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return false;
	return geomPtr->IsIndexingDirty();
}

bool NuiMayaPointCloudGeometryOverride::isStreamDirty(const MHWRender::MVertexBufferDescriptor& desc)
{
	if(!fShape)
		return false;
	NuiCLMappableData* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return false;
	return geomPtr->IsStreamDirty();
}

void NuiMayaPointCloudGeometryOverride::cleanUp()
{
	fActiveVertexIndices.clear();
}
