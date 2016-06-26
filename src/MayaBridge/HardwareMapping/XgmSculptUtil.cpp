#include <xgmaya/XgmBrushContext.h> // QT 1st

#include <xgmaya/XgmSculptUtil.h>

#include <utility> // std::pair

#include <SeExpr/SeCurve.h>

#include <maya/MCursor.h>
#include <maya/MDagPath.h>
#include <maya/MFnDagNode.h>
#include <maya/MItDag.h>
#include <maya/MString.h>
#include <maya/MPlug.h>
#include <maya/MStringArray.h>
#include <maya/MFnPluginData.h>
#include <maya/MOpenCLInfo.h>
#include <maya/MFnCamera.h>
#include <maya/MFloatMatrix.h>
#include <maya/MGlobal.h>
#include <maya/MVector.h>
#include <maya/MFrameContext.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MFnNurbsCurveData.h>
#include <maya/MPointArray.h>
#include <maya/M3dView.h>
#include <maya/MFnMesh.h>
#include <maya/MItSelectionList.h>
#include <maya/MItDependencyGraph.h>
#include <maya/MQuaternion.h>
#include <maya/MViewport2Renderer.h>
#include <maya/MHWGeometry.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MFnAttribute.h>

// geomtry data populate
bool xgmsculptutil::populateGeomFromPlug(
	XgmBrushContext* brushCtx,
	std::vector<cl_mem>* primitiveInfos,
	std::vector<cl_mem>* positions,
	std::vector<cl_mem>* positions_final,
	std::vector<cl_mem>* texcoords,
	std::vector<cl_mem>* falloffs,
	std::vector<cl_mem>* rttfalloffs,
	std::vector<cl_mem>* cachedBuffers,
	std::vector<cl_mem>* meshN,
	std::vector<cl_mem>* tweaks,
	std::vector<cl_mem>* originalTweaks,
	std::vector<unsigned int>* primitiveCount,
	std::vector<unsigned int>* vertexCount,
	std::vector<cl_mem>* selectionSet,
	std::vector<cl_mem>* frozenSet,
	std::vector<cl_mem>* originalFrozenSet,
	std::vector<cl_mem>* widthPerSpline,
	std::vector<cl_mem>* widthPerCV
	)
{
	// Fetch data from final description node
	if (positions_final || frozenSet || originalFrozenSet)
	{
		std::shared_ptr<XgSplineData> splineData =
			xgmsculptutil::getOutSplineData(brushCtx->descNode());
		if (splineData) {
			for (auto it = splineData->itemBegin(); it != splineData->itemEnd(); it++)
			{
				if (positions_final)
					positions_final->push_back(XgmHWMappable::asPosition3fBufferCL(it->second->positions()));

				if (frozenSet)
					frozenSet->push_back(XgmHWMappable::asTexture1fBufferCL(it->second->frozenSet()));

				if(originalFrozenSet) {
					if (it->second->originalFrozenSet().size() != it->second->frozenSet().size())
					{
						XgMappableAccessor::reset(it->second->originalFrozenSet());
						XgMappableAccessor::initImpl<XgVectorMappableImplf>(
							it->second->originalFrozenSet(), it->second->frozenSet().size()
							);
					}
					originalFrozenSet->push_back(XgmHWMappable::asTexture1fBufferCL(it->second->originalFrozenSet()));
				}
			}
		}
	}

	// Fetch data from current sculpt node
	{
		std::shared_ptr<XgSplineData> splineData =
			xgmsculptutil::getOutSplineData(brushCtx->sculptNode());
		if (!splineData) return false;

		if (splineData->itemBegin() == splineData->itemEnd()) return false;

		std::shared_ptr<XgSplineTweakData> tweakData;
		if (tweaks)
		{
			tweakData = getSplineTweakData(brushCtx);
			assert(tweakData);
		}

		cl_context          context = MOpenCLInfo::getOpenCLContext();
		// Create OpenCL wrapper objects
		for (auto it = splineData->itemBegin(); it != splineData->itemEnd(); it++)
		{
			if (primitiveInfos)
				primitiveInfos->push_back(XgmHWMappable::asUInt32IndexBufferCL(it->second->primitiveInfos()));
			if (positions)
				positions->push_back(XgmHWMappable::asPosition3fBufferCL(it->second->positions()));
			if (texcoords)
				texcoords->push_back(XgmHWMappable::asTexture2fBufferCL(it->second->texcoords()));
			if (falloffs)
				falloffs->push_back(XgmHWMappable::asTexture1fBufferCL(it->second->falloffs()));
			if (rttfalloffs)
				rttfalloffs->push_back(XgmHWMappable::asTexture1fBufferCL(it->second->rootToTipFalloffs()));
			if (cachedBuffers) {
				cl_mem b = XgGPUMemManager::instance().CreateBufferCL(
					context, CL_MEM_READ_WRITE, it->second->positions().size()*sizeof(float), NULL, NULL, "cachedBuffers");
				cachedBuffers->push_back(b);
			}
			if (meshN)
				meshN->push_back(XgmHWMappable::asPosition3fBufferCL(it->second->meshN()));
			if (tweaks)
			{
				std::shared_ptr<XgSplineTweakDataItem> tweakDataItem = tweakData->getItem(it->second->getId());
				assert(tweakDataItem);
				tweaks->push_back(XgmHWMappable::asPosition3fBufferCL(tweakDataItem->tweaks()));
			}
			if (originalTweaks)
			{
				std::shared_ptr<XgSplineTweakDataItem> tweakDataItem = tweakData->getItem(it->second->getId());
				assert(tweakDataItem);
				if (tweakDataItem->originalTweaks().size() != tweakDataItem->tweaks().size())
				{
					XgMappableAccessor::reset(tweakDataItem->originalTweaks());
					XgMappableAccessor::initImpl<XgVectorMappableImpl3f>(
						tweakDataItem->originalTweaks(), tweakDataItem->tweaks().size()
						);
				}
				originalTweaks->push_back(XgmHWMappable::asTexture3fBufferCL(tweakDataItem->originalTweaks()));
			}
			if (primitiveCount)
				primitiveCount->push_back(it->second->primitiveInfos().size() / XG_SPLINE_DATA_PRIMITIVE_INFO_STRIDE);
			if (vertexCount)
				vertexCount->push_back(it->second->positions().size());
			if (selectionSet)
				selectionSet->push_back(XgmHWMappable::asTexture1fBufferCL(it->second->selectionSet()));
			if (widthPerSpline)
				widthPerSpline->push_back(XgmHWMappable::asTexture1fBufferCL(it->second->widthPerSpline()));
			if (widthPerCV)
				widthPerCV->push_back(XgmHWMappable::asTexture1fBufferCL(it->second->widthPerCV()));
		}
	}

	return true;
}

// wrapper for MRenderer::holdGPUMemory( MInt64 sizeInBytes, MInt64* evictedGPUMemSize )
bool xgmsculptutil::informMayaHoldGPU(unsigned int size, unsigned int* evictedGPUMemSize/* = nullptr*/)
{
    if (evictedGPUMemSize) {
        *evictedGPUMemSize = 0;
    }

    MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
    assert(renderer);
    if (!renderer) {
        return false;
    }

    MInt64 evicted = 0;
    const MStatus result = renderer->holdGPUMemory((MInt64)size, &evicted);
    if (evictedGPUMemSize) {
        *evictedGPUMemSize = (unsigned int)evicted;
    }

    return result == MS::kSuccess;
}

// wrapper for MRenderer::releaseGPUMemory( MInt64 sizeInBytes )
bool xgmsculptutil::informMayaReleaseGPU(unsigned int size)
{
    MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
    assert(renderer);
    if (!renderer) {
        return false;
    }

    return MS::kSuccess == renderer->releaseGPUMemory((MInt64)size);
}

// wrapper for 'lockResourceHandle'
void xgmsculptutil::lockResourceHandle(void* ogsBuf, XgGPUMemSharedType type)
{
    assert(ogsBuf);
    if (!ogsBuf)
        return;

    if (type == XgGPUMemSharedType::XG_GPUMEM_SHARED_VB) {
        ((MHWRender::MVertexBuffer*)ogsBuf)->lockResourceHandle();
    }
    else if(type == XgGPUMemSharedType::XG_GPUMEM_SHARED_IB) {
        ((MHWRender::MIndexBuffer*)ogsBuf)->lockResourceHandle();
    }
    else {
        assert(false);
        XG_ERROR("Unrecognized gpu sharing type!\n");
    }
}

// wrapper for 'unlockResourceHandle'
void xgmsculptutil::unlockResourceHandle(void* ogsBuf, XgGPUMemSharedType type)
{
    assert(ogsBuf);
    if (!ogsBuf)
        return;

    if (type == XgGPUMemSharedType::XG_GPUMEM_SHARED_VB) {
        ((MHWRender::MVertexBuffer*)ogsBuf)->unlockResourceHandle();
    }
    else if (type == XgGPUMemSharedType::XG_GPUMEM_SHARED_IB) {
        ((MHWRender::MIndexBuffer*)ogsBuf)->unlockResourceHandle();
    }
    else {
        assert(false);
        XG_ERROR("Unrecognized gpu sharing type!\n");
    }
}


