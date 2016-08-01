#include "../api_macros.h"
#include "NuiMayaDeviceGrabber.h"
#include "NuiMayaPreviewerRenderOverride.h"
#include "Frame/Buffer/NuiFrameCache.h"
#include "Frame/Buffer/NuiFrameBuffer.h"
#include "DeviceManager/NuiRGBDDeviceController.h"
#include "SLAM/NuiKinfuManager.h"
#include "../SkeletonDriver/NuiMayaSkeletonData.h"
#include "../SkeletonDriver/NuiMayaGestureData.h"
#include "../SkeletonDriver/NuiMayaFacialModelData.h"
#include "../PointCloudShape/NuiMayaImageCLData.h"
#include "Shape/NuiMeshShape.h"
#include "Frame/NuiFrameUtilities.h"

#include <maya/MTime.h>
#include <maya/MGlobal.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnPluginData.h>

#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MFloatPointArray.h>
#include <maya/MFloatArray.h>
#include <maya/MDistance.h>

static std::string sTestDataFolder = getenv("NUI_TESTDATA") ? getenv("NUI_TESTDATA") : "G:\\tmp\\";

MTypeId     NuiMayaDeviceGrabber::id( 0x80089 );

// Attributes
// 
MObject     NuiMayaDeviceGrabber::aTime;
MObject		NuiMayaDeviceGrabber::aDeviceOn;
MObject		NuiMayaDeviceGrabber::aDeviceMode;
MObject		NuiMayaDeviceGrabber::aUseCache;
MObject		NuiMayaDeviceGrabber::aNearMode;
MObject		NuiMayaDeviceGrabber::aElevationAngle;
MObject		NuiMayaDeviceGrabber::aPreviewerOn;
MObject		NuiMayaDeviceGrabber::aKinFuOn;
MObject		NuiMayaDeviceGrabber::aVolumeVoxelSize;
MObject		NuiMayaDeviceGrabber::aShowMesh;
MObject		NuiMayaDeviceGrabber::aShowInvalid;
MObject		NuiMayaDeviceGrabber::aShowOnlyBody;
MObject		NuiMayaDeviceGrabber::aMinDepth;
MObject		NuiMayaDeviceGrabber::aMaxDepth;
MObject     NuiMayaDeviceGrabber::aOutputPointCloud;
MObject     NuiMayaDeviceGrabber::aOutputMesh;
MObject     NuiMayaDeviceGrabber::aOutputSkeleton;
MObject     NuiMayaDeviceGrabber::aOutputGesture;
MObject     NuiMayaDeviceGrabber::aOutputFacialModel;
MObject		NuiMayaDeviceGrabber::aFaceRotateX;
MObject		NuiMayaDeviceGrabber::aFaceRotateY;
MObject		NuiMayaDeviceGrabber::aFaceRotateZ;
MObject		NuiMayaDeviceGrabber::aFaceRotate;
MObject		NuiMayaDeviceGrabber::aCameraRotateX;
MObject		NuiMayaDeviceGrabber::aCameraRotateY;
MObject		NuiMayaDeviceGrabber::aCameraRotateZ;
MObject		NuiMayaDeviceGrabber::aCameraRotate;
MObject		NuiMayaDeviceGrabber::aCameraTranslateX;
MObject		NuiMayaDeviceGrabber::aCameraTranslateY;
MObject		NuiMayaDeviceGrabber::aCameraTranslateZ;
MObject		NuiMayaDeviceGrabber::aCameraTranslate;

NuiMayaDeviceGrabber::NuiMayaDeviceGrabber()
	: m_pDevice(NULL)
	, m_pCache(NULL)
	, m_pPreviewer(NULL)
	, m_kinfu(NULL)
{
}
NuiMayaDeviceGrabber::~NuiMayaDeviceGrabber()
{
	if(m_pDevice)
		m_pDevice->stopDevice();
	SafeDelete(m_pDevice);
	SafeDelete(m_pPreviewer);
	SafeDelete(m_kinfu);

	if(m_pCache)
		m_pCache->clear();
	SafeDelete(m_pCache);

	removeCallbacks();
}

void NuiMayaDeviceGrabber::attrChangedCB(MNodeMessage::AttributeMessage msg, MPlug & plug, MPlug & otherPlug, void* clientData)
{
	NuiMayaDeviceGrabber *wrapper = static_cast<NuiMayaDeviceGrabber *>(clientData);
	if(!wrapper)
		return;

	MStatus stat;
	MObject attr = plug.attribute(&stat);
	MFnAttribute fnAttr(attr);

	if (fnAttr.name() == "deviceOn" ||
		fnAttr.name() == "deviceMode" ) {
		wrapper->updateDevice();
	}
	else if (fnAttr.name() == "nearMode") {
		wrapper->updateNearMode();
	}
	else if (fnAttr.name() == "elevationAngle") {
		wrapper->updateaElevationAngle();
	}
	else if (fnAttr.name() == "previewerOn") {
		wrapper->updatePreviewer();
	}
	else if (fnAttr.name() == "fusionOn") {
		wrapper->updateKinfu();
	}
}

void NuiMayaDeviceGrabber::addCallbacks()
{
	MStatus status;
	
	MObject node = thisMObject();
	fCallbackIds.append( MNodeMessage::addAttributeChangedCallback(node, attrChangedCB, (void*)this) );
}

void NuiMayaDeviceGrabber::removeCallbacks()
{
	MMessage::removeCallbacks(fCallbackIds);
}

void NuiMayaDeviceGrabber::postConstructor()
{
	// Avoid node to be auto deleted when all connection breaks
	setExistWithoutInConnections(true);
	setExistWithoutOutConnections(true);

	addCallbacks();

	SafeDelete(m_pCache);
	m_pCache = new NuiFrameBuffer();

	SafeDelete(m_pDevice);
	m_pDevice = new NuiRGBDDeviceController();
}

void* NuiMayaDeviceGrabber::creator()
//
//	Description:
//		this method exists to give Maya a way to create new objects
//      of this type. 
//
//	Return Value:
//		a new object of this type
//
{
	return new NuiMayaDeviceGrabber();
}

MStatus NuiMayaDeviceGrabber::initialize()
//
//	Description:
//		This method is called to create and initialize all of the attributes
//      and attribute dependencies for this node type.  This is only called 
//		once when the node type is registered with Maya.
//
//	Return Values:
//		MS::kSuccess
//		MS::kFailure
//
{
	// This sample creates a single input float attribute and a single
	// output float attribute.
	//
	MFnNumericAttribute nAttr;
	MFnTypedAttribute	typedAttr;
	MFnUnitAttribute	unitAttr;
	MFnEnumAttribute	enumAttr;
	MStatus				stat;

	aTime = unitAttr.create( "time", "tm",
		MFnUnitAttribute::kTime,
		0.0, &stat );
	stat = addAttribute( aTime );
	if (!stat) { stat.perror("addAttribute time"); return stat;}

	aUseCache = nAttr.create( "useCache", "uc", MFnNumericData::kBoolean, false );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	stat = addAttribute( aUseCache );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aDeviceOn = nAttr.create( "deviceOn", "do", MFnNumericData::kBoolean, false );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	stat = addAttribute( aDeviceOn );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aDeviceMode = enumAttr.create( "deviceMode", "dm", NuiRGBDDeviceController::EDeviceMode_VertexColorCamera, &stat );
	if (!stat) { stat.perror("create DeviceMode attribute"); return stat;}
	stat = enumAttr.addField( "Depth,Color", NuiRGBDDeviceController::EDeviceMode_DepthColor );
	if (!stat) { stat.perror("add enum type DepthColor"); return stat;}
	stat = enumAttr.addField( "Depth,Color,Player", NuiRGBDDeviceController::EDeviceMode_VertexColorCamera );
	if (!stat) { stat.perror("add enum type DepthColorPlayer"); return stat;}
	stat = enumAttr.addField( "Depth,Color,Skeleton", NuiRGBDDeviceController::EDeviceMode_VertexColorSkeleton );
	if (!stat) { stat.perror("add enum type DepthColorSkeleton"); return stat;}
	stat = enumAttr.addField( "Depth,Color,Skeleton,Face", NuiRGBDDeviceController::EDeviceMode_VertexColorSkeletonFace );
	if (!stat) { stat.perror("add enum type DepthColorSkeletonFace"); return stat;}
	stat = enumAttr.addField( "Depth,Color,Skeleton,Gesture", NuiRGBDDeviceController::EDeviceMode_VertexColorSkeletonGesture );
	if (!stat) { stat.perror("add enum type DepthColorSkeletonGesture"); return stat;}
	stat = enumAttr.addField( "Color", NuiRGBDDeviceController::EDeviceMode_Color );
	if (!stat) { stat.perror("add enum type Color"); return stat;}
	stat = enumAttr.addField( "Depth", NuiRGBDDeviceController::EDeviceMode_Vertex );
	if (!stat) { stat.perror("add enum type Depth"); return stat;}
	stat = enumAttr.addField( "Skeleton", NuiRGBDDeviceController::EDeviceMode_Skeleton );
	if (!stat) { stat.perror("add enum type Skeleton"); return stat;}
	stat = enumAttr.addField( "Fusion", NuiRGBDDeviceController::EDeviceMode_Fusion );
	if (!stat) { stat.perror("add enum type fusion"); return stat;}
	CHECK_MSTATUS( enumAttr.setHidden( false ) );
	CHECK_MSTATUS( enumAttr.setKeyable( false ) );
	stat = addAttribute( aDeviceMode );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aNearMode = nAttr.create( "nearMode", "ne", MFnNumericData::kBoolean, false );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	stat = addAttribute( aNearMode );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aElevationAngle = nAttr.create( "elevationAngle", "ea", MFnNumericData::kInt, 0 );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	nAttr.setKeyable(true);
	nAttr.setMin(-27);
	nAttr.setMax(27);
	// Attribute is keyable and will show up in the channel box
	stat = addAttribute( aElevationAngle );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aPreviewerOn = nAttr.create( "previewerOn", "po", MFnNumericData::kBoolean, false );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	stat = addAttribute( aPreviewerOn );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aKinFuOn = nAttr.create( "fusionOn", "fo", MFnNumericData::kBoolean, false );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	stat = addAttribute( aKinFuOn );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aVolumeVoxelSize = nAttr.create( "volumeVoxelSize", "vvs", MFnNumericData::kFloat, 0.01f );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	nAttr.setKeyable(true);
	nAttr.setMin(0.005f);
	nAttr.setMax(0.02f);
	// Attribute is keyable and will show up in the channel box
	stat = addAttribute( aVolumeVoxelSize );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aShowInvalid = nAttr.create("showInvalid", "siv", MFnNumericData::kBoolean, false, &stat);
	MCHECKERROR( stat, "create showInvalid attribute" )
	nAttr.setKeyable(true);
	ADD_ATTRIBUTE( aShowInvalid );

	aShowOnlyBody = nAttr.create("showOnlyBody", "sc", MFnNumericData::kBoolean, false, &stat);
	MCHECKERROR( stat, "create showOnlyBody attribute" )
	nAttr.setKeyable(true);
	ADD_ATTRIBUTE( aShowOnlyBody );

	aShowMesh = nAttr.create("showMesh", "sm", MFnNumericData::kBoolean, false, &stat);
	MCHECKERROR( stat, "create showMesh attribute" )
	nAttr.setKeyable(true);
	ADD_ATTRIBUTE( aShowMesh );

	aMinDepth = nAttr.create( "nearPlane", "np", MFnNumericData::kShort, 400 );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	nAttr.setKeyable(true);
	nAttr.setMin(400);
	nAttr.setMax(4500);
	// Attribute is keyable and will show up in the channel box
	stat = addAttribute( aMinDepth );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aMaxDepth = nAttr.create( "farPlane", "fp", MFnNumericData::kShort, 4200 );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	nAttr.setKeyable(true);
	nAttr.setMin(400);
	nAttr.setMax(4500);
	// Attribute is keyable and will show up in the channel box
	stat = addAttribute( aMaxDepth );
	if (!stat) { stat.perror("addAttribute"); return stat;}


	// ----------------------- OUTPUTS -------------------------
	aOutputPointCloud = typedAttr.create( "outputPointCloud", "opc",
		NuiMayaImageCLData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create outputPointCloud attribute"); return stat;}
	typedAttr.setWritable( false );
	typedAttr.setStorable(false);
	stat = addAttribute( aOutputPointCloud );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aOutputSkeleton = typedAttr.create( "outputSkeleton", "osk",
		NuiMayaSkeletonData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create outputSkeleton attribute"); return stat;}
	typedAttr.setWritable( false );
	typedAttr.setStorable(false);
	stat = addAttribute( aOutputSkeleton );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aOutputMesh = typedAttr.create( "outputMesh", "om",
		MFnData::kMesh,
		MObject::kNullObj, &stat );
	MCHECKERROR( stat, "create outputSurface attribute" )
		typedAttr.setWritable( false );
	ADD_ATTRIBUTE( aOutputMesh );

	aOutputGesture = typedAttr.create( "outputGesture", "ogs",
		NuiMayaGestureData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create outputGesture attribute"); return stat;}
	typedAttr.setWritable( false );
	typedAttr.setStorable(false);
	stat = addAttribute( aOutputGesture );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aOutputFacialModel = typedAttr.create( "outputFacialModel", "ofm",
		NuiMayaFacialModelData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create outputFacialModel attribute"); return stat;}
	typedAttr.setWritable( false );
	stat = addAttribute( aOutputFacialModel );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aFaceRotateX = nAttr.create( "faceRotateX", "frX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create face rotateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aFaceRotateY = nAttr.create( "faceRotateY", "frY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create face rotateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aFaceRotateZ = nAttr.create( "faceRotateZ", "frZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rotateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aFaceRotate = nAttr.create( "faceRotate", "fr", aFaceRotateX, aFaceRotateY, aFaceRotateZ, &stat );
	if (!stat) { stat.perror("create face rotate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aFaceRotate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aCameraRotateX = nAttr.create( "cameraRotateX", "crX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create cameraRotateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCameraRotateY = nAttr.create( "cameraRotateY", "crY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rotateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCameraRotateZ = nAttr.create( "cameraRotateZ", "crZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rotateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCameraRotate = nAttr.create( "cameraRotate", "cr", aCameraRotateX, aCameraRotateY, aCameraRotateZ, &stat );
	if (!stat) { stat.perror("create cameraRotate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aCameraRotate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aCameraTranslateX = nAttr.create( "cameraTranslateX", "ctX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create cameraTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCameraTranslateY = nAttr.create( "cameraTranslateY", "ctY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create cameraTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCameraTranslateZ = nAttr.create( "cameraTranslateZ", "ctZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create cameraTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCameraTranslate = nAttr.create( "cameraTranslate", "ct", aCameraTranslateX, aCameraTranslateY, aCameraTranslateZ, &stat );
	if (!stat) { stat.perror("create cameraTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aCameraTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// Set up a dependency between the input and the output.  This will cause
	// the output to be marked dirty when the input changes.  The output will
	// then be recomputed the next time the value of the output is requested.
	//
	stat = attributeAffects( aTime, aOutputPointCloud );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aOutputSkeleton );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aOutputFacialModel );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aFaceRotateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aFaceRotateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aFaceRotateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aFaceRotate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraRotateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraRotateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraRotateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraRotate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	return MS::kSuccess;
}


/* override */
MStatus NuiMayaDeviceGrabber::connectionMade( const MPlug& plug,
											   const MPlug& otherPlug,
											   bool asSrc )
//
// Description
//
//    Whenever a connection is made to this node, this method
//    will get called.
//
{
	/*if ( plug == aOutputPointCloud ) {
		m_cacheFlags |= NuiDeviceBufferImpl::ECache_CLData;
	}
	else if ( plug == aOutputSkeleton ) {
		m_cacheFlags |= NuiDeviceBufferImpl::ECache_Skeleton;
	}
	else if ( plug == aOutputGesture ) {
		m_cacheFlags |= NuiDeviceBufferImpl::ECache_Gesture;
	}
	else if ( plug == aOutputFacialModel ) {
		m_cacheFlags |= NuiDeviceBufferImpl::ECache_Face;
	}
	if(m_pCache)
		m_pCache->SetFlags(m_cacheFlags);*/

	return MPxNode::connectionMade( plug, otherPlug, asSrc );
}

/* override */
MStatus NuiMayaDeviceGrabber::connectionBroken( const MPlug& plug,
												 const MPlug& otherPlug,
												 bool asSrc )
//
// Description
//
//    Whenever a connection to this node is broken, this method
//    will get called.
//
{
	bool bHasConnected = plug.isConnected();
	/*if(!bHasConnected)
	{
		if ( plug == aOutputPointCloud ) {
			m_cacheFlags = ~(~m_cacheFlags | NuiDeviceBufferImpl::ECache_CLData);
		}
		else if ( plug == aOutputSkeleton ) {
			m_cacheFlags = ~(~m_cacheFlags | NuiDeviceBufferImpl::ECache_Skeleton);
		}
		else if ( plug == aOutputGesture ) {
			m_cacheFlags = ~(~m_cacheFlags | NuiDeviceBufferImpl::ECache_Gesture);
		}
		else if ( plug == aOutputFacialModel ) {
			m_cacheFlags = ~(~m_cacheFlags | NuiDeviceBufferImpl::ECache_Face);
		}
		if(m_pCache)
			m_pCache->SetFlags(m_cacheFlags);
	}*/

	// When aOutputPointCloud connection lost, we should unlock any graphic/compute shared memory
	if (plug == aOutputPointCloud)
	{
		std::shared_ptr<NuiCLMappableData> clData =
			NuiMayaImageCLData::findData(thisMObject(), aOutputPointCloud);

		if (clData) {
			clData->relaxToCPU();
		}
	}
	return MPxNode::connectionBroken( plug, otherPlug, asSrc );
}

MStatus NuiMayaDeviceGrabber::compute( const MPlug& plug, MDataBlock& datablock )
//
//	Description:
//		This method computes the value of the given output plug based
//		on the values of the input attributes.
//
//	Arguments:
//		plug - the plug to compute
//		data - object that provides access to the attributes for this node
//
{
	assert(m_pCache);
	if(!m_pCache)
		return MS::kFailure;

	MStatus returnStatus;
	/* Get time */
	MDataHandle timeData = datablock.inputValue( aTime, &returnStatus ); 
	MCHECKERROR(returnStatus, "Error getting time data handle\n")
	MTime time = timeData.asTime();
	//!< 30 frames per second
	int	  frame = (int)time.as( MTime::kNTSCFrame ) - 1;//Noted: The first frame in MAYA is 1;

	if(m_pDevice)
	{
		std::shared_ptr<NuiCompositeFrame> pFrame = m_pDevice->popFrame();
		if(pFrame)
		{
			pFrame->m_depthFrame.SetMinDepth(getShortValue(aMinDepth));
			pFrame->m_depthFrame.SetMaxDepth(getShortValue(aMaxDepth));

			if(m_kinfu && m_kinfu->isThreadOn())
			{
				m_kinfu->pushbackFrame(pFrame);
			}
			m_pCache->pushbackFrame(pFrame);
			pFrame.reset();
		}
	}

	std::shared_ptr<NuiCompositeFrame> pCurrentFrame = m_pCache->getLatestFrame();
		
	if ( plug == aOutputPointCloud )
	{
		std::shared_ptr<NuiCLMappableData> clData(nullptr);
		MDataHandle outHandle = datablock.outputValue( aOutputPointCloud );
		NuiMayaImageCLData* clmData = static_cast<NuiMayaImageCLData*>(outHandle.asPluginData());
		if(!clmData)
		{
			// Create some user defined geometry data and access the
			// geometry so we can set it
			//
			MFnPluginData fnDataCreator;
			MTypeId tmpid( NuiMayaImageCLData::id );

			fnDataCreator.create( tmpid, &returnStatus );
			MCHECKERROR( returnStatus, "compute : error creating pointCloudData")

			clmData = (NuiMayaImageCLData*)fnDataCreator.data( &returnStatus );
			MCHECKERROR( returnStatus, "compute : error gettin at proxy pointCloudData object")

			clData = std::shared_ptr<NuiCLMappableData>(new NuiCLMappableData());
			clmData->setData(clData);

			returnStatus = outHandle.set( clmData );
			MCHECKERROR( returnStatus, "compute : error gettin at proxy pointCloudData object")
		}
		else
		{
			clData = clmData->data();
		}
		int indexFlags = getBooleanValue(aShowMesh) ? (NuiCLMappableData::E_MappableData_Triangle | NuiCLMappableData::E_MappableData_Wireframe) : NuiCLMappableData::E_MappableData_Point;
		bool bReceived = (m_kinfu && m_kinfu->isThreadOn()) ?
			m_kinfu->getCLData(clData.get(), getBooleanValue(aShowMesh)) :
			NuiFrameUtilities::FrameToMappableData(pCurrentFrame.get(), clData.get(), indexFlags, getBooleanValue(aShowOnlyBody), 0.2f);
			
		datablock.setClean( plug );
	}
	else if ( plug == aOutputMesh )
	{
		NuiMeshShape mesh;
		bool bReceived = (m_kinfu && m_kinfu->isThreadOn()) ?
			m_kinfu->getMesh(&mesh) :
			NuiFrameUtilities::FrameToMesh(pCurrentFrame.get(), &mesh, getBooleanValue(aShowOnlyBody), 0.2f);
		// Create some mesh data and access the
		// geometry so we can set it
		//
		MFnMeshData dataCreator;
		MObject newOutputData = dataCreator.create(&returnStatus);
		MCHECKERROR(returnStatus, "ERROR creating outputData");

		// If there is an input mesh then copy it's values
		// and construct some apiMeshGeom for it.
		//
		returnStatus = bReceived ? computeOutputMesh( &mesh, datablock, newOutputData) : MS::kFailure;
		if(returnStatus != MS::kSuccess)
		{
			createEmptyMesh( newOutputData );
		}
		// Assign the new data to the outputSurface handle
		//
		MDataHandle outHandle = datablock.outputValue( aOutputMesh );
		outHandle.set( newOutputData );
		datablock.setClean( plug );
	}
	else if ( plug == aOutputSkeleton ||
		plug == aOutputGesture )
	{
		if(pCurrentFrame)
		{
			MFnPluginData fnSkeletonDataCreator;
			MTypeId tmpSkeletonId( NuiMayaSkeletonData::id );
			fnSkeletonDataCreator.create( tmpSkeletonId, &returnStatus );
			MCHECKERROR( returnStatus, "compute : error creating skeleton data")
				NuiMayaSkeletonData * newSkeletonData = (NuiMayaSkeletonData*)fnSkeletonDataCreator.data( &returnStatus );
			MCHECKERROR( returnStatus, "compute : error gettin at proxy skeleton data object")
				NuiSkeletonJoints* pSkeleton = newSkeletonData->m_pSkeletonData.get();
			assert(pSkeleton);
			pSkeleton->SetInvalid();

			MFnPluginData fnGestureDataCreator;
			MTypeId tmpGestureId( NuiMayaGestureData::id );
			fnGestureDataCreator.create( tmpGestureId, &returnStatus );
			MCHECKERROR( returnStatus, "compute : error creating gesture data")
				NuiMayaGestureData* newGestureData = (NuiMayaGestureData*)fnGestureDataCreator.data( &returnStatus );
			MCHECKERROR( returnStatus, "compute : error gettin at proxy gesture data object")
				NuiGestureResult* pGesture = newGestureData->m_pGestureResult;
			assert(pGesture);

			for (UINT iBody = 0; iBody < pCurrentFrame->m_skeletonFrame.GetBodyCount(); ++iBody)
			{
				if( pCurrentFrame->m_skeletonFrame.ReadSkeleton(iBody, pSkeleton) )
				{
					pCurrentFrame->m_gestureFrame.ReadGestureResult(iBody, pGesture);
					// Assign the first available skeleton.
					//
					break;
				}
			}
			MDataHandle outSkeletonHandle = datablock.outputValue( aOutputSkeleton );
			outSkeletonHandle.set( newSkeletonData );
			datablock.setClean( aOutputSkeleton );

			MDataHandle outGestureHandle = datablock.outputValue( aOutputGesture );
			outGestureHandle.set( newGestureData );
			datablock.setClean( aOutputGesture );
		}
		else
		{
			MGlobal::displayError( " Failed to acquire the frame from the cache." );
		}
	}
	else if (plug == aOutputFacialModel)
	{
		if(pCurrentFrame)
		{
			// Create some user defined geometry data and access the
			// geometry so we can set it
			//
			MFnPluginData fnDataCreator;
			MTypeId tmpid( NuiMayaFacialModelData::id );

			fnDataCreator.create( tmpid, &returnStatus );
			MCHECKERROR( returnStatus, "compute : error creating faceData")

			NuiMayaFacialModelData * newData = (NuiMayaFacialModelData*)fnDataCreator.data( &returnStatus );
			MCHECKERROR( returnStatus, "compute : error gettin at proxy face data object")

			NuiFacialModel* pFacialModel = newData->m_pFacialModel;
			assert(pFacialModel);

			for (UINT iBody = 0; iBody < pCurrentFrame->m_facialModelFrame.GetBodyCount(); ++iBody)
			{
				if( pCurrentFrame->m_facialModelFrame.ReadFacialModel(iBody, pFacialModel) )
				{
					// Assign the first available facial model.
					//
					break;
				}
			}
			MDataHandle outHandle = datablock.outputValue( aOutputFacialModel );
			outHandle.set( newData );
			datablock.setClean( plug );
		}
		else
		{
			MGlobal::displayError( " Failed to acquire the frame from the cache." );
		}
	}
	else if(plug == aFaceRotate ||
		plug == aFaceRotateX || 
		plug == aFaceRotateY || 
		plug == aFaceRotateZ )
	{
		if(pCurrentFrame)
		{
			NuiTrackedFace face;
			for (UINT iBody = 0; iBody < pCurrentFrame->m_faceTrackingFrame.GetBodyCount(); ++iBody)
			{
				if( pCurrentFrame->m_faceTrackingFrame.ReadTrackedFace(iBody, &face) )
				{
					// Assign the first available face.
					//
					double rotationXYZ[3];
					face.GetRotationXYZ(&rotationXYZ[0], &rotationXYZ[1], &rotationXYZ[2]);
					MDataHandle otHandle = datablock.outputValue( aFaceRotate ); 
					otHandle.set( rotationXYZ[0], rotationXYZ[1], rotationXYZ[2] );
					break;
				}
			}
			datablock.setClean(aFaceRotate);
		}
		else
		{
			MGlobal::displayError( " Failed to acquire the frame from the cache." );
		}
	}
	else if(plug == aCameraRotate ||
		plug == aCameraRotateX || 
		plug == aCameraRotateY || 
		plug == aCameraRotateZ ||
		plug == aCameraTranslate ||
		plug == aCameraTranslateX || 
		plug == aCameraTranslateY || 
		plug == aCameraTranslateZ)
	{
		NuiCameraPos cam;
		bool received = false;
		if(pCurrentFrame)
		{
			cam = pCurrentFrame->GetCameraParams();
			received = true;
		}
		else if(m_kinfu)
		{
			m_kinfu->getCameraPose(&cam);
			received = true;
		}
		if(received)
		{
			//Eigen::Quaternion<float> cameraRot (affine.rotation());
			//double x = cameraRot.x();
			//double y = cameraRot.y();
			//double z = cameraRot.z();
			//double w = cameraRot.w();

			//// convert rotation quaternion to Euler angles in degrees
			//double pitch = atan2( 2 * ( y * z + w * x ), w * w - x * x - y * y + z * z ) / M_PI * 180.0;
			//double yaw = asin( 2 * ( w * y - x * z ) ) / M_PI * 180.0;
			//double roll = atan2( 2 * ( x * y + w * z ), w * w + x * x - y * y - z * z ) / M_PI * 180.0;

			//MDataHandle otHandle = datablock.outputValue( aCameraRotate ); 
			//otHandle.set( pitch, yaw + 180.0, roll );
			datablock.setClean(aCameraRotate);

			Vector3f trans = cam.getTranslation();
			trans[2] = trans[2] - 4.0f;
			trans = cam.getRotation() * trans;
			MDataHandle otHandle = datablock.outputValue( aCameraTranslate ); 
			otHandle.set( (double)trans[0],
				(double)-trans[1],
				(double)trans[2] );
			datablock.setClean(aCameraTranslate);
		}
	}

	return returnStatus;
}

bool NuiMayaDeviceGrabber::updateDevice()
{
	assert(m_pDevice);
	if(!m_pDevice)
		return false;

	MStatus returnStatus;
	MDataBlock datablock = forceCache();

	bool bDeviceOn = false;
	MDataHandle inputHandle = datablock.inputValue( aDeviceOn, &returnStatus );
	if(returnStatus == MS::kSuccess)
		bDeviceOn = inputHandle.asBool();

	if(bDeviceOn)
	{
		short deviceMode = NuiRGBDDeviceController::EDeviceMode_DepthColor;
		MDataHandle inputHandle = datablock.inputValue( aDeviceMode, &returnStatus );
		if(returnStatus == MS::kSuccess)
			deviceMode = inputHandle.asShort();

		bool bUseCache = false;
		inputHandle = datablock.inputValue( aUseCache, &returnStatus );
		if(returnStatus == MS::kSuccess)
			bUseCache = inputHandle.asBool();

		if(bUseCache)
		{
			if( !m_pDevice->startFileLoader((DWORD)deviceMode, sTestDataFolder) )
			{
				MGlobal::displayError( " Failed to start the file Loader." );
				return false;
			}
		}
		else
		{
			if( !m_pDevice->startDevice((DWORD)deviceMode) )
			{
				MGlobal::displayError( " Failed to setup the device." );
				return false;
			}
		}
	}
	else if(m_pDevice)
	{
		m_pDevice->stopDevice();
	}
	return true;
}

void NuiMayaDeviceGrabber::updateaElevationAngle()
{
	MStatus returnStatus;
	MDataBlock datablock = forceCache();

	int elevationAngle = 0;
	MDataHandle inputHandle = datablock.inputValue( aElevationAngle, &returnStatus );
	if(returnStatus == MS::kSuccess)
		elevationAngle = inputHandle.asInt();

	if(m_pDevice)
		m_pDevice->UpdateElevationAngle(elevationAngle);
}

bool NuiMayaDeviceGrabber::updateNearMode()
{
	MStatus returnStatus;
	MDataBlock datablock = forceCache();

	bool bNearMode = false;
	MDataHandle inputHandle = datablock.inputValue( aNearMode, &returnStatus );
	if(returnStatus == MS::kSuccess)
		bNearMode = inputHandle.asBool();

	return (m_pDevice ? m_pDevice->UpdateNearMode(bNearMode) : false);
}

void NuiMayaDeviceGrabber::startPreviewer()
{
	if(!m_pPreviewer)
	{
		m_pPreviewer = new NuiMayaCacheTimer(PreviewerCallingBack);
	}
	assert(m_pPreviewer);
	int interval = 100;
	m_pPreviewer->start(interval);
}

void NuiMayaDeviceGrabber::stopPreviewer()
{
	if(m_pPreviewer)
	{
		m_pPreviewer->stop();
	}
}

void NuiMayaDeviceGrabber::updatePreviewer()
{
	MStatus returnStatus;
	MDataBlock datablock = forceCache();

	bool bPreviewerOn = false;
	MDataHandle inputHandle = datablock.inputValue( aPreviewerOn, &returnStatus );
	if(returnStatus == MS::kSuccess)
		bPreviewerOn = inputHandle.asBool();

	updatePreviewRendererMode();
	if(bPreviewerOn)
		startPreviewer();
	else
		stopPreviewer();
}

bool NuiMayaDeviceGrabber::updatePreviewRendererMode()
{
	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if (!renderer)
		return false;

	NuiMayaPreviewerRenderOverride* renderOverrideInstance = (NuiMayaPreviewerRenderOverride*)
		renderer->findRenderOverride(NuiMayaPreviewerRenderOverride::kNuiPreviewerRendererName);
	if (!renderOverrideInstance)
		return false;

	int flags = NuiMayaPreviewerRenderOverride::EPreview_ColorMode;
	renderOverrideInstance->updatePreviewerMode((NuiMayaPreviewerRenderOverride::PreviewModeFlag)flags);
	return true;
}

void NuiMayaDeviceGrabber::PreviewerCallingBack(void* pVoid)
{
	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if (!renderer)
		return;

	NuiMayaPreviewerRenderOverride* renderOverrideInstance = (NuiMayaPreviewerRenderOverride*)
		renderer->findRenderOverride(NuiMayaPreviewerRenderOverride::kNuiPreviewerRendererName);
	if (!renderOverrideInstance)
		return;

	NuiFrameCacheImpl* pCache = reinterpret_cast<NuiFrameCacheImpl*>(pVoid);
	if (!pCache)
		return;
	
	if( renderOverrideInstance->updatePreviewerTexture(pCache) )
		renderOverrideInstance->refreshView();
}

void NuiMayaDeviceGrabber::updateKinfu()
{
	if(!m_pDevice)
		return;

	MStatus returnStatus;
	MDataBlock datablock = forceCache();

	bool bKinfuOn = false;
	MDataHandle inputHandle = datablock.inputValue( aKinFuOn, &returnStatus );
	if(returnStatus == MS::kSuccess)
		bKinfuOn = inputHandle.asBool();

	if(bKinfuOn)
	{
		if(!m_kinfu)
		{
			if(!m_kinfu)
				m_kinfu = new NuiKinfuManager();

			float voxelSize = 0.01f;
			inputHandle = datablock.inputValue( aVolumeVoxelSize, &returnStatus );
			if(returnStatus == MS::kSuccess)
				voxelSize = inputHandle.asFloat();
			m_kinfu->resetVolume(voxelSize, false);
			m_kinfu->startThread();
		}
	}
	else
	{
		SafeDelete(m_kinfu);
	}
}

bool NuiMayaDeviceGrabber::getBooleanValue(const MObject &attribute)
{
	MDataBlock block = forceCache();
	MDataHandle handle = block.inputValue(attribute);
	return handle.asBool();
}

UINT16 NuiMayaDeviceGrabber::getShortValue(const MObject &attribute)
{
	MDataBlock block = forceCache();
	MDataHandle handle = block.inputValue(attribute);
	return handle.asShort();
}

void NuiMayaDeviceGrabber::createEmptyMesh( MObject& out_empytMesh )
{
	MStatus status;
	MFnMeshData meshData;
	out_empytMesh = meshData.create( &status );
	CHECK_MSTATUS( status );

	MFloatPointArray  	vertexArray;
	MIntArray  	        polygonCounts;
	MIntArray  	        polygonConnects;

	MFnMesh meshCreator;
	MObject newMeshObject = meshCreator.create( 
		0, // nb vertices
		0, // nb triangles 
		vertexArray, 
		polygonCounts, 
		polygonConnects, 
		out_empytMesh );
}


MStatus NuiMayaDeviceGrabber::assignMeshUV( MObject&	meshData, const MIntArray& polygonCounts, const MIntArray& uvIds )
{

	MStatus stat = MS::kSuccess;
	MString uvSetName("uvset1");
	MFnMesh meshFn(meshData);
	stat = meshFn.getCurrentUVSetName(uvSetName);
	if ( stat != MS::kSuccess )
	{
		uvSetName = MString ("uvset1");
		stat = meshFn.createUVSet(uvSetName);
		stat = meshFn.setCurrentUVSetName(uvSetName);
	}

	//stat = meshFn.clearUVs();
	//stat = meshFn.setUVs(uArray,vArray,&uvSetName);
	stat = meshFn.assignUVs(polygonCounts, uvIds, &uvSetName);
	if(stat != MS::kSuccess)
		MGlobal::displayError( " Failed to assign UVs." );
	return stat;
}

MStatus NuiMayaDeviceGrabber::computeOutputMesh(
	NuiMeshShape*		pMesh,
	MDataBlock&			datablock,
	MObject&			meshData)
//
// Description
//
//     This function takes an input surface of type kMeshData and converts
//     the geometry into this nodes attributes.
//     Returns kFailure if nothing is connected.
//
{
	if ( NULL == pMesh ) {
		cerr << "NULL pointCloudGeom found\n";
		return MS::kFailure;
	}

	double distanceUnit = 1.0;
	MDistance::Unit currentUnit = MDistance::internalUnit();
	if(currentUnit != MDistance::kInvalid)
	{
		distanceUnit = MDistance(1.0, MDistance::kMeters).as(currentUnit);
	}
	else
	{
		MGlobal::displayError( " Invalid distance unit." );
		return MS::kFailure;
	}

	MFloatPointArray vertexArray;
	MIntArray polygonCounts;
	MIntArray polygonConnects;
	MFloatArray uArray;
	MFloatArray vArray;
	MColorArray colorArray;
	MIntArray vertexIndices;

	const UINT numPoints = (UINT)pMesh->pointsNum();
	for (UINT i = 0; i < numPoints; ++i)
	{
		const SgVec3f& pt = pMesh->getPoint(i);
		MPoint pnt(pt[0], pt[1], pt[2]);
		vertexArray.append(pnt);
		SgVec2f uv = pMesh->getUV(i);
		uArray.append(uv[0]);
		vArray.append(uv[1]);
		SgVec3f color = pMesh->getColor(i);
		colorArray.append(color[0], color[1], color[2]);
		vertexIndices.append(i);
	}
	const UINT numTris = (UINT)pMesh->trianglesNum();
	for (UINT i = 0; i < numTris; ++i)
	{
		int index1 = pMesh->triangleIndex(3*i);
		int index2 = pMesh->triangleIndex(3*i+1);
		int index3 = pMesh->triangleIndex(3*i+2);
		if(index1 < 0 || index1 >= (int)numPoints || index2 < 0 || index2 >= (int)numPoints || index3 < 0 || index3 >= (int)numPoints)
			continue;

		polygonConnects.append(index1);
		polygonConnects.append(index2);
		polygonConnects.append(index3);

		polygonCounts.append(3);
	}

	MStatus stat;
	MFnMesh meshFn;
	meshFn.create(vertexArray.length(), polygonCounts.length(), vertexArray, polygonCounts, polygonConnects, uArray, vArray, meshData, &stat);

	meshFn.setVertexColors(colorArray, vertexIndices);

	bool needAssignUV = true;
	/*MDataHandle inputHandle = datablock.inputValue( aAssignUV, &stat );
	if(stat == MS::kSuccess)
		needAssignUV = inputHandle.asBool();*/
	if(needAssignUV)
	//	assignMeshUV(meshData, polygonCounts, polygonConnects);
	{
		MString uvSetName("uvset1");
		MFnMesh meshFn(meshData);
		stat = meshFn.getCurrentUVSetName(uvSetName);
		if ( stat != MS::kSuccess )
		{
			uvSetName = MString ("uvset1");
			stat = meshFn.createUVSet(uvSetName);
			stat = meshFn.setCurrentUVSetName(uvSetName);
		}

		stat = meshFn.clearUVs();
		stat = meshFn.setUVs(uArray,vArray,&uvSetName);
		stat = meshFn.assignUVs(polygonCounts, polygonConnects, &uvSetName);
		if(stat != MS::kSuccess)
			MGlobal::displayError( " Failed to assign UVs." );
	}
	return stat;
}