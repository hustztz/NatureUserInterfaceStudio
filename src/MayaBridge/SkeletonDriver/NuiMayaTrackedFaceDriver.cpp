#include "NuiMayaTrackedFaceDriver.h"

#include "NuiMayaTrackedFaceData.h"
#include "Shape/NuiTrackedFace.h"

#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnPluginData.h>
#include <maya/MGlobal.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MFloatPointArray.h>

MTypeId     NuiMayaTrackedFaceDriver::id( 0x08471B );

// Attributes
//
MObject		NuiMayaTrackedFaceDriver::aInputData;
MObject		NuiMayaTrackedFaceDriver::aRotateX;
MObject		NuiMayaTrackedFaceDriver::aRotateY;
MObject		NuiMayaTrackedFaceDriver::aRotateZ;
MObject		NuiMayaTrackedFaceDriver::aRotate;
MObject		NuiMayaTrackedFaceDriver::aEyeLeftTranslateX;
MObject		NuiMayaTrackedFaceDriver::aEyeLeftTranslateY;
MObject		NuiMayaTrackedFaceDriver::aEyeLeftTranslateZ;
MObject		NuiMayaTrackedFaceDriver::aEyeLeftTranslate;
MObject		NuiMayaTrackedFaceDriver::aEyeRightTranslateX;
MObject		NuiMayaTrackedFaceDriver::aEyeRightTranslateY;
MObject		NuiMayaTrackedFaceDriver::aEyeRightTranslateZ;
MObject		NuiMayaTrackedFaceDriver::aEyeRightTranslate;
MObject		NuiMayaTrackedFaceDriver::aNoseTranslateX;
MObject		NuiMayaTrackedFaceDriver::aNoseTranslateY;
MObject		NuiMayaTrackedFaceDriver::aNoseTranslateZ;
MObject		NuiMayaTrackedFaceDriver::aNoseTranslate;
MObject		NuiMayaTrackedFaceDriver::aMouthCornerLeftTranslateX;
MObject		NuiMayaTrackedFaceDriver::aMouthCornerLeftTranslateY;
MObject		NuiMayaTrackedFaceDriver::aMouthCornerLeftTranslateZ;
MObject		NuiMayaTrackedFaceDriver::aMouthCornerLeftTranslate;
MObject		NuiMayaTrackedFaceDriver::aMouthCornerRightTranslateX;
MObject		NuiMayaTrackedFaceDriver::aMouthCornerRightTranslateY;
MObject		NuiMayaTrackedFaceDriver::aMouthCornerRightTranslateZ;
MObject		NuiMayaTrackedFaceDriver::aMouthCornerRightTranslate;

NuiMayaTrackedFaceDriver::NuiMayaTrackedFaceDriver()
{
}
NuiMayaTrackedFaceDriver::~NuiMayaTrackedFaceDriver()
{
}

void NuiMayaTrackedFaceDriver::postConstructor()
{

}

MStatus NuiMayaTrackedFaceDriver::compute( const MPlug& plug, MDataBlock& datablock )
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
	MStatus returnStatus;

	/* Get face data */
	MDataHandle faceTrackingHandle = datablock.inputValue( aInputData, &returnStatus );
	if (!returnStatus) {
		returnStatus.perror("Error getting skeleton data handle\n");
		return returnStatus;
	}
	NuiMayaTrackedFaceData* pData = (NuiMayaTrackedFaceData*)faceTrackingHandle.asPluginData();
	if ( NULL == pData ) {
		cerr << "NULL face data found\n";
		return MS::kFailure;
	}
	NuiTrackedFace* pFaceShape = pData->m_pTrackedFace;
	if ( NULL == pFaceShape ) {
		cerr << "NULL face shape found\n";
		return MS::kFailure;
	}

	if(plug == aRotate ||
		plug == aRotateX || 
		plug == aRotateY || 
		plug == aRotateZ )
	{
		double rotationXYZ[3];
		pFaceShape->GetRotationXYZ(&rotationXYZ[0], &rotationXYZ[1], &rotationXYZ[2]);
		MDataHandle otHandle = datablock.outputValue( aRotate ); 
		otHandle.set( rotationXYZ[0], rotationXYZ[1], rotationXYZ[2] );
		datablock.setClean(aRotate);
	}
	else if( plug == aEyeLeftTranslate ||
		plug == aEyeLeftTranslateX || 
		plug == aEyeLeftTranslateY || 
		plug == aEyeLeftTranslateZ )
	{
		/*PointF 
		Vector3		centerPt = pFaceShape->GetFacePoint();

		MDataHandle otHandle = datablock.outputValue( aCenterTranslate ); 
		otHandle.set( (double)centerPt.x,
		(double)centerPt.y,
		(double)centerPt.z );
		datablock.setClean(aCenterTranslate);*/
	}
	else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}


void* NuiMayaTrackedFaceDriver::creator()
//
//	Description:
//		this method exists to give Maya a way to create new objects
//      of this type. 
//
//	Return Value:
//		a new object of this type
//
{
	return new NuiMayaTrackedFaceDriver();
}

MStatus NuiMayaTrackedFaceDriver::initialize()
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
	MStatus				stat;

	// ----------------------- INPUTS --------------------------
	aInputData = typedAttr.create( "inputData", "isd",
		NuiMayaTrackedFaceData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create inputData attribute"); return stat;}
	typedAttr.setReadable( false );
	typedAttr.setWritable( true );
	typedAttr.setStorable( false );
	stat = addAttribute( aInputData );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// ----------------------- OUTPUTS -------------------------
	aRotateX = nAttr.create( "rotateX", "rX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rotateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRotateY = nAttr.create( "rotateY", "rY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rotateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRotateZ = nAttr.create( "rotateZ", "rZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rotateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRotate = nAttr.create( "rotate", "ro", aRotateX, aRotateY, aRotateZ, &stat );
	if (!stat) { stat.perror("create rotate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRotate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// Set up a dependency between the input and the output.  This will cause
	// the output to be marked dirty when the input changes.  The output will
	// then be recomputed the next time the value of the output is requested.
	//
	stat = attributeAffects( aInputData, aRotateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRotateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRotateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRotate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	return MS::kSuccess;
}