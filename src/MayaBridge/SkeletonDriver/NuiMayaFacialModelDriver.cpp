#include "NuiMayaFacialModelDriver.h"

#include "NuiMayaImageData.h"
#include "NuiMayaFacialModelData.h"
#include "Shape/NuiFacialModel.h"

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

MTypeId     NuiMayaFacialModelDriver::id( 0x08471B );

// Attributes
//
MObject		NuiMayaFacialModelDriver::aInputFacialData;
MObject		NuiMayaFacialModelDriver::aInputImageData;
MObject		NuiMayaFacialModelDriver::aRotateX;
MObject		NuiMayaFacialModelDriver::aRotateY;
MObject		NuiMayaFacialModelDriver::aRotateZ;
MObject		NuiMayaFacialModelDriver::aRotate;
MObject		NuiMayaFacialModelDriver::aEyeLeftTranslateX;
MObject		NuiMayaFacialModelDriver::aEyeLeftTranslateY;
MObject		NuiMayaFacialModelDriver::aEyeLeftTranslateZ;
MObject		NuiMayaFacialModelDriver::aEyeLeftTranslate;
MObject		NuiMayaFacialModelDriver::aEyeRightTranslateX;
MObject		NuiMayaFacialModelDriver::aEyeRightTranslateY;
MObject		NuiMayaFacialModelDriver::aEyeRightTranslateZ;
MObject		NuiMayaFacialModelDriver::aEyeRightTranslate;
MObject		NuiMayaFacialModelDriver::aNoseTranslateX;
MObject		NuiMayaFacialModelDriver::aNoseTranslateY;
MObject		NuiMayaFacialModelDriver::aNoseTranslateZ;
MObject		NuiMayaFacialModelDriver::aNoseTranslate;
MObject		NuiMayaFacialModelDriver::aMouthCornerLeftTranslateX;
MObject		NuiMayaFacialModelDriver::aMouthCornerLeftTranslateY;
MObject		NuiMayaFacialModelDriver::aMouthCornerLeftTranslateZ;
MObject		NuiMayaFacialModelDriver::aMouthCornerLeftTranslate;
MObject		NuiMayaFacialModelDriver::aMouthCornerRightTranslateX;
MObject		NuiMayaFacialModelDriver::aMouthCornerRightTranslateY;
MObject		NuiMayaFacialModelDriver::aMouthCornerRightTranslateZ;
MObject		NuiMayaFacialModelDriver::aMouthCornerRightTranslate;
MObject		NuiMayaFacialModelDriver::aJawOpen;
MObject		NuiMayaFacialModelDriver::aLipPucker;
MObject		NuiMayaFacialModelDriver::aJawSlideRight;
MObject		NuiMayaFacialModelDriver::aLipStretcherRight;
MObject		NuiMayaFacialModelDriver::aLipStretcherLeft;
MObject		NuiMayaFacialModelDriver::aLipCornerPullerLeft;
MObject		NuiMayaFacialModelDriver::aLipCornerPullerRight;
MObject		NuiMayaFacialModelDriver::aLipCornerDepressorLeft;
MObject		NuiMayaFacialModelDriver::aLipCornerDepressorRight;
MObject		NuiMayaFacialModelDriver::aLeftcheekPuff;
MObject		NuiMayaFacialModelDriver::aRightcheekPuff;
MObject		NuiMayaFacialModelDriver::aLefteyeClosed;
MObject		NuiMayaFacialModelDriver::aRighteyeClosed;
MObject		NuiMayaFacialModelDriver::aRighteyebrowLowerer;
MObject		NuiMayaFacialModelDriver::aLefteyebrowLowerer;
MObject		NuiMayaFacialModelDriver::aLowerlipDepressorLeft;
MObject		NuiMayaFacialModelDriver::aLowerlipDepressorRight;

NuiMayaFacialModelDriver::NuiMayaFacialModelDriver()
{
}
NuiMayaFacialModelDriver::~NuiMayaFacialModelDriver()
{
}

void NuiMayaFacialModelDriver::postConstructor()
{

}

MStatus NuiMayaFacialModelDriver::compute( const MPlug& plug, MDataBlock& datablock )
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
	MDataHandle faceTrackingHandle = datablock.inputValue( aInputFacialData, &returnStatus );
	if (!returnStatus) {
		returnStatus.perror("Error getting skeleton data handle\n");
		return returnStatus;
	}
	NuiMayaFacialModelData* pData = (NuiMayaFacialModelData*)faceTrackingHandle.asPluginData();
	if ( NULL == pData ) {
		cerr << "NULL face data found\n";
		return MS::kFailure;
	}
	NuiFacialModel* pFaceShape = pData->m_pFacialModel;
	if ( NULL == pFaceShape ) {
		cerr << "NULL face shape found\n";
		return MS::kFailure;
	}

	if(plug == aJawOpen )
	{
		MDataHandle otHandle = datablock.outputValue( aJawOpen ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_JawOpen) );
		datablock.setClean(aJawOpen);
	}
	else if(plug == aLipPucker )
	{
		MDataHandle otHandle = datablock.outputValue( aLipPucker ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_LipPucker) );
		datablock.setClean(aLipPucker);
	}
	else if(plug == aJawSlideRight )
	{
		MDataHandle otHandle = datablock.outputValue( aJawSlideRight ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_JawSlideRight) );
		datablock.setClean(aJawSlideRight);
	}
	else if(plug == aLipStretcherRight )
	{
		MDataHandle otHandle = datablock.outputValue( aLipStretcherRight ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_LipStretcherRight) );
		datablock.setClean(aLipStretcherRight);
	}
	else if(plug == aLipStretcherLeft )
	{
		MDataHandle otHandle = datablock.outputValue( aLipStretcherLeft ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_LipStretcherLeft) );
		datablock.setClean(aLipStretcherLeft);
	}
	else if(plug == aLipCornerPullerLeft )
	{
		MDataHandle otHandle = datablock.outputValue( aLipCornerPullerLeft ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_LipCornerPullerLeft) );
		datablock.setClean(aLipCornerPullerLeft);
	}
	else if(plug == aLipCornerPullerRight )
	{
		MDataHandle otHandle = datablock.outputValue( aLipCornerPullerRight ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_LipCornerPullerRight) );
		datablock.setClean(aLipCornerPullerRight);
	}
	else if(plug == aLipCornerDepressorLeft )
	{
		MDataHandle otHandle = datablock.outputValue( aLipCornerDepressorLeft ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_LipCornerDepressorLeft) );
		datablock.setClean(aLipCornerDepressorLeft);
	}
	else if(plug == aLipCornerDepressorRight )
	{
		MDataHandle otHandle = datablock.outputValue( aLipCornerDepressorRight ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_LipCornerDepressorRight) );
		datablock.setClean(aLipCornerDepressorRight);
	}
	else if(plug == aLeftcheekPuff )
	{
		MDataHandle otHandle = datablock.outputValue( aLeftcheekPuff ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_LeftcheekPuff) );
		datablock.setClean(aLeftcheekPuff);
	}
	else if(plug == aRightcheekPuff )
	{
		MDataHandle otHandle = datablock.outputValue( aRightcheekPuff ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_RightcheekPuff) );
		datablock.setClean(aRightcheekPuff);
	}
	else if(plug == aLefteyeClosed )
	{
		MDataHandle otHandle = datablock.outputValue( aLefteyeClosed ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_LefteyeClosed) );
		datablock.setClean(aLefteyeClosed);
	}
	else if(plug == aRighteyeClosed )
	{
		MDataHandle otHandle = datablock.outputValue( aRighteyeClosed ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_RighteyeClosed) );
		datablock.setClean(aRighteyeClosed);
	}
	else if(plug == aRighteyebrowLowerer )
	{
		MDataHandle otHandle = datablock.outputValue( aRighteyebrowLowerer ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_RighteyebrowLowerer) );
		datablock.setClean(aRighteyebrowLowerer);
	}
	else if(plug == aLefteyebrowLowerer )
	{
		MDataHandle otHandle = datablock.outputValue( aLefteyebrowLowerer ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_LefteyebrowLowerer) );
		datablock.setClean(aLefteyebrowLowerer);
	}
	else if(plug == aLowerlipDepressorLeft )
	{
		MDataHandle otHandle = datablock.outputValue( aLowerlipDepressorLeft ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_LowerlipDepressorLeft) );
		datablock.setClean(aLowerlipDepressorLeft);
	}
	else if(plug == aLowerlipDepressorRight )
	{
		MDataHandle otHandle = datablock.outputValue( aLowerlipDepressorRight ); 
		otHandle.set( pFaceShape->GetAU(FaceShapeAnimations::FaceShapeAnimations_LowerlipDepressorRight) );
		datablock.setClean(aLowerlipDepressorRight);
	}
	else if(plug == aRotate ||
		plug == aRotateX || 
		plug == aRotateY || 
		plug == aRotateZ )
	{
		/*double rotationXYZ[3];
		pFaceShape->GetRotationXYZ(&rotationXYZ[0], &rotationXYZ[1], &rotationXYZ[2]);
		MDataHandle otHandle = datablock.outputValue( aRotate ); 
		otHandle.set( rotationXYZ[0], rotationXYZ[1], rotationXYZ[2] );
		datablock.setClean(aRotate);*/
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


void* NuiMayaFacialModelDriver::creator()
//
//	Description:
//		this method exists to give Maya a way to create new objects
//      of this type. 
//
//	Return Value:
//		a new object of this type
//
{
	return new NuiMayaFacialModelDriver();
}

MStatus NuiMayaFacialModelDriver::initialize()
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
	aInputFacialData = typedAttr.create( "inputFacialData", "ifd",
		NuiMayaFacialModelData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create inputFacialData attribute"); return stat;}
	typedAttr.setReadable( false );
	typedAttr.setWritable( true );
	typedAttr.setStorable( false );
	stat = addAttribute( aInputFacialData );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aInputImageData = typedAttr.create( "inputImageData", "iid",
		NuiMayaImageData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create inputImageData attribute"); return stat;}
	typedAttr.setReadable( false );
	typedAttr.setWritable( true );
	typedAttr.setStorable( false );
	stat = addAttribute( aInputImageData );
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

	aJawOpen = nAttr.create( "jawOpen", "jo", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create jawOpen attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aJawOpen );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aLipPucker = nAttr.create( "lipPucker", "lp", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create lipPucker attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLipPucker );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aJawSlideRight = nAttr.create( "jawSlideRight", "jsr", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create jawSlideRight attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aJawSlideRight );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aLipStretcherRight = nAttr.create( "lipStretcherRight", "lsr", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create lipStretcherRight attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLipStretcherRight );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aLipStretcherLeft = nAttr.create( "lipStretcherLeft", "lsl", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create lipStretcherLeft attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLipStretcherLeft );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aLipCornerPullerLeft = nAttr.create( "lipCornerPullerLeft", "lpl", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create aLipCornerPullerLeft attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLipCornerPullerLeft );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aLipCornerPullerRight = nAttr.create( "lipCornerPullerRight", "lpr", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create aLipCornerPullerRight attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLipCornerPullerRight );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aLipCornerDepressorLeft = nAttr.create( "lipCornerDepressorLeft", "lcdl", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create aLipCornerDepressorLeft attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLipCornerDepressorLeft );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aLipCornerDepressorRight = nAttr.create( "lipCornerDepressorRight", "lcdr", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create aLipCornerDepressorRight attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLipCornerDepressorRight );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aLeftcheekPuff = nAttr.create( "leftcheekPuff", "lcp", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create aLeftcheekPuff attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftcheekPuff );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aRightcheekPuff = nAttr.create( "rightcheekPuff", "rcp", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create aRightcheekPuff attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightcheekPuff );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aLefteyeClosed = nAttr.create( "lefteyeClosed", "lec", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create aLefteyeClosed attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLefteyeClosed );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aRighteyeClosed = nAttr.create( "righteyeClosed", "rec", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create aRighteyeClosed attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRighteyeClosed );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aRighteyebrowLowerer = nAttr.create( "righteyebrowLowerer", "rbl", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create aRighteyebrowLowerer attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRighteyebrowLowerer );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aLefteyebrowLowerer = nAttr.create( "lefteyebrowLowerer", "lbl", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create aLefteyebrowLowerer attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLefteyebrowLowerer );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aLowerlipDepressorLeft = nAttr.create( "lowerlipDepressorLeft", "lldl", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create aLowerlipDepressorLeft attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLowerlipDepressorLeft );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aLowerlipDepressorRight = nAttr.create( "lowerlipDepressorRight", "lldr", MFnNumericData::kFloat, 0.0, &stat );
	if (!stat) { stat.perror("create aLowerlipDepressorRight attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLowerlipDepressorRight );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// Set up a dependency between the input and the output.  This will cause
	// the output to be marked dirty when the input changes.  The output will
	// then be recomputed the next time the value of the output is requested.
	//
	stat = attributeAffects( aInputFacialData, aRotateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aRotateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aRotateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aRotate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputFacialData, aJawOpen );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aLipPucker );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aJawSlideRight );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aLipStretcherRight );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aLipStretcherLeft );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aLipCornerPullerLeft );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aLipCornerPullerRight );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aLipCornerDepressorLeft );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aLipCornerDepressorRight );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aLeftcheekPuff );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aRightcheekPuff );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aLefteyeClosed );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aRighteyeClosed );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aRighteyebrowLowerer );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aLefteyebrowLowerer );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aLowerlipDepressorLeft );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputFacialData, aLowerlipDepressorRight );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	return MS::kSuccess;
}