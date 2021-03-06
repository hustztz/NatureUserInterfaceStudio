#pragma once

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MCallbackIdArray.h>
#include <maya/MNodeMessage.h>

class NuiMayaFacialModelDriver : public MPxNode
{
public:
	NuiMayaFacialModelDriver();
	virtual				~NuiMayaFacialModelDriver();
	// Override functions
	virtual void		postConstructor();
	virtual MStatus		compute( const MPlug& plug, MDataBlock& datablock );

	static  void*		creator();
	static  MStatus		initialize();

public:

	// There needs to be a MObject handle declared for each attribute that
	// the node will have.  These handles are needed for getting and setting
	// the values later.
	//
	static  MObject		aInputFacialData;
	static  MObject		aInputImageData;

	static  MObject		aRotateX;
	static  MObject		aRotateY;
	static  MObject		aRotateZ;
	static  MObject		aRotate;

	static  MObject		aEyeLeftTranslateX;
	static  MObject		aEyeLeftTranslateY;
	static  MObject		aEyeLeftTranslateZ;
	static  MObject		aEyeLeftTranslate;

	static  MObject		aEyeRightTranslateX;
	static  MObject		aEyeRightTranslateY;
	static  MObject		aEyeRightTranslateZ;
	static  MObject		aEyeRightTranslate;

	static  MObject		aNoseTranslateX;
	static  MObject		aNoseTranslateY;
	static  MObject		aNoseTranslateZ;
	static  MObject		aNoseTranslate;

	static  MObject		aMouthCornerLeftTranslateX;
	static  MObject		aMouthCornerLeftTranslateY;
	static  MObject		aMouthCornerLeftTranslateZ;
	static  MObject		aMouthCornerLeftTranslate;

	static  MObject		aMouthCornerRightTranslateX;
	static  MObject		aMouthCornerRightTranslateY;
	static  MObject		aMouthCornerRightTranslateZ;
	static  MObject		aMouthCornerRightTranslate;

	static  MObject		aJawOpen;
	static  MObject		aLipPucker;
	static  MObject		aJawSlideRight;
	static  MObject		aLipStretcherRight;
	static  MObject		aLipStretcherLeft;
	static  MObject		aLipCornerPullerLeft;
	static  MObject		aLipCornerPullerRight;
	static  MObject		aLipCornerDepressorLeft;
	static  MObject		aLipCornerDepressorRight;
	static  MObject		aLeftcheekPuff;
	static  MObject		aRightcheekPuff;
	static  MObject		aLefteyeClosed;
	static  MObject		aRighteyeClosed;
	static  MObject		aRighteyebrowLowerer;
	static  MObject		aLefteyebrowLowerer;
	static  MObject		aLowerlipDepressorLeft;
	static  MObject		aLowerlipDepressorRight;

	// The typeid is a unique 32bit indentifier that describes this node.
	// It is used to save and retrieve nodes of this type from the binary
	// file format.  If it is not unique, it will cause file IO problems.
	//
	static	MTypeId		id;
};