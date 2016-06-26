#pragma once

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MCallbackIdArray.h>
#include <maya/MNodeMessage.h>

class NuiMayaTrackedFaceDriver : public MPxNode
{
public:
	NuiMayaTrackedFaceDriver();
	virtual				~NuiMayaTrackedFaceDriver();
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
	static  MObject		aInputData;

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

	// The typeid is a unique 32bit indentifier that describes this node.
	// It is used to save and retrieve nodes of this type from the binary
	// file format.  If it is not unique, it will cause file IO problems.
	//
	static	MTypeId		id;
};