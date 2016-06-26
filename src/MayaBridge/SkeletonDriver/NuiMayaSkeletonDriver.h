#pragma once

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MCallbackIdArray.h>
#include <maya/MNodeMessage.h>

class NuiSkeletonJoints;

class NuiMayaSkeletonDriver : public MPxNode
{
	static const int GESTURE_NUM = 2;
public:
	NuiMayaSkeletonDriver();
	virtual				~NuiMayaSkeletonDriver();
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
	static  MObject		aInputSkeletonData;
	static  MObject		aInputGestureData;

	static  MObject		aGestureStates;

	static  MObject		aLeftHandState;
	static  MObject		aRightHandState;

	static  MObject		aHeadRelativeTranslate;
	static  MObject		aHeadRelativeTranslateX;
	static  MObject		aHeadRelativeTranslateY;
	static  MObject		aHeadRelativeTranslateZ;
	static  MObject		aHeadTranslate;
	static  MObject		aHeadTranslateX;
	static  MObject		aHeadTranslateY;
	static  MObject		aHeadTranslateZ;
	static  MObject		aHeadValid;
	static  MObject		aNeckRelativeTranslate;
	static  MObject		aNeckRelativeTranslateX;
	static  MObject		aNeckRelativeTranslateY;
	static  MObject		aNeckRelativeTranslateZ;
	static  MObject		aNeckValid;

	static  MObject		aLeftShoulderRelativeTranslate;
	static  MObject		aLeftShoulderRelativeTranslateX;
	static  MObject		aLeftShoulderRelativeTranslateY;
	static  MObject		aLeftShoulderRelativeTranslateZ;
	static  MObject		aLeftShoulderValid;
	static  MObject		aRightShoulderRelativeTranslate;
	static  MObject		aRightShoulderRelativeTranslateX;
	static  MObject		aRightShoulderRelativeTranslateY;
	static  MObject		aRightShoulderRelativeTranslateZ;
	static  MObject		aRightShoulderValid;
	static  MObject		aLeftElbowRelativeTranslate;
	static  MObject		aLeftElbowRelativeTranslateX;
	static  MObject		aLeftElbowRelativeTranslateY;
	static  MObject		aLeftElbowRelativeTranslateZ;
	static  MObject		aLeftElbowTranslate;
	static  MObject		aLeftElbowTranslateX;
	static  MObject		aLeftElbowTranslateY;
	static  MObject		aLeftElbowTranslateZ;
	static  MObject		aLeftElbowValid;
	static  MObject		aRightElbowRelativeTranslate;
	static  MObject		aRightElbowRelativeTranslateX;
	static  MObject		aRightElbowRelativeTranslateY;
	static  MObject		aRightElbowRelativeTranslateZ;
	static  MObject		aRightElbowTranslate;
	static  MObject		aRightElbowTranslateX;
	static  MObject		aRightElbowTranslateY;
	static  MObject		aRightElbowTranslateZ;
	static  MObject		aRightElbowValid;
	static  MObject		aLeftHandRelativeTranslate;
	static  MObject		aLeftHandRelativeTranslateX;
	static  MObject		aLeftHandRelativeTranslateY;
	static  MObject		aLeftHandRelativeTranslateZ;
	static  MObject		aLeftHandTranslate;
	static  MObject		aLeftHandTranslateX;
	static  MObject		aLeftHandTranslateY;
	static  MObject		aLeftHandTranslateZ;
	static  MObject		aLeftHandValid;
	static  MObject		aRightHandRelativeTranslate;
	static  MObject		aRightHandRelativeTranslateX;
	static  MObject		aRightHandRelativeTranslateY;
	static  MObject		aRightHandRelativeTranslateZ;
	static  MObject		aRightHandTranslate;
	static  MObject		aRightHandTranslateX;
	static  MObject		aRightHandTranslateY;
	static  MObject		aRightHandTranslateZ;
	static  MObject		aRightHandValid;

	static  MObject		aTorsoRelativeTranslate;
	static  MObject		aTorsoRelativeTranslateX;
	static  MObject		aTorsoRelativeTranslateY;
	static  MObject		aTorsoRelativeTranslateZ;
	static  MObject		aTorsoValid;

	static  MObject		aLeftHipRelativeTranslate;
	static  MObject		aLeftHipRelativeTranslateX;
	static  MObject		aLeftHipRelativeTranslateY;
	static  MObject		aLeftHipRelativeTranslateZ;
	static  MObject		aLeftHipValid;
	static  MObject		aRightHipRelativeTranslate;
	static  MObject		aRightHipRelativeTranslateX;
	static  MObject		aRightHipRelativeTranslateY;
	static  MObject		aRightHipRelativeTranslateZ;
	static  MObject		aRightHipValid;
	static  MObject		aLeftKneeRelativeTranslate;
	static  MObject		aLeftKneeRelativeTranslateX;
	static  MObject		aLeftKneeRelativeTranslateY;
	static  MObject		aLeftKneeRelativeTranslateZ;
	static  MObject		aLeftKneeValid;
	static  MObject		aRightKneeRelativeTranslate;
	static  MObject		aRightKneeRelativeTranslateX;
	static  MObject		aRightKneeRelativeTranslateY;
	static  MObject		aRightKneeRelativeTranslateZ;
	static  MObject		aRightKneeValid;
	static  MObject		aLeftFootRelativeTranslate;
	static  MObject		aLeftFootRelativeTranslateX;
	static  MObject		aLeftFootRelativeTranslateY;
	static  MObject		aLeftFootRelativeTranslateZ;
	static  MObject		aLeftFootValid;
	static  MObject		aRightFootRelativeTranslate;
	static  MObject		aRightFootRelativeTranslateX;
	static  MObject		aRightFootRelativeTranslateY;
	static  MObject		aRightFootRelativeTranslateZ;
	static  MObject		aRightFootValid;

	static  MObject		aCenterMassTranslate;
	static  MObject		aCenterMassTranslateX;
	static  MObject		aCenterMassTranslateY;
	static  MObject		aCenterMassTranslateZ;
	static  MObject		aCenterMassValid;

	// The typeid is a unique 32bit indentifier that describes this node.
	// It is used to save and retrieve nodes of this type from the binary
	// file format.  If it is not unique, it will cause file IO problems.
	//
	static	MTypeId		id;

protected:
	void		computeWholeSkeleton( MDataBlock& datablock, NuiSkeletonJoints* pSkeleton );
};