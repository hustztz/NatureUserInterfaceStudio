#include "NuiMayaSkeletonDriver.h"
#include "NuiMayaSkeletonData.h"
#include "NuiMayaGestureData.h"
#include "../api_macros.h"

#include "Shape/NuiSkeletonJoints.h"

#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnPluginData.h>
#include <maya/MGlobal.h>
#include <maya/MDistance.h>
#include <maya/MArrayDataBuilder.h>

MTypeId     NuiMayaSkeletonDriver::id( 0x08102B );

// Attributes
//
MObject		NuiMayaSkeletonDriver::aInputSkeletonData;
MObject		NuiMayaSkeletonDriver::aInputGestureData;

MObject		NuiMayaSkeletonDriver::aGestureStates;
MObject		NuiMayaSkeletonDriver::aLeftHandState;
MObject		NuiMayaSkeletonDriver::aRightHandState;

MObject		NuiMayaSkeletonDriver::aHeadRelativeTranslate;
MObject		NuiMayaSkeletonDriver::aHeadRelativeTranslateX;
MObject		NuiMayaSkeletonDriver::aHeadRelativeTranslateY;
MObject		NuiMayaSkeletonDriver::aHeadRelativeTranslateZ;
MObject		NuiMayaSkeletonDriver::aHeadTranslate;
MObject		NuiMayaSkeletonDriver::aHeadTranslateX;
MObject		NuiMayaSkeletonDriver::aHeadTranslateY;
MObject		NuiMayaSkeletonDriver::aHeadTranslateZ;
MObject		NuiMayaSkeletonDriver::aHeadValid;
MObject		NuiMayaSkeletonDriver::aNeckRelativeTranslate;
MObject		NuiMayaSkeletonDriver::aNeckRelativeTranslateX;
MObject		NuiMayaSkeletonDriver::aNeckRelativeTranslateY;
MObject		NuiMayaSkeletonDriver::aNeckRelativeTranslateZ;
MObject		NuiMayaSkeletonDriver::aNeckValid;

MObject		NuiMayaSkeletonDriver::aLeftShoulderRelativeTranslate;
MObject		NuiMayaSkeletonDriver::aLeftShoulderRelativeTranslateX;
MObject		NuiMayaSkeletonDriver::aLeftShoulderRelativeTranslateY;
MObject		NuiMayaSkeletonDriver::aLeftShoulderRelativeTranslateZ;
MObject		NuiMayaSkeletonDriver::aLeftShoulderValid;
MObject		NuiMayaSkeletonDriver::aRightShoulderRelativeTranslate;
MObject		NuiMayaSkeletonDriver::aRightShoulderRelativeTranslateX;
MObject		NuiMayaSkeletonDriver::aRightShoulderRelativeTranslateY;
MObject		NuiMayaSkeletonDriver::aRightShoulderRelativeTranslateZ;
MObject		NuiMayaSkeletonDriver::aRightShoulderValid;
MObject		NuiMayaSkeletonDriver::aLeftElbowRelativeTranslate;
MObject		NuiMayaSkeletonDriver::aLeftElbowRelativeTranslateX;
MObject		NuiMayaSkeletonDriver::aLeftElbowRelativeTranslateY;
MObject		NuiMayaSkeletonDriver::aLeftElbowRelativeTranslateZ;
MObject		NuiMayaSkeletonDriver::aLeftElbowTranslate;
MObject		NuiMayaSkeletonDriver::aLeftElbowTranslateX;
MObject		NuiMayaSkeletonDriver::aLeftElbowTranslateY;
MObject		NuiMayaSkeletonDriver::aLeftElbowTranslateZ;
MObject		NuiMayaSkeletonDriver::aLeftElbowValid;
MObject		NuiMayaSkeletonDriver::aRightElbowRelativeTranslate;
MObject		NuiMayaSkeletonDriver::aRightElbowRelativeTranslateX;
MObject		NuiMayaSkeletonDriver::aRightElbowRelativeTranslateY;
MObject		NuiMayaSkeletonDriver::aRightElbowRelativeTranslateZ;
MObject		NuiMayaSkeletonDriver::aRightElbowTranslate;
MObject		NuiMayaSkeletonDriver::aRightElbowTranslateX;
MObject		NuiMayaSkeletonDriver::aRightElbowTranslateY;
MObject		NuiMayaSkeletonDriver::aRightElbowTranslateZ;
MObject		NuiMayaSkeletonDriver::aRightElbowValid;
MObject		NuiMayaSkeletonDriver::aLeftHandRelativeTranslate;
MObject		NuiMayaSkeletonDriver::aLeftHandRelativeTranslateX;
MObject		NuiMayaSkeletonDriver::aLeftHandRelativeTranslateY;
MObject		NuiMayaSkeletonDriver::aLeftHandRelativeTranslateZ;
MObject		NuiMayaSkeletonDriver::aLeftHandTranslate;
MObject		NuiMayaSkeletonDriver::aLeftHandTranslateX;
MObject		NuiMayaSkeletonDriver::aLeftHandTranslateY;
MObject		NuiMayaSkeletonDriver::aLeftHandTranslateZ;
MObject		NuiMayaSkeletonDriver::aLeftHandValid;
MObject		NuiMayaSkeletonDriver::aRightHandRelativeTranslate;
MObject		NuiMayaSkeletonDriver::aRightHandRelativeTranslateX;
MObject		NuiMayaSkeletonDriver::aRightHandRelativeTranslateY;
MObject		NuiMayaSkeletonDriver::aRightHandRelativeTranslateZ;
MObject		NuiMayaSkeletonDriver::aRightHandTranslate;
MObject		NuiMayaSkeletonDriver::aRightHandTranslateX;
MObject		NuiMayaSkeletonDriver::aRightHandTranslateY;
MObject		NuiMayaSkeletonDriver::aRightHandTranslateZ;
MObject		NuiMayaSkeletonDriver::aRightHandValid;

MObject		NuiMayaSkeletonDriver::aTorsoRelativeTranslate;
MObject		NuiMayaSkeletonDriver::aTorsoRelativeTranslateX;
MObject		NuiMayaSkeletonDriver::aTorsoRelativeTranslateY;
MObject		NuiMayaSkeletonDriver::aTorsoRelativeTranslateZ;
MObject		NuiMayaSkeletonDriver::aTorsoValid;

MObject		NuiMayaSkeletonDriver::aLeftHipRelativeTranslate;
MObject		NuiMayaSkeletonDriver::aLeftHipRelativeTranslateX;
MObject		NuiMayaSkeletonDriver::aLeftHipRelativeTranslateY;
MObject		NuiMayaSkeletonDriver::aLeftHipRelativeTranslateZ;
MObject		NuiMayaSkeletonDriver::aLeftHipValid;
MObject		NuiMayaSkeletonDriver::aRightHipRelativeTranslate;
MObject		NuiMayaSkeletonDriver::aRightHipRelativeTranslateX;
MObject		NuiMayaSkeletonDriver::aRightHipRelativeTranslateY;
MObject		NuiMayaSkeletonDriver::aRightHipRelativeTranslateZ;
MObject		NuiMayaSkeletonDriver::aRightHipValid;
MObject		NuiMayaSkeletonDriver::aLeftKneeRelativeTranslate;
MObject		NuiMayaSkeletonDriver::aLeftKneeRelativeTranslateX;
MObject		NuiMayaSkeletonDriver::aLeftKneeRelativeTranslateY;
MObject		NuiMayaSkeletonDriver::aLeftKneeRelativeTranslateZ;
MObject		NuiMayaSkeletonDriver::aLeftKneeValid;
MObject		NuiMayaSkeletonDriver::aRightKneeRelativeTranslate;
MObject		NuiMayaSkeletonDriver::aRightKneeRelativeTranslateX;
MObject		NuiMayaSkeletonDriver::aRightKneeRelativeTranslateY;
MObject		NuiMayaSkeletonDriver::aRightKneeRelativeTranslateZ;
MObject		NuiMayaSkeletonDriver::aRightKneeValid;
MObject		NuiMayaSkeletonDriver::aLeftFootRelativeTranslate;
MObject		NuiMayaSkeletonDriver::aLeftFootRelativeTranslateX;
MObject		NuiMayaSkeletonDriver::aLeftFootRelativeTranslateY;
MObject		NuiMayaSkeletonDriver::aLeftFootRelativeTranslateZ;
MObject		NuiMayaSkeletonDriver::aLeftFootValid;
MObject		NuiMayaSkeletonDriver::aRightFootRelativeTranslate;
MObject		NuiMayaSkeletonDriver::aRightFootRelativeTranslateX;
MObject		NuiMayaSkeletonDriver::aRightFootRelativeTranslateY;
MObject		NuiMayaSkeletonDriver::aRightFootRelativeTranslateZ;
MObject		NuiMayaSkeletonDriver::aRightFootValid;

MObject		NuiMayaSkeletonDriver::aCenterMassTranslate;
MObject		NuiMayaSkeletonDriver::aCenterMassTranslateX;
MObject		NuiMayaSkeletonDriver::aCenterMassTranslateY;
MObject		NuiMayaSkeletonDriver::aCenterMassTranslateZ;
MObject		NuiMayaSkeletonDriver::aCenterMassValid;

NuiMayaSkeletonDriver::NuiMayaSkeletonDriver()
{
}
NuiMayaSkeletonDriver::~NuiMayaSkeletonDriver()
{
}

void NuiMayaSkeletonDriver::postConstructor()
{

}


void* NuiMayaSkeletonDriver::creator()
//
//	Description:
//		this method exists to give Maya a way to create new objects
//      of this type. 
//
//	Return Value:
//		a new object of this type
//
{
	return new NuiMayaSkeletonDriver();
}

void	NuiMayaSkeletonDriver::computeWholeSkeleton( MDataBlock& datablock, NuiSkeletonJoints* pSkeleton )
{
	double distanceUnit = 1.0;
	MDistance::Unit currentUnit = MDistance::internalUnit();
	if(currentUnit != MDistance::kInvalid)
	{
		distanceUnit = MDistance(1.0, MDistance::kMeters).as(currentUnit);
	}
 
	double retiaveX = 0.0;
	double retiaveY = 0.0;
	double retiaveZ = 0.0;

	/*if ( plug == aCenterMassValid ||
		plug == aCenterMassTranslate )*/
	{
		// Center
		MDataHandle otHandle = datablock.outputValue( aCenterMassTranslate );
		otHandle.set( distanceUnit * pSkeleton->GetJoint3DPosX(JOINT_TYPE_HIP_CENTER),
			distanceUnit * pSkeleton->GetJoint3DPosY(JOINT_TYPE_HIP_CENTER),
			distanceUnit * pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HIP_CENTER) );
		datablock.setClean(aCenterMassTranslate);

		otHandle = datablock.outputValue( aCenterMassValid ); 
		otHandle.set( !pSkeleton->IsInferred(JOINT_TYPE_HIP_CENTER) );
		datablock.setClean(aCenterMassValid);
	}
	/*else if ( plug == aTorsoValid ||
		plug == aTorsoTranslate )*/
	{
		// Spine
		MDataHandle otHandle = datablock.outputValue( aTorsoValid );
		if(!pSkeleton->IsInferred(JOINT_TYPE_SPINE))
		{
			retiaveX = pSkeleton->GetJoint3DPosX(JOINT_TYPE_SPINE) - pSkeleton->GetJoint3DPosX(JOINT_TYPE_HIP_CENTER);
			retiaveY = pSkeleton->GetJoint3DPosY(JOINT_TYPE_SPINE) - pSkeleton->GetJoint3DPosY(JOINT_TYPE_HIP_CENTER);
			retiaveZ = pSkeleton->GetJoint3DPosZ(JOINT_TYPE_SPINE) - pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HIP_CENTER);
			otHandle.set( true );
		}
		else
			otHandle.set( false );
		datablock.setClean(aTorsoValid);

		otHandle = datablock.outputValue( aTorsoRelativeTranslate ); 
		otHandle.set( distanceUnit * retiaveX, distanceUnit * retiaveY, distanceUnit * retiaveZ );
		datablock.setClean(aTorsoRelativeTranslate);
	}
	/*else if ( plug == aNeckValid ||
		plug == aNeckTranslate )*/
	{
		// Neck
		MDataHandle otHandle = datablock.outputValue( aNeckValid );
		if(!pSkeleton->IsInferred(JOINT_TYPE_SHOULDER_CENTER))
		{
			retiaveX = pSkeleton->GetJoint3DPosX(JOINT_TYPE_SHOULDER_CENTER) - pSkeleton->GetJoint3DPosX(JOINT_TYPE_SPINE);
			retiaveY = pSkeleton->GetJoint3DPosY(JOINT_TYPE_SHOULDER_CENTER) - pSkeleton->GetJoint3DPosY(JOINT_TYPE_SPINE);
			retiaveZ = pSkeleton->GetJoint3DPosZ(JOINT_TYPE_SHOULDER_CENTER) - pSkeleton->GetJoint3DPosZ(JOINT_TYPE_SPINE);
			otHandle.set( true );
		}
		else
			otHandle.set( false );
		datablock.setClean(aNeckValid);

		otHandle = datablock.outputValue( aNeckRelativeTranslate ); 
		otHandle.set( distanceUnit * retiaveX, distanceUnit * retiaveY, distanceUnit * retiaveZ );
		datablock.setClean(aNeckRelativeTranslate);
	}
	/*else if ( plug == aHeadValid ||
		plug == aHeadTranslate )*/
	{
		// Head
		MDataHandle otHandle = datablock.outputValue( aHeadValid );
		if(!pSkeleton->IsInferred(JOINT_TYPE_HEAD))
		{
			retiaveX = pSkeleton->GetJoint3DPosX(JOINT_TYPE_HEAD) - pSkeleton->GetJoint3DPosX(JOINT_TYPE_SHOULDER_CENTER);
			retiaveY = pSkeleton->GetJoint3DPosY(JOINT_TYPE_HEAD) - pSkeleton->GetJoint3DPosY(JOINT_TYPE_SHOULDER_CENTER);
			retiaveZ = pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HEAD) - pSkeleton->GetJoint3DPosZ(JOINT_TYPE_SHOULDER_CENTER);
			otHandle.set( true );
		}
		else
			otHandle.set( false );
		datablock.setClean(aHeadValid);

		otHandle = datablock.outputValue( aHeadRelativeTranslate ); 
		otHandle.set( distanceUnit * retiaveX, distanceUnit * retiaveY, distanceUnit * retiaveZ );
		datablock.setClean(aHeadRelativeTranslate);
	}
	/*else if ( plug == aLeftShoulderValid ||
		plug == aLeftShoulderTranslate )*/
	{
		// Left shoulder
		MDataHandle otHandle = datablock.outputValue( aLeftShoulderValid );
		if(!pSkeleton->IsInferred(JOINT_TYPE_SHOULDER_LEFT))
		{
			retiaveX = pSkeleton->GetJoint3DPosX(JOINT_TYPE_SHOULDER_LEFT) - pSkeleton->GetJoint3DPosX(JOINT_TYPE_SHOULDER_CENTER);
			retiaveY = pSkeleton->GetJoint3DPosY(JOINT_TYPE_SHOULDER_LEFT) - pSkeleton->GetJoint3DPosY(JOINT_TYPE_SHOULDER_CENTER);
			retiaveZ = pSkeleton->GetJoint3DPosZ(JOINT_TYPE_SHOULDER_LEFT) - pSkeleton->GetJoint3DPosZ(JOINT_TYPE_SHOULDER_CENTER);
			otHandle.set( true );
		}
		else
		{
			retiaveX = retiaveY = retiaveZ = 0.0;
			otHandle.set( false );
		}
		datablock.setClean(aLeftShoulderValid);

		otHandle = datablock.outputValue( aLeftShoulderRelativeTranslate ); 
		otHandle.set( distanceUnit * retiaveX, distanceUnit * retiaveY, distanceUnit * retiaveZ );
		datablock.setClean(aLeftShoulderRelativeTranslate);
	}
	/*else if ( plug == aLeftElbowValid ||
		plug == aLeftElbowTranslate )*/
	{
		// Left elbow
		MDataHandle otHandle = datablock.outputValue( aLeftElbowValid );
		if(!pSkeleton->IsInferred(JOINT_TYPE_ELBOW_LEFT))
		{
			retiaveX = pSkeleton->GetJoint3DPosX(JOINT_TYPE_ELBOW_LEFT) - pSkeleton->GetJoint3DPosX(JOINT_TYPE_SHOULDER_LEFT);
			retiaveY = pSkeleton->GetJoint3DPosY(JOINT_TYPE_ELBOW_LEFT) - pSkeleton->GetJoint3DPosY(JOINT_TYPE_SHOULDER_LEFT);
			retiaveZ = pSkeleton->GetJoint3DPosZ(JOINT_TYPE_ELBOW_LEFT) - pSkeleton->GetJoint3DPosZ(JOINT_TYPE_SHOULDER_LEFT);
			otHandle.set( true );
		}
		else
			otHandle.set( false );
		datablock.setClean(aLeftElbowValid);

		otHandle = datablock.outputValue( aLeftElbowRelativeTranslate ); 
		otHandle.set( distanceUnit * retiaveX, distanceUnit * retiaveY, distanceUnit * retiaveZ );
		datablock.setClean(aLeftElbowRelativeTranslate);
	}
	/*else if ( plug == aLeftHandValid ||
		plug == aLeftHandTranslate )*/
	{
		// Left hand
		MDataHandle otHandle = datablock.outputValue( aLeftHandValid );
		if(!pSkeleton->IsInferred(JOINT_TYPE_HAND_LEFT))
		{
			retiaveX = pSkeleton->GetJoint3DPosX(JOINT_TYPE_HAND_LEFT) - pSkeleton->GetJoint3DPosX(JOINT_TYPE_ELBOW_LEFT);
			retiaveY = pSkeleton->GetJoint3DPosY(JOINT_TYPE_HAND_LEFT) - pSkeleton->GetJoint3DPosY(JOINT_TYPE_ELBOW_LEFT);
			retiaveZ = pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HAND_LEFT) - pSkeleton->GetJoint3DPosZ(JOINT_TYPE_ELBOW_LEFT);
			otHandle.set( true );
		}
		else
			otHandle.set( false );
		datablock.setClean(aLeftHandValid);

		otHandle = datablock.outputValue( aLeftHandRelativeTranslate ); 
		otHandle.set( distanceUnit * retiaveX, distanceUnit * retiaveY, distanceUnit * retiaveZ );
		datablock.setClean(aLeftHandRelativeTranslate);
	}
	/*else if ( plug == aRightShoulderValid ||
		plug == aRightShoulderTranslate )*/
	{
		// Right shoulder
		MDataHandle otHandle = datablock.outputValue( aRightShoulderValid );
		if(!pSkeleton->IsInferred(JOINT_TYPE_SHOULDER_RIGHT))
		{
			retiaveX = pSkeleton->GetJoint3DPosX(JOINT_TYPE_SHOULDER_RIGHT) - pSkeleton->GetJoint3DPosX(JOINT_TYPE_SHOULDER_CENTER);
			retiaveY = pSkeleton->GetJoint3DPosY(JOINT_TYPE_SHOULDER_RIGHT) - pSkeleton->GetJoint3DPosY(JOINT_TYPE_SHOULDER_CENTER);
			retiaveZ = pSkeleton->GetJoint3DPosZ(JOINT_TYPE_SHOULDER_RIGHT) - pSkeleton->GetJoint3DPosZ(JOINT_TYPE_SHOULDER_CENTER);
			otHandle.set( true );
		}
		else
		{
			retiaveX = retiaveY = retiaveZ = 0.0;
			otHandle.set( false );
		}
		datablock.setClean(aRightShoulderValid);

		otHandle = datablock.outputValue( aRightShoulderRelativeTranslate ); 
		otHandle.set( distanceUnit * retiaveX, distanceUnit * retiaveY, distanceUnit * retiaveZ );
		datablock.setClean(aRightShoulderRelativeTranslate);
	}
	/*else if ( plug == aRightElbowValid ||
		plug == aRightElbowTranslate )*/
	{
		// Right elbow
		MDataHandle otHandle = datablock.outputValue( aRightElbowValid );
		if(!pSkeleton->IsInferred(JOINT_TYPE_ELBOW_RIGHT))
		{
			retiaveX = pSkeleton->GetJoint3DPosX(JOINT_TYPE_ELBOW_RIGHT) - pSkeleton->GetJoint3DPosX(JOINT_TYPE_SHOULDER_RIGHT);
			retiaveY = pSkeleton->GetJoint3DPosY(JOINT_TYPE_ELBOW_RIGHT) - pSkeleton->GetJoint3DPosY(JOINT_TYPE_SHOULDER_RIGHT);
			retiaveZ = pSkeleton->GetJoint3DPosZ(JOINT_TYPE_ELBOW_RIGHT) - pSkeleton->GetJoint3DPosZ(JOINT_TYPE_SHOULDER_RIGHT);
			otHandle.set( true );
		}
		else
			otHandle.set( false );
		datablock.setClean(aRightElbowValid);

		otHandle = datablock.outputValue( aRightElbowRelativeTranslate ); 
		otHandle.set( distanceUnit * retiaveX, distanceUnit * retiaveY, distanceUnit * retiaveZ );
		datablock.setClean(aRightElbowRelativeTranslate);
	}
	/*else if ( plug == aRightHandValid ||
		plug == aRightHandTranslate )*/
	{
		// Right hand
		MDataHandle otHandle = datablock.outputValue( aRightHandValid );
		if(!pSkeleton->IsInferred(JOINT_TYPE_HAND_RIGHT))
		{
			retiaveX = pSkeleton->GetJoint3DPosX(JOINT_TYPE_HAND_RIGHT) - pSkeleton->GetJoint3DPosX(JOINT_TYPE_ELBOW_RIGHT);
			retiaveY = pSkeleton->GetJoint3DPosY(JOINT_TYPE_HAND_RIGHT) - pSkeleton->GetJoint3DPosY(JOINT_TYPE_ELBOW_RIGHT);
			retiaveZ = pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HAND_RIGHT) - pSkeleton->GetJoint3DPosZ(JOINT_TYPE_ELBOW_RIGHT);
			otHandle.set( true );
		}
		else
			otHandle.set( false );
		datablock.setClean(aRightHandValid);

		otHandle = datablock.outputValue( aRightHandRelativeTranslate ); 
		otHandle.set( distanceUnit * retiaveX, distanceUnit * retiaveY, distanceUnit * retiaveZ );
		datablock.setClean(aRightHandRelativeTranslate);
	}
	/*else if ( plug == aLeftHipValid ||
		plug == aLeftHipTranslate )*/
	{
		// Left hip
		MDataHandle otHandle = datablock.outputValue( aLeftHipValid );
		if(!pSkeleton->IsInferred(JOINT_TYPE_HIP_LEFT))
		{
			retiaveX = pSkeleton->GetJoint3DPosX(JOINT_TYPE_HIP_LEFT) - pSkeleton->GetJoint3DPosX(JOINT_TYPE_HIP_CENTER);
			retiaveY = pSkeleton->GetJoint3DPosY(JOINT_TYPE_HIP_LEFT) - pSkeleton->GetJoint3DPosY(JOINT_TYPE_HIP_CENTER);
			retiaveZ = pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HIP_LEFT) - pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HIP_CENTER);
			otHandle.set( true );
		}
		else
		{
			retiaveX = retiaveY = retiaveZ = 0.0;
			otHandle.set( false );
		}
		datablock.setClean(aLeftHipValid);

		otHandle = datablock.outputValue( aLeftHipRelativeTranslate ); 
		otHandle.set( distanceUnit * retiaveX, distanceUnit * retiaveY, distanceUnit * retiaveZ );
		datablock.setClean(aLeftHipRelativeTranslate);
	}
	/*else if ( plug == aLeftKneeValid ||
		plug == aLeftKneeTranslate )*/
	{
		// Left knee
		MDataHandle otHandle = datablock.outputValue( aLeftKneeValid );
		if(!pSkeleton->IsInferred(JOINT_TYPE_KNEE_LEFT))
		{
			retiaveX = pSkeleton->GetJoint3DPosX(JOINT_TYPE_KNEE_LEFT) - pSkeleton->GetJoint3DPosX(JOINT_TYPE_HIP_LEFT);
			retiaveY = pSkeleton->GetJoint3DPosY(JOINT_TYPE_KNEE_LEFT) - pSkeleton->GetJoint3DPosY(JOINT_TYPE_HIP_LEFT);
			retiaveZ = pSkeleton->GetJoint3DPosZ(JOINT_TYPE_KNEE_LEFT) - pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HIP_LEFT);
			otHandle.set( true );
		}
		else
			otHandle.set( false );
		datablock.setClean(aLeftKneeValid);

		otHandle = datablock.outputValue( aLeftKneeRelativeTranslate ); 
		otHandle.set( distanceUnit * retiaveX, distanceUnit * retiaveY, distanceUnit * retiaveZ );
		datablock.setClean(aLeftKneeRelativeTranslate);
	}
	/*else if ( plug == aLeftFootValid ||
		plug == aLeftFootTranslate )*/
	{
		// Left foot
		MDataHandle otHandle = datablock.outputValue( aLeftFootValid );
		if(!pSkeleton->IsInferred(JOINT_TYPE_FOOT_LEFT))
		{
			retiaveX = pSkeleton->GetJoint3DPosX(JOINT_TYPE_FOOT_LEFT) - pSkeleton->GetJoint3DPosX(JOINT_TYPE_KNEE_LEFT);
			retiaveY = pSkeleton->GetJoint3DPosY(JOINT_TYPE_FOOT_LEFT) - pSkeleton->GetJoint3DPosY(JOINT_TYPE_KNEE_LEFT);
			retiaveZ = pSkeleton->GetJoint3DPosZ(JOINT_TYPE_FOOT_LEFT) - pSkeleton->GetJoint3DPosZ(JOINT_TYPE_KNEE_LEFT);
			otHandle.set( true );
		}
		else
			otHandle.set( false );
		datablock.setClean(aLeftFootValid);

		otHandle = datablock.outputValue( aLeftFootRelativeTranslate ); 
		otHandle.set( distanceUnit * retiaveX, distanceUnit * retiaveY, distanceUnit * retiaveZ );
		datablock.setClean(aLeftFootRelativeTranslate);
	}
	/*else if ( plug == aRightHipValid ||
		plug == aRightHipTranslate )*/
	{
		// Right hip
		MDataHandle otHandle = datablock.outputValue( aRightHipValid );
		if(!pSkeleton->IsInferred(JOINT_TYPE_HIP_RIGHT))
		{
			retiaveX = pSkeleton->GetJoint3DPosX(JOINT_TYPE_HIP_RIGHT) - pSkeleton->GetJoint3DPosX(JOINT_TYPE_HIP_CENTER);
			retiaveY = pSkeleton->GetJoint3DPosY(JOINT_TYPE_HIP_RIGHT) - pSkeleton->GetJoint3DPosY(JOINT_TYPE_HIP_CENTER);
			retiaveZ = pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HIP_RIGHT) - pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HIP_CENTER);
			otHandle.set( true );
		}
		else
		{
			retiaveX = retiaveY = retiaveZ = 0.0;
			otHandle.set( false );
		}
		datablock.setClean(aRightHipValid);

		otHandle = datablock.outputValue( aRightHipRelativeTranslate ); 
		otHandle.set( distanceUnit * retiaveX, distanceUnit * retiaveY, distanceUnit * retiaveZ );
		datablock.setClean(aRightHipRelativeTranslate);
	}
	/*else if ( plug == aRightKneeValid ||
		plug == aRightKneeTranslate )*/
	{
		// Right knee
		MDataHandle otHandle = datablock.outputValue( aRightKneeValid );
		if(!pSkeleton->IsInferred(JOINT_TYPE_KNEE_RIGHT))
		{
			retiaveX = pSkeleton->GetJoint3DPosX(JOINT_TYPE_KNEE_RIGHT) - pSkeleton->GetJoint3DPosX(JOINT_TYPE_HIP_RIGHT);
			retiaveY = pSkeleton->GetJoint3DPosY(JOINT_TYPE_KNEE_RIGHT) - pSkeleton->GetJoint3DPosY(JOINT_TYPE_HIP_RIGHT);
			retiaveZ = pSkeleton->GetJoint3DPosZ(JOINT_TYPE_KNEE_RIGHT) - pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HIP_RIGHT);
			otHandle.set( true );
		}
		else
			otHandle.set( false );
		datablock.setClean(aRightKneeValid);

		otHandle = datablock.outputValue( aRightKneeRelativeTranslate ); 
		otHandle.set( distanceUnit * retiaveX, distanceUnit * retiaveY, distanceUnit * retiaveZ );
		datablock.setClean(aRightKneeRelativeTranslate);
	}
	/*else if ( plug == aRightFootValid ||
		plug == aRightFootTranslate )*/
	{
		// Right foot
		MDataHandle otHandle = datablock.outputValue( aRightFootValid );
		if(!pSkeleton->IsInferred(JOINT_TYPE_FOOT_RIGHT))
		{
			retiaveX = pSkeleton->GetJoint3DPosX(JOINT_TYPE_FOOT_RIGHT) - pSkeleton->GetJoint3DPosX(JOINT_TYPE_KNEE_RIGHT);
			retiaveY = pSkeleton->GetJoint3DPosY(JOINT_TYPE_FOOT_RIGHT) - pSkeleton->GetJoint3DPosY(JOINT_TYPE_KNEE_RIGHT);
			retiaveZ = pSkeleton->GetJoint3DPosZ(JOINT_TYPE_FOOT_RIGHT) - pSkeleton->GetJoint3DPosZ(JOINT_TYPE_KNEE_RIGHT);
			otHandle.set( true );
		}
		else
			otHandle.set( false );
		datablock.setClean(aRightFootValid);

		otHandle = datablock.outputValue( aRightFootRelativeTranslate );
		otHandle.set( distanceUnit * retiaveX, distanceUnit * retiaveY, distanceUnit * retiaveZ );
		datablock.setClean(aRightFootRelativeTranslate);
	}
}

MStatus NuiMayaSkeletonDriver::compute( const MPlug& plug, MDataBlock& datablock )
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

	if( plug == aGestureStates || plug.parent() == aGestureStates )
	{
		/* Get skeleton data */
		MDataHandle gestureHandle = datablock.inputValue( aInputGestureData, &returnStatus );
		if (!returnStatus) {
			returnStatus.perror("Error getting gesture data handle\n");
			return returnStatus;
		}
		NuiMayaGestureData* pData = (NuiMayaGestureData*)gestureHandle.asPluginData();
		if ( NULL == pData ) {
			MGlobal::displayError( "NULL gesture data found.");
			return MS::kFailure;
		}
		NuiGestureResult* pGesture = pData->m_pGestureResult;
		if ( NULL == pGesture ) {
			MGlobal::displayError( "NULL gesture shape found");
			return MS::kFailure;
		}

		MArrayDataHandle outputArray = datablock.outputValue( aGestureStates, &returnStatus );
		MCHECKERROR(returnStatus, "Error in block.aGestureStates.")

		MArrayDataBuilder otBuilder(aGestureStates, GESTURE_NUM, &returnStatus);
		MCHECKERROR(returnStatus, "Error in block builder.aGestureStates.")

		MDataHandle otHandle = otBuilder.addElement(0, &returnStatus);
		MCHECKERROR(returnStatus, "Error in add first gesture State.")
		otHandle.set(false);
		NuiDiscreteGestureResult gestureDiscreteResult = pGesture->GetDiscreteGestureResult(Gesture_Discrete_HandUp);
		if(gestureDiscreteResult.bDetected && gestureDiscreteResult.fConfidence > 0.0f)
		{
			MGlobal::displayInfo( " Gesture: Hand Up." );
			
			NuiContinuousGestureResult gestureContinuousResult = pGesture->GetContinuousGestureResult(Gesture_Continuous_Swipe);
			if(gestureContinuousResult.fProgress > 0.5f)
			{
				MGlobal::displayInfo( " Gesture: Hand Swipe." );
				otHandle.set(true);
			}
		}
		otHandle = otBuilder.addElement(1, &returnStatus);
		MCHECKERROR(returnStatus, "Error in add first gesture State.")
		gestureDiscreteResult = pGesture->GetDiscreteGestureResult(Gesture_Discrete_OpenHand);
		if(gestureDiscreteResult.bDetected && gestureDiscreteResult.fConfidence > 0.0f)
		{
			MGlobal::displayInfo( " Gesture: Open Hand." );
			otHandle.set(true);
		}
		else
		{
			otHandle.set(false);
		}

		returnStatus = outputArray.set(otBuilder);
		MCHECKERROR(returnStatus, "set gesture Builder failed\n");
		returnStatus = outputArray.setAllClean();
		MCHECKERROR(returnStatus, "set gesture Array failed\n");
	}
	else
	{
		/* Get skeleton data */
		MDataHandle skeletonHandle = datablock.inputValue( aInputSkeletonData, &returnStatus );
		if (!returnStatus) {
			returnStatus.perror("Error getting skeleton data handle\n");
			return returnStatus;
		}
		NuiMayaSkeletonData* pData = (NuiMayaSkeletonData*)skeletonHandle.asPluginData();
		if ( NULL == pData ) {
			MGlobal::displayError( "NULL skeleton data found.");
			return MS::kFailure;
		}
		NuiSkeletonJoints* pSkeleton = pData->m_pSkeletonData.get();
		if ( NULL == pSkeleton ) {
			MGlobal::displayError( "NULL skeleton shape found");
			return MS::kFailure;
		}

		if(plug == aLeftHandState)
		{
			MDataHandle otHandle = datablock.outputValue( aLeftHandState );
			HandState state = pSkeleton->GetLeftHand();
			if(HandState_Open == state)
				otHandle.set( true );
			else
				otHandle.set( false );
			datablock.setClean(plug);
		}
		else if(plug == aRightHandState)
		{
			MDataHandle otHandle = datablock.outputValue( aRightHandState );
			HandState state = pSkeleton->GetRightHand();
			if(HandState_Open == state)
				otHandle.set( true );
			else
				otHandle.set( false );
			datablock.setClean(plug);
		}
		else if(plug == aHeadValid)
		{
			MDataHandle otHandle = datablock.outputValue( aHeadValid );
			otHandle.set( !pSkeleton->IsInferred(JOINT_TYPE_HEAD) );
			datablock.setClean(plug);
		}
		else if(plug == aHeadTranslate ||
			plug == aHeadTranslateX ||
			plug == aHeadTranslateY ||
			plug == aHeadTranslateZ)
		{
			MDataHandle otHandle = datablock.outputValue( aHeadTranslate );
			if(!pSkeleton->IsInferred(JOINT_TYPE_HEAD))
			{
				double distanceUnit = 1.0;
				MDistance::Unit currentUnit = MDistance::internalUnit();
				if(currentUnit != MDistance::kInvalid)
				{
					distanceUnit = MDistance(1.0, MDistance::kMeters).as(currentUnit);
				}
				otHandle.set( distanceUnit * pSkeleton->GetJoint3DPosX(JOINT_TYPE_HEAD),
					distanceUnit * pSkeleton->GetJoint3DPosY(JOINT_TYPE_HEAD),
					distanceUnit * pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HEAD) );
			}
			else
			{
				otHandle.set( 0.0, 0.0, 0.0 );
			}
			datablock.setClean(plug);
		}
		else if(plug == aLeftHandValid)
		{
			MDataHandle otHandle = datablock.outputValue( aLeftHandValid );
			otHandle.set( !pSkeleton->IsInferred(JOINT_TYPE_HAND_LEFT) );
			datablock.setClean(plug);
		}
		else if(plug == aLeftHandTranslate ||
			plug == aLeftHandTranslateX ||
			plug == aLeftHandTranslateY ||
			plug == aLeftHandTranslateZ)
		{
			MDataHandle otHandle = datablock.outputValue( aLeftHandTranslate );
			if(!pSkeleton->IsInferred(JOINT_TYPE_HAND_LEFT))
			{
				/*MString debug1 = MString("In driver: ") + pSkeleton->IsInferred(JOINT_TYPE_HAND_LEFT) + pSkeleton->GetJoint3DPosX(JOINT_TYPE_HAND_LEFT) + pSkeleton->GetJoint3DPosY(JOINT_TYPE_HAND_LEFT) + pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HAND_LEFT);
				returnStatus.perror(  debug1);*/
				double distanceUnit = 1.0;
				MDistance::Unit currentUnit = MDistance::internalUnit();
				if(currentUnit != MDistance::kInvalid)
				{
					distanceUnit = MDistance(1.0, MDistance::kMeters).as(currentUnit);
				}
				otHandle.set( distanceUnit * pSkeleton->GetJoint3DPosX(JOINT_TYPE_HAND_LEFT),
					distanceUnit * pSkeleton->GetJoint3DPosY(JOINT_TYPE_HAND_LEFT),
					distanceUnit * pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HAND_LEFT) );
			}
			else
			{
				otHandle.set( 0.0, 0.0, 0.0 );
			}
			datablock.setClean(plug);
		}
		else if(plug == aRightHandValid)
		{
			MDataHandle otHandle = datablock.outputValue( aRightHandValid );
			otHandle.set( !pSkeleton->IsInferred(JOINT_TYPE_HAND_RIGHT) );
			datablock.setClean(plug);
		}
		else if(plug == aRightHandTranslate ||
			plug == aRightHandTranslateX ||
			plug == aRightHandTranslateY ||
			plug == aRightHandTranslateZ)
		{
			MDataHandle otHandle = datablock.outputValue( aRightHandTranslate );
			if(!pSkeleton->IsInferred(JOINT_TYPE_HAND_RIGHT))
			{
				double distanceUnit = 1.0;
				MDistance::Unit currentUnit = MDistance::internalUnit();
				if(currentUnit != MDistance::kInvalid)
				{
					distanceUnit = MDistance(1.0, MDistance::kMeters).as(currentUnit);
				}
				otHandle.set( distanceUnit * pSkeleton->GetJoint3DPosX(JOINT_TYPE_HAND_RIGHT),
					distanceUnit * pSkeleton->GetJoint3DPosY(JOINT_TYPE_HAND_RIGHT),
					distanceUnit * pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HAND_RIGHT) );
			}
			else
			{
				otHandle.set( 0.0, 0.0, 0.0 );
			}
			datablock.setClean(plug);
		}
		else if(plug == aLeftElbowValid)
		{
			MDataHandle otHandle = datablock.outputValue( aLeftElbowValid );
			otHandle.set( !pSkeleton->IsInferred(JOINT_TYPE_ELBOW_LEFT) );
			datablock.setClean(plug);
		}
		else if(plug == aLeftElbowTranslate ||
			plug == aLeftElbowTranslateX ||
			plug == aLeftElbowTranslateY ||
			plug == aLeftElbowTranslateZ)
		{
			MDataHandle otHandle = datablock.outputValue( aLeftElbowTranslate );
			if(!pSkeleton->IsInferred(JOINT_TYPE_ELBOW_LEFT))
			{
				/*MString debug1 = MString("In driver: ") + pSkeleton->IsInferred(JOINT_TYPE_HAND_LEFT) + pSkeleton->GetJoint3DPosX(JOINT_TYPE_HAND_LEFT) + pSkeleton->GetJoint3DPosY(JOINT_TYPE_HAND_LEFT) + pSkeleton->GetJoint3DPosZ(JOINT_TYPE_HAND_LEFT);
				returnStatus.perror(  debug1);*/
				double distanceUnit = 1.0;
				MDistance::Unit currentUnit = MDistance::internalUnit();
				if(currentUnit != MDistance::kInvalid)
				{
					distanceUnit = MDistance(1.0, MDistance::kMeters).as(currentUnit);
				}
				otHandle.set( distanceUnit * pSkeleton->GetJoint3DPosX(JOINT_TYPE_ELBOW_LEFT),
					distanceUnit * pSkeleton->GetJoint3DPosY(JOINT_TYPE_ELBOW_LEFT),
					distanceUnit * pSkeleton->GetJoint3DPosZ(JOINT_TYPE_ELBOW_LEFT) );
			}
			else
			{
				otHandle.set( 0.0, 0.0, 0.0 );
			}
			datablock.setClean(plug);
		}
		else if(plug == aRightElbowValid)
		{
			MDataHandle otHandle = datablock.outputValue( aRightElbowValid );
			otHandle.set( !pSkeleton->IsInferred(JOINT_TYPE_ELBOW_RIGHT) );
			datablock.setClean(plug);
		}
		else if(plug == aRightHandTranslate ||
			plug == aRightHandTranslateX ||
			plug == aRightHandTranslateY ||
			plug == aRightHandTranslateZ)
		{
			MDataHandle otHandle = datablock.outputValue( aRightElbowTranslate );
			if(!pSkeleton->IsInferred(JOINT_TYPE_ELBOW_RIGHT))
			{
				double distanceUnit = 1.0;
				MDistance::Unit currentUnit = MDistance::internalUnit();
				if(currentUnit != MDistance::kInvalid)
				{
					distanceUnit = MDistance(1.0, MDistance::kMeters).as(currentUnit);
				}
				otHandle.set( distanceUnit * pSkeleton->GetJoint3DPosX(JOINT_TYPE_ELBOW_RIGHT),
					distanceUnit * pSkeleton->GetJoint3DPosY(JOINT_TYPE_ELBOW_RIGHT),
					distanceUnit * pSkeleton->GetJoint3DPosZ(JOINT_TYPE_ELBOW_RIGHT) );
			}
			else
			{
				otHandle.set( 0.0, 0.0, 0.0 );
			}
			datablock.setClean(plug);
		}
		else
		{
			computeWholeSkeleton(datablock, pSkeleton);
		}
	}
	/*else {
		return MS::kUnknownParameter;
	}*/

	return MS::kSuccess;
}

MStatus NuiMayaSkeletonDriver::initialize()
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
	aInputSkeletonData = typedAttr.create( "inputSkeletonData", "isd",
		NuiMayaSkeletonData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create inputSkeletonData attribute"); return stat;}
	typedAttr.setReadable( false );
	typedAttr.setWritable( true );
	typedAttr.setStorable( false );
	stat = addAttribute( aInputSkeletonData );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aInputGestureData = typedAttr.create( "inputGestureData", "igd",
		NuiMayaGestureData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create inputGestureData attribute"); return stat;}
	typedAttr.setReadable( false );
	typedAttr.setWritable( true );
	typedAttr.setStorable( false );
	stat = addAttribute( aInputGestureData );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// ----------------------- OUTPUTS -------------------------
	// Gesture state
	aGestureStates = nAttr.create( "gestureStates", "gss", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create gestureStates attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	nAttr.setArray(true);
	nAttr.setUsesArrayDataBuilder(true);
	stat = addAttribute( aGestureStates );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// Hand state
	aLeftHandState = nAttr.create( "leftHandState", "lhs", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create leftHandState attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftHandState );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aRightHandState = nAttr.create( "rightHandState", "rhs", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create rightHandState attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightHandState );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// Head
	aHeadRelativeTranslateX = nAttr.create( "headRelativeTranslateX", "hrtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create headRelativeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aHeadRelativeTranslateY = nAttr.create( "headRelativeTranslateY", "hrtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create headRelativeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aHeadRelativeTranslateZ = nAttr.create( "headRelativeTranslateZ", "hrtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create headRelativeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aHeadRelativeTranslate = nAttr.create( "headRelativeTranslate", "hrt", aHeadRelativeTranslateX, aHeadRelativeTranslateY, aHeadRelativeTranslateZ, &stat );
	if (!stat) { stat.perror("create headRelativeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aHeadRelativeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aHeadTranslateX = nAttr.create( "headTranslateX", "htX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create headTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aHeadTranslateY = nAttr.create( "headTranslateY", "htY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create headTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aHeadTranslateZ = nAttr.create( "headTranslateZ", "htZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create headTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aHeadTranslate = nAttr.create( "headTranslate", "ht", aHeadTranslateX, aHeadTranslateY, aHeadTranslateZ, &stat );
	if (!stat) { stat.perror("create headTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aHeadTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aHeadValid = nAttr.create( "headValid", "hv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create headValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aHeadValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// Neck
	aNeckRelativeTranslateX = nAttr.create( "neckRelativeTranslateX", "nrtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create neckRelativeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aNeckRelativeTranslateY = nAttr.create( "neckRelativeTranslateY", "nrtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create neckRelativeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aNeckRelativeTranslateZ = nAttr.create( "neckRelativeTranslateZ", "nrtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create neckRelativeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aNeckRelativeTranslate = nAttr.create( "neckRelativeTranslate", "nrt", aNeckRelativeTranslateX, aNeckRelativeTranslateY, aNeckRelativeTranslateZ, &stat );
	if (!stat) { stat.perror("create neckRelativeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aNeckRelativeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aNeckValid = nAttr.create( "neckValid", "nv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create neckValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aNeckValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// LeftShoulder
	aLeftShoulderRelativeTranslateX = nAttr.create( "leftShoulderRelativeTranslateX", "lsrtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftShoulderRelativeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftShoulderRelativeTranslateY = nAttr.create( "leftShoulderRelativeTranslateY", "lsrtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftShoulderTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftShoulderRelativeTranslateZ = nAttr.create( "leftShoulderRelativeTranslateZ", "lsrtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftShoulderRelativeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftShoulderRelativeTranslate = nAttr.create( "leftShoulderRelativeTranslate", "lsrt", aLeftShoulderRelativeTranslateX, aLeftShoulderRelativeTranslateY, aLeftShoulderRelativeTranslateZ, &stat );
	if (!stat) { stat.perror("create leftShoulderRelativeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftShoulderRelativeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aLeftShoulderValid = nAttr.create( "leftShoulderValid", "lsv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create leftShoulderValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftShoulderValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// RightShoulder
	aRightShoulderRelativeTranslateX = nAttr.create( "rightShoulderRelativeTranslateX", "rsrtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightShoulderRelativeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightShoulderRelativeTranslateY = nAttr.create( "rightShoulderRelativeTranslateY", "rsrtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightShoulderRelativeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightShoulderRelativeTranslateZ = nAttr.create( "rightShoulderRelativeTranslateZ", "rsrtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightShoulderRelativeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightShoulderRelativeTranslate = nAttr.create( "rightShoulderRelativeTranslate", "rsrt", aRightShoulderRelativeTranslateX, aRightShoulderRelativeTranslateY, aRightShoulderRelativeTranslateZ, &stat );
	if (!stat) { stat.perror("create rightShoulderRelativeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightShoulderRelativeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aRightShoulderValid = nAttr.create( "rightShoulderValid", "rsv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create rightShoulderValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightShoulderValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// LeftElbow
	aLeftElbowRelativeTranslateX = nAttr.create( "leftElbowRelativeTranslateX", "lertX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftElbowRelativeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftElbowRelativeTranslateY = nAttr.create( "leftElbowRelativeTranslateY", "lertY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftElbowRelativeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftElbowRelativeTranslateZ = nAttr.create( "leftElbowRelativeTranslateZ", "lertZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftElbowRelativeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftElbowRelativeTranslate = nAttr.create( "leftElbowRelativeTranslate", "lert", aLeftElbowRelativeTranslateX, aLeftElbowRelativeTranslateY, aLeftElbowRelativeTranslateZ, &stat );
	if (!stat) { stat.perror("create leftElbowRelativeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftElbowRelativeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aLeftElbowTranslateX = nAttr.create( "leftElbowTranslateX", "letX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftElbowTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftElbowTranslateY = nAttr.create( "leftElbowTranslateY", "letY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftElbowTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftElbowTranslateZ = nAttr.create( "leftElbowTranslateZ", "letZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftElbowTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftElbowTranslate = nAttr.create( "leftElbowTranslate", "let", aLeftElbowTranslateX, aLeftElbowTranslateY, aLeftElbowTranslateZ, &stat );
	if (!stat) { stat.perror("create leftElbowTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftElbowTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aLeftElbowValid = nAttr.create( "leftElbowValid", "lev", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create leftElbowValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftElbowValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// RightElbow
	aRightElbowRelativeTranslateX = nAttr.create( "rightElbowRelativeTranslateX", "rertX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightElbowRelativeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightElbowRelativeTranslateY = nAttr.create( "rightElbowRelativeTranslateY", "rertY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightElbowRelativeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightElbowRelativeTranslateZ = nAttr.create( "rightElbowRelativeTranslateZ", "rertZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightElbowRelativeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightElbowRelativeTranslate = nAttr.create( "rightElbowRelativeTranslate", "rert", aRightElbowRelativeTranslateX, aRightElbowRelativeTranslateY, aRightElbowRelativeTranslateZ, &stat );
	if (!stat) { stat.perror("create rightElbowRelativeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightElbowRelativeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aRightElbowTranslateX = nAttr.create( "rightElbowTranslateX", "retX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightElbowTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightElbowTranslateY = nAttr.create( "rightElbowTranslateY", "retY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightElbowTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightElbowTranslateZ = nAttr.create( "rightElbowTranslateZ", "retZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightElbowTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightElbowTranslate = nAttr.create( "rightElbowTranslate", "ret", aRightElbowTranslateX, aRightElbowTranslateY, aRightElbowTranslateZ, &stat );
	if (!stat) { stat.perror("create rightElbowTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightElbowTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aRightElbowValid = nAttr.create( "rightElbowValid", "rev", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create rightElbowValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightElbowValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// LeftHead
	aLeftHandRelativeTranslateX = nAttr.create( "leftHandRelativeTranslateX", "lhrtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHandRelativeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftHandRelativeTranslateY = nAttr.create( "leftHandRelativeTranslateY", "lhrtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHandRelativeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftHandRelativeTranslateZ = nAttr.create( "leftHandRelativeTranslateZ", "lhrtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHandRelativeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftHandRelativeTranslate = nAttr.create( "leftHandRelativeTranslate", "lhrt", aLeftHandRelativeTranslateX, aLeftHandRelativeTranslateY, aLeftHandRelativeTranslateZ, &stat );
	if (!stat) { stat.perror("create leftHandRelativeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftHandRelativeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aLeftHandTranslateX = nAttr.create( "leftHandTranslateX", "lhtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHandTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(false);
	aLeftHandTranslateY = nAttr.create( "leftHandTranslateY", "lhtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHandTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(false);
	aLeftHandTranslateZ = nAttr.create( "leftHandTranslateZ", "lhtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHandTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(false);
	aLeftHandTranslate = nAttr.create( "leftHandTranslate", "lht", aLeftHandTranslateX, aLeftHandTranslateY, aLeftHandTranslateZ, &stat );
	if (!stat) { stat.perror("create leftHandTranslate attribute"); return stat;}
	nAttr.setDefault(0.0, 0.0, 0.0);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(false);
	stat = addAttribute( aLeftHandTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aLeftHandValid = nAttr.create( "leftHandValid", "lhv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create leftHandValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftHandValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// RightHead
	aRightHandRelativeTranslateX = nAttr.create( "rightHandRelativeTranslateX", "rhrtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHandRelativeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHandRelativeTranslateY = nAttr.create( "rightHandRelativeTranslateY", "rhrtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHandRelativeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHandRelativeTranslateZ = nAttr.create( "rightHandRelativeTranslateZ", "rhrtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHandRelativeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHandRelativeTranslate = nAttr.create( "rightHandRelativeTranslate", "rhrt", aRightHandRelativeTranslateX, aRightHandRelativeTranslateY, aRightHandRelativeTranslateZ, &stat );
	if (!stat) { stat.perror("create rightHandRelativeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightHandRelativeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aRightHandTranslateX = nAttr.create( "rightHandTranslateX", "rhtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHandTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHandTranslateY = nAttr.create( "rightHandTranslateY", "rhtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHandTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHandTranslateZ = nAttr.create( "rightHandTranslateZ", "rhtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHandTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHandTranslate = nAttr.create( "rightHandTranslate", "rht", aRightHandTranslateX, aRightHandTranslateY, aRightHandTranslateZ, &stat );
	if (!stat) { stat.perror("create rightHandTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightHandTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aRightHandValid = nAttr.create( "rightHandValid", "rhv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create rightHandValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightHandValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// Torso
	aTorsoRelativeTranslateX = nAttr.create( "torsoRelativeTranslateX", "trtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create torsoRelativeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aTorsoRelativeTranslateY = nAttr.create( "torsoRelativeTranslateY", "trtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create torsoRelativeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aTorsoRelativeTranslateZ = nAttr.create( "torsoRelativeTranslateZ", "trtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create torsoRelativeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aTorsoRelativeTranslate = nAttr.create( "torsoRelativeTranslate", "trt", aTorsoRelativeTranslateX, aTorsoRelativeTranslateY, aTorsoRelativeTranslateZ, &stat );
	if (!stat) { stat.perror("create torsoRelativeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aTorsoRelativeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aTorsoValid = nAttr.create( "torsoValid", "tv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create torsoValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aTorsoValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// LeftHip
	aLeftHipRelativeTranslateX = nAttr.create( "leftHipRelativeTranslateX", "lhprX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHipRelativeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftHipRelativeTranslateY = nAttr.create( "leftHipRelativeTranslateY", "lhprY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHipRelativeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftHipRelativeTranslateZ = nAttr.create( "leftHipRelativeTranslateZ", "lhprZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHipRelativeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftHipRelativeTranslate = nAttr.create( "leftHipRelativeTranslate", "lhpr", aLeftHipRelativeTranslateX, aLeftHipRelativeTranslateY, aLeftHipRelativeTranslateZ, &stat );
	if (!stat) { stat.perror("create leftHipRelativeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftHipRelativeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aLeftHipValid = nAttr.create( "leftHipValid", "lhpv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create leftHipValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftHipValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// RightHip
	aRightHipRelativeTranslateX = nAttr.create( "rightHipRelativeTranslateX", "rhprX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHipRelativeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHipRelativeTranslateY = nAttr.create( "rightHipRelativeTranslateY", "rhprY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHipRelativeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHipRelativeTranslateZ = nAttr.create( "rightHipRelativeTranslateZ", "rhprZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHipRelativeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHipRelativeTranslate = nAttr.create( "rightHipRelativeTranslate", "rhpr", aRightHipRelativeTranslateX, aRightHipRelativeTranslateY, aRightHipRelativeTranslateZ, &stat );
	if (!stat) { stat.perror("create rightHipRelativeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightHipRelativeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aRightHipValid = nAttr.create( "rightHipValid", "rhpv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create rightHipValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightHipValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// LeftKnee
	aLeftKneeRelativeTranslateX = nAttr.create( "leftKneeRelativeTranslateX", "lkrtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftKneeRelativeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftKneeRelativeTranslateY = nAttr.create( "leftKneeRelativeTranslateY", "lkrrtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftKneeRelativeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftKneeRelativeTranslateZ = nAttr.create( "leftKneeRelativeTranslateZ", "lktZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftKneeRelativeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftKneeRelativeTranslate = nAttr.create( "leftKneeRelativeTranslate", "lkrt", aLeftKneeRelativeTranslateX, aLeftKneeRelativeTranslateY, aLeftKneeRelativeTranslateZ, &stat );
	if (!stat) { stat.perror("create leftKneeRelativeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftKneeRelativeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aLeftKneeValid = nAttr.create( "leftKneeValid", "lkv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create leftKneeValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftKneeValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// RightKnee
	aRightKneeRelativeTranslateX = nAttr.create( "rightKneeRelativeTranslateX", "rkrtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightKneeRelativeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightKneeRelativeTranslateY = nAttr.create( "rightKneeRelativeTranslateY", "rkrtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightKneeRelativeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightKneeRelativeTranslateZ = nAttr.create( "rightKneeRelativeTranslateZ", "rkrtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightKneeRelativeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightKneeRelativeTranslate = nAttr.create( "rightKneeRelativeTranslate", "rkrt", aRightKneeRelativeTranslateX, aRightKneeRelativeTranslateY, aRightKneeRelativeTranslateZ, &stat );
	if (!stat) { stat.perror("create rightKneeRelativeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightKneeRelativeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aRightKneeValid = nAttr.create( "rightKneeValid", "rkv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create rightKneeValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightKneeValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// LeftFoot
	aLeftFootRelativeTranslateX = nAttr.create( "leftFootRelativeTranslateX", "lfrtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftFootRelativeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftFootRelativeTranslateY = nAttr.create( "leftFootRelativeTranslateY", "lfrtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftFootRelativeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftFootRelativeTranslateZ = nAttr.create( "leftFootRelativeTranslateZ", "lfrtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftFootRelativeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftFootRelativeTranslate = nAttr.create( "leftFootRelativeTranslate", "lfrt", aLeftFootRelativeTranslateX, aLeftFootRelativeTranslateY, aLeftFootRelativeTranslateZ, &stat );
	if (!stat) { stat.perror("create leftFootRelativeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftFootRelativeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aLeftFootValid = nAttr.create( "leftFootValid", "lfv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create leftFootValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftFootValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// RightFoot
	aRightFootRelativeTranslateX = nAttr.create( "rightFootRelativeTranslateX", "rfrtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightFootRelativeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightFootRelativeTranslateY = nAttr.create( "rightFootRelativeTranslateY", "rfrtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightFootRelativeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightFootRelativeTranslateZ = nAttr.create( "rightFootRelativeTranslateZ", "rfrtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightFootRelativeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightFootRelativeTranslate = nAttr.create( "rightFootRelativeTranslate", "rfrt", aRightFootRelativeTranslateX, aRightFootRelativeTranslateY, aRightFootRelativeTranslateZ, &stat );
	if (!stat) { stat.perror("create rightFootRelativeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightFootRelativeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aRightFootValid = nAttr.create( "rightFootValid", "rfv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create rightFootValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightFootValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// CenterMass
	aCenterMassTranslateX = nAttr.create( "centerMassTranslateX", "cmtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create centerMassTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCenterMassTranslateY = nAttr.create( "centerMassTranslateY", "cmtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create centerMassTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCenterMassTranslateZ = nAttr.create( "centerMassTranslateZ", "cmtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create centerMassTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCenterMassTranslate = nAttr.create( "centerMassTranslate", "cmt", aCenterMassTranslateX, aCenterMassTranslateY, aCenterMassTranslateZ, &stat );
	if (!stat) { stat.perror("create centerMassTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aCenterMassTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aCenterMassValid = nAttr.create( "centerMassValid", "cmv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create centerMassValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aCenterMassValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}


	// Set up a dependency between the input and the output.  This will cause
	// the output to be marked dirty when the input changes.  The output will
	// then be recomputed the next time the value of the output is requested.
	//
	stat = attributeAffects( aInputGestureData, aGestureStates );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aLeftHandState );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightHandState );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aCenterMassTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aCenterMassTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aCenterMassTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aCenterMassTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aCenterMassValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aTorsoRelativeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aTorsoRelativeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aTorsoRelativeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aTorsoRelativeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aTorsoValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aNeckRelativeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aNeckRelativeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aNeckRelativeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aNeckRelativeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aNeckValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aHeadRelativeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aHeadRelativeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aHeadRelativeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aHeadRelativeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aHeadTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aHeadTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aHeadTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aHeadTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aHeadValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aLeftShoulderRelativeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftShoulderRelativeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftShoulderRelativeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftShoulderRelativeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftShoulderValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aRightShoulderRelativeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightShoulderRelativeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightShoulderRelativeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightShoulderRelativeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightShoulderValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aLeftElbowRelativeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftElbowRelativeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftElbowRelativeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftElbowRelativeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftElbowTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftElbowTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftElbowTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftElbowTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftElbowValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aRightElbowRelativeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightElbowRelativeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightElbowRelativeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightElbowRelativeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightElbowTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightElbowTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightElbowTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightElbowTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightElbowValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aLeftHandRelativeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftHandRelativeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftHandRelativeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftHandRelativeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftHandTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftHandTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftHandTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftHandTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftHandValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aRightHandRelativeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightHandRelativeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightHandRelativeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightHandRelativeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightHandTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightHandTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightHandTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightHandTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightHandValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aLeftHipRelativeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftHipRelativeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftHipRelativeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftHipRelativeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftHipValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aRightHipRelativeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightHipRelativeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightHipRelativeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightHipRelativeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightHipValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aLeftKneeRelativeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftKneeRelativeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftKneeRelativeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftKneeRelativeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftKneeValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aRightKneeRelativeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightKneeRelativeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightKneeRelativeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightKneeRelativeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightKneeValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aLeftFootRelativeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftFootRelativeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftFootRelativeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftFootRelativeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aLeftFootValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputSkeletonData, aRightFootRelativeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightFootRelativeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightFootRelativeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightFootRelativeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputSkeletonData, aRightFootValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	return MS::kSuccess;
}