#pragma once

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MCallbackIdArray.h>
#include <maya/MNodeMessage.h>

#include "NuiMayaCacheTimer.h"

// Forwards
class NuiRGBDDeviceController;
class NuiFrameCacheImpl;
class NuiKinfuManager;
class NuiMeshShape;

class NuiMayaDeviceGrabber : public MPxNode
{
public:
							NuiMayaDeviceGrabber();
	virtual					~NuiMayaDeviceGrabber(); 
	// Override functions
	virtual void			postConstructor();
	virtual MStatus			compute( const MPlug& plug, MDataBlock& datablock );

	virtual MStatus			connectionMade( const MPlug& plug,
											const MPlug& otherPlug,
											bool asSrc );
	virtual MStatus			connectionBroken( const MPlug& plug,
											const MPlug& otherPlug,
											bool asSrc );

	static  void*			creator();
	static  MStatus			initialize();

	// Callbacks
	static void				attrChangedCB(MNodeMessage::AttributeMessage msg, MPlug & plug, MPlug & otherPlug, void* clientData);

	// Helper functions
	bool					updateDevice();
	void					updateCache();
	bool					updateNearMode();
	void					updateaElevationAngle();
	void					updatePreviewer();
	void					updateKinfu();

	bool					updatePreviewRendererMode();

public:
	// There needs to be a MObject handle declared for each attribute that
	// the node will have.  These handles are needed for getting and setting
	// the values later.
	//
	static  MObject			aTime;
	static  MObject			aDeviceOn;
	static  MObject			aDeviceMode;
	static  MObject			aUseCache;
	static  MObject			aNearMode;
	static  MObject			aElevationAngle;
	static  MObject			aPreviewerOn;
	static  MObject			aKinFuOn;

	static  MObject			aShowMesh;
	static  MObject			aShowInvalid;
	static  MObject			aShowOnlyBody;

	static  MObject			aMinDepth;
	static  MObject			aMaxDepth;

	static  MObject			aOutputPointCloud;
	static  MObject			aOutputMesh;
	static  MObject			aOutputSkeleton;
	static  MObject			aOutputGesture;
	static  MObject			aOutputFacialModel;

	static  MObject			aFaceRotateX;
	static  MObject			aFaceRotateY;
	static  MObject			aFaceRotateZ;
	static  MObject			aFaceRotate;

	static  MObject			aCameraRotateX;
	static  MObject			aCameraRotateY;
	static  MObject			aCameraRotateZ;
	static  MObject			aCameraRotate;
	static  MObject			aCameraTranslateX;
	static  MObject			aCameraTranslateY;
	static  MObject			aCameraTranslateZ;
	static  MObject			aCameraTranslate;

	// The typeid is a unique 32bit indentifier that describes this node.
	// It is used to save and retrieve nodes of this type from the binary
	// file format.  If it is not unique, it will cause file IO problems.
	//
	static	MTypeId			id;

private:
	MCallbackIdArray		fCallbackIds;

	void					addCallbacks();
	void					removeCallbacks();

	void					startPreviewer();
	void					stopPreviewer();
	static void             PreviewerCallingBack(void* lpParam);

	bool					getBooleanValue(const MObject &attribute);
	UINT16					getShortValue(const MObject &attribute);

	MStatus					computeOutputMesh( NuiMeshShape* pMesh,
		MDataBlock& datablock,
		MObject& meshData );
	MStatus					assignMeshUV( MObject&	meshData, const MIntArray& polygonCounts, const MIntArray& uvIds );
	void					createEmptyMesh( MObject& out_empytMesh );

private:	
	NuiRGBDDeviceController*	m_pDevice;
	NuiFrameCacheImpl*		m_pCache;
	NuiMayaCacheTimer*		m_pPreviewer;

	NuiKinfuManager*		m_kinfu;
};