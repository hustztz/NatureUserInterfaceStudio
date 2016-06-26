#include "NuiMayaMeshGenerator.h"
#include "NuiMayaMeshData.h"
#include "../api_macros.h"

#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnPluginData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFloatPointArray.h>
#include <maya/MFloatArray.h>
#include <maya/MDistance.h>
#include <maya/MGlobal.h>

#include <map>

////////////////////////////////////////////////////////////////////////////////
//
// Shape implementation
//
////////////////////////////////////////////////////////////////////////////////

MObject NuiMayaMeshGenerator::aInputData;
MObject NuiMayaMeshGenerator::aOutputMesh;
MObject NuiMayaMeshGenerator::aBodyIndex;
MObject NuiMayaMeshGenerator::aAssignUV;

MTypeId NuiMayaMeshGenerator::id( 0x80190 );

NuiMayaMeshGenerator::NuiMayaMeshGenerator()
	: fNeedDirty(true)
{}
NuiMayaMeshGenerator::~NuiMayaMeshGenerator() {}

///////////////////////////////////////////////////////////////////////////////
//
// Overrides
//
///////////////////////////////////////////////////////////////////////////////

/* override */
MStatus NuiMayaMeshGenerator::compute( const MPlug& plug, MDataBlock& datablock )
//
// Description
//
//    When input attributes are dirty this method will be called to
//    recompute the output attributes.
//
{ 
	MStatus stat;
	if ( plug == aOutputMesh /*&& fNeedDirty*/) {
		// Create some mesh data and access the
		// geometry so we can set it
		//
		MFnMeshData dataCreator;
		MObject newOutputData = dataCreator.create(&stat);
		MCHECKERROR(stat, "ERROR creating outputData");

		// If there is an input mesh then copy it's values
		// and construct some apiMeshGeom for it.
		//
		stat = computeOutputMesh( plug, datablock, newOutputData);
		if(stat != MS::kSuccess)
		{
			createEmptyMesh( newOutputData );
		}
		// Assign the new data to the outputSurface handle
		//
		MDataHandle outHandle = datablock.outputValue( aOutputMesh );
		outHandle.set( newOutputData );
		datablock.setClean( plug );
		// Clear the flag.
		fNeedDirty = false;
		return MS::kSuccess;
	}
	else {
		return MS::kUnknownParameter;
	}
}

void NuiMayaMeshGenerator::createEmptyMesh( MObject& out_empytMesh )
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


MStatus NuiMayaMeshGenerator::assignMeshUV( MObject&	meshData, const MIntArray& polygonCounts, const MIntArray& uvIds )
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

MStatus NuiMayaMeshGenerator::computeOutputMesh( 
	const MPlug&		plug,
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
	MStatus stat;

	MDataHandle inputHandle = datablock.inputValue( aInputData, &stat );
	MCHECKERROR( stat, "computeInputSurface error getting inputSurface")

	// Check if anything is connected
	//
	MObject thisObj = thisMObject();
	MPlug surfPlug( thisObj, aInputData );
	if ( !surfPlug.isConnected() ) {
		stat = datablock.setClean( plug );
		MCHECKERROR( stat, "compute setClean" )
		return MS::kFailure;
	}

	NuiMayaMeshData* surf = (NuiMayaMeshData*) inputHandle.asPluginData();
	if ( NULL == surf ) {
		cerr << "NULL inputCloudData data found\n";
		return MS::kFailure;
	}

	NuiMeshShape* geomPtr = surf->data().get();
	if ( NULL == geomPtr ) {
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
	MIntArray uvIds;

	const UINT numPoints = (UINT)geomPtr->pointsNum();
	for (UINT i = 0; i < numPoints; ++i)
	{
		const SgVec3f& pt = geomPtr->getPoint(i);
		MPoint pnt(pt[0], pt[1], pt[2]);
		vertexArray.append(pnt);

		const SgVec2f& uv = geomPtr->getUV(i);
		uArray.append(uv[0]);
		vArray.append(uv[1]);
		uvIds.append(i);
	}
	const UINT numTris = (UINT)geomPtr->trianglesNum();
	for (UINT i = 0; i < numTris; ++i)
	{
		int index1 = geomPtr->triangleIndex(3*i);
		int index2 = geomPtr->triangleIndex(3*i+1);
		int index3 = geomPtr->triangleIndex(3*i+2);
		if(index1 < 0 || index2 < 0 || index3 < 0)
			continue;

		polygonConnects.append(index1);
		polygonConnects.append(index2);
		polygonConnects.append(index3);
		polygonCounts.append(3);
	}

	MFnMesh meshFn;
	meshFn.create(vertexArray.length(), polygonCounts.length(), vertexArray, polygonCounts, polygonConnects, uArray, vArray, meshData, &stat);

	bool needAssignUV = true;
	/*inputHandle = datablock.inputValue( aAssignUV, &stat );
	if(stat == MS::kSuccess)
		needAssignUV = inputHandle.asBool();*/
	if(needAssignUV)
	//	assignMeshUV(meshData, polygonCounts, uvIds);
	{
		MString uvSetName("uvset1");
		/*MFnMesh meshFn(meshData);
		stat = meshFn.getCurrentUVSetName(uvSetName);
		if ( stat != MS::kSuccess )
		{
			uvSetName = MString ("uvset1");
			stat = meshFn.createUVSet(uvSetName);
			stat = meshFn.setCurrentUVSetName(uvSetName);
		}

		stat = meshFn.clearUVs();
		stat = meshFn.setUVs(uArray,vArray,&uvSetName);*/
		stat = meshFn.assignUVs(polygonCounts, uvIds, &uvSetName);
		if(stat != MS::kSuccess)
			MGlobal::displayError( " Failed to assign UVs." );
	}
	return stat;
}

/* override */
//
// Description
//
//	Horribly abuse the purpose of this method to notify the Viewport 2.0
//  renderer that something about this shape has changed and that it should
//  be retranslated.
//
MStatus NuiMayaMeshGenerator::setDependentsDirty( const MPlug& plug, MPlugArray& plugArray)
{
	// if the dirty attribute is the output mesh then we need to signal the
	// the renderer that it needs to update the object

	/*if ( plug == aInputPointCloud)
	{
		fNeedDirty = true;
	}*/
	return MS::kSuccess;
}

MStatus NuiMayaMeshGenerator::connectionMade( const MPlug& plug,
	const MPlug& otherPlug,
	bool asSrc )
//
// Description
//
//    Whenever a connection to this node is broken, this method
//    will get called.
//
{
	if ( plug == aInputData ) {
		fNeedDirty = true;
	}

	return MPxNode::connectionBroken( plug, otherPlug, asSrc );
}

void* NuiMayaMeshGenerator::creator()
//
// Description
//    Called internally to create a new instance of the users MPx node.
//
{
	return new NuiMayaMeshGenerator();
}

MStatus NuiMayaMeshGenerator::initialize()
//
// Description
//
//    Attribute (static) initialization. See api_macros.h.
//
{ 
	MStatus				stat;
	MFnTypedAttribute	typedAttr;
	MFnNumericAttribute nAttr;

	// ----------------------- INPUTS -------------------------
	aInputData = typedAttr.create( "inputMesh", "ipc",
		NuiMayaMeshData::id,
		MObject::kNullObj, &stat );
	MCHECKERROR( stat, "create inputMesh attribute" )
	typedAttr.setStorable( false );
	ADD_ATTRIBUTE( aInputData );

	aBodyIndex = nAttr.create( "bodyIndex", "bi", MFnNumericData::kByte, 6 );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	nAttr.setKeyable(true);
	nAttr.setMin(0);
	nAttr.setMax(6);
	// Add the attributes we have created to the node
	//
	stat = addAttribute( aBodyIndex );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aAssignUV = nAttr.create( "assignUV", "uv", MFnNumericData::kBoolean, true );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	nAttr.setKeyable(true);
	// Add the attributes we have created to the node
	//
	stat = addAttribute( aAssignUV );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// ----------------------- OUTPUTS -------------------------
	aOutputMesh = typedAttr.create( "outputMesh", "out",
		MFnData::kMesh,
		MObject::kNullObj, &stat );
	MCHECKERROR( stat, "create outputSurface attribute" )
	typedAttr.setWritable( false );
	ADD_ATTRIBUTE( aOutputMesh );

	// ---------- Specify what inputs affect the outputs ----------
	//
	ATTRIBUTE_AFFECTS( aInputData, aOutputMesh );
	ATTRIBUTE_AFFECTS( aBodyIndex, aOutputMesh );
	ATTRIBUTE_AFFECTS( aAssignUV, aOutputMesh );

	return MS::kSuccess;
}