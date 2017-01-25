#include "NuiMayaMappableData.h"
#include "NuiMayaMappableDataIterator.h"
#include "../api_macros.h"

#include <maya/MPlug.h>
#include <maya/MFnPluginData.h>
#include <maya/MIOStream.h>
#include <maya/MFnSingleIndexedComponent.h>
#include <maya/MArgList.h>

//////////////////////////////////////////////////////////////////////

// Ascii file IO defines
//
#define kDblQteChar				"\""
#define kSpaceChar				"	"
#define kWrapString				"\n\t\t"
#define kPCDKeyword				"DeviceImageCLData"
#define kPCDName				"DeviceImageCLDataName"

//////////////////////////////////////////////////////////////////////

const MTypeId NuiMayaMappableData::id( 0x80006 );
const MString NuiMayaMappableData::typeName( "deviceMappableCLData" );

/* override */
MStatus NuiMayaMappableData::readASCII( const MArgList& argList, unsigned& index )
//
// Description
//    ASCII file input method.
//
{
	MStatus result;
	MString typeStr;

	result = argList.get( index, typeStr );

	if ( result && (typeStr == kPCDKeyword) ) {
		result = argList.get( ++index, typeStr );

		if ( result && m_pCLData && (typeStr == kPCDName) ) {
			// TODO: read
		}
	}

	return result;
}

/* override */
MStatus NuiMayaMappableData::readBinary( istream& /*in*/, unsigned /*length*/ )
//
// Description
//     NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
MStatus NuiMayaMappableData::writeASCII( ostream& out )
//
// Description
//    ASCII file output method.
//
{
	out << "\n";
	out << kWrapString;
	out << kDblQteChar << kPCDKeyword << kDblQteChar
		<< kSpaceChar << kPCDName;

	// TODO: write

	return MS::kSuccess;
}

/* override */
MStatus NuiMayaMappableData::writeBinary( ostream& /*out*/ )
//
// Description
//    NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
void NuiMayaMappableData::copy ( const MPxData& other )
{
	m_pCLData = (((const NuiMayaMappableData &)other).m_pCLData);
}

/* override */
MTypeId NuiMayaMappableData::typeId() const
//
// Description
//    Binary tag used to identify this kind of data
//
{
	return NuiMayaMappableData::id;
}

/* override */
MString NuiMayaMappableData::name() const
//
// Description
//    String name used to identify this kind of data
//
{
	return NuiMayaMappableData::typeName;
}

void * NuiMayaMappableData::creator()
{
	return new NuiMayaMappableData;
}

/* override */
MPxGeometryIterator* NuiMayaMappableData::iterator( MObjectArray & componentList,
											MObject & component,
											bool useComponents)
//
// Description
//
{
	NuiMayaMappableDataIterator * result = NULL;
	if ( useComponents ) {
		result = new NuiMayaMappableDataIterator( static_cast<void*>(m_pCLData.get()), componentList );
	}
	else {
		result = new NuiMayaMappableDataIterator( static_cast<void*>(m_pCLData.get()), component );
	}
	return result;
}

/* override */
MPxGeometryIterator* NuiMayaMappableData::iterator( MObjectArray & componentList,
											MObject & component,
											bool useComponents,
											bool /*world*/) const
//
// Description
//
{
	NuiMayaMappableDataIterator * result = NULL;
	if ( useComponents ) {
		result = new NuiMayaMappableDataIterator( static_cast<void*>(m_pCLData.get()), componentList );
	}
	else {
		result = new NuiMayaMappableDataIterator( static_cast<void*>(m_pCLData.get()), component );
	}
	return result;
}

/* override */
bool NuiMayaMappableData::updateCompleteVertexGroup( MObject & component ) const
//
// Description
//     Make sure complete vertex group data is up-to-date.
//     Returns true if the component was updated, false if it was already ok.
//
//     This is used by deformers when deforming the "whole" object and
//     not just selected components.
//
{
	MStatus stat;
	MFnSingleIndexedComponent fnComponent( component, &stat );

	// Make sure there is non-null geometry and that the component
	// is "complete". A "complete" component represents every 
	// vertex in the shape.
	//
	if ( stat && m_pCLData && (fnComponent.isComplete()) ) {
	
		int maxVerts ;
		fnComponent.getCompleteData( maxVerts );
		const int numVertices = m_pCLData->GetPositionNum();

		if ( (numVertices > 0) && (maxVerts != numVertices) ) {
			// Set the component to be complete, i.e. the elements in
			// the component will be [0:numVertices-1]
			//
			fnComponent.setCompleteData( numVertices );
			return true;
		}
	}

	return false;
}

std::shared_ptr<NuiCLMappableData> NuiMayaMappableData::findDataFromXgmData(const MObject& object)
{
	// Find the internal Maya data letter. Empty data is allowed.
	NuiMayaMappableData* mayaDataLetter =
		static_cast<NuiMayaMappableData*>(MFnPluginData(object).data());
	if (mayaDataLetter)
	{
		// Get the wrapped spline data
		return mayaDataLetter->data();
	}
	return nullptr;
}

std::shared_ptr<NuiCLMappableData> NuiMayaMappableData::findData(const MPlug& plug)
{
	static const std::shared_ptr<NuiCLMappableData> sNullData;

	// Null plug ?
	if (plug.isNull()) return sNullData;

	// Null data ?
	MObject object = plug.asMObject();
	if (object.isNull()) return sNullData;

	return findDataFromXgmData(object);
}

std::shared_ptr<NuiCLMappableData> NuiMayaMappableData::findData(const MObject& object, const MObject& attribute)
{
	return findData(MPlug(object, attribute));
}

MObject NuiMayaMappableData::createSharedData(const MObject& data)
{
	MStatus status = MS::kSuccess;

	// Create an empty object always
	MObject object;
	NuiMayaMappableData* destination = nullptr;
	{
		MFnPluginData fnDataCreator;
		object = fnDataCreator.create(NuiMayaMappableData::id, &status);
		MSTATS_ERROR_RETURN_VALUE( status, object)

		destination = static_cast<NuiMayaMappableData*>(fnDataCreator.data(&status));
		assert(destination);
		MSTATS_ERROR_RETURN_VALUE( status, object)

		if (!destination) return object;
	}

	// Return the object with empty plugin data
	if (data.isNull())
		return object;

	// Find the source data for sharing
	std::shared_ptr<NuiCLMappableData> clData;
	{
		MFnPluginData clDataFn(data, &status);
		MSTATS_ERROR_RETURN_VALUE( status, object)

		NuiMayaMappableData* source = static_cast<NuiMayaMappableData*>(clDataFn.data(&status));
		MSTATS_ERROR_RETURN_VALUE( status, object)

		if (source)
			clData = source->data();
	}

	// If we don't have any source data, just use an null data
	if (!clData)
		return object;

	destination->data() = clData;
	//// Create a new XgmSplineData sharing the same internal XgSplineData
	//if (notShareFlag == XG_STREAM_SEMANTIC::NONE) {
	//	destination->data() = clData;
	//}
	//else
	//{
	//	destination->data() = xgData->makeShared(notShareFlag);
	//}

	return object;
}
