#include "NuiMayaImageCLData.h"
#include "NuiMayaImageCLDataIterator.h"
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

const MTypeId NuiMayaImageCLData::id( 0x80006 );
const MString NuiMayaImageCLData::typeName( "deviceImageCLData" );

/* override */
MStatus NuiMayaImageCLData::readASCII( const MArgList& argList, unsigned& index )
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
MStatus NuiMayaImageCLData::readBinary( istream& /*in*/, unsigned /*length*/ )
//
// Description
//     NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
MStatus NuiMayaImageCLData::writeASCII( ostream& out )
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
MStatus NuiMayaImageCLData::writeBinary( ostream& /*out*/ )
//
// Description
//    NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
void NuiMayaImageCLData::copy ( const MPxData& other )
{
	m_pCLData = (((const NuiMayaImageCLData &)other).m_pCLData);
}

/* override */
MTypeId NuiMayaImageCLData::typeId() const
//
// Description
//    Binary tag used to identify this kind of data
//
{
	return NuiMayaImageCLData::id;
}

/* override */
MString NuiMayaImageCLData::name() const
//
// Description
//    String name used to identify this kind of data
//
{
	return NuiMayaImageCLData::typeName;
}

void * NuiMayaImageCLData::creator()
{
	return new NuiMayaImageCLData;
}

/* override */
MPxGeometryIterator* NuiMayaImageCLData::iterator( MObjectArray & componentList,
											MObject & component,
											bool useComponents)
//
// Description
//
{
	NuiMayaImageCLDataIterator * result = NULL;
	if ( useComponents ) {
		result = new NuiMayaImageCLDataIterator( static_cast<void*>(m_pCLData.get()), componentList );
	}
	else {
		result = new NuiMayaImageCLDataIterator( static_cast<void*>(m_pCLData.get()), component );
	}
	return result;
}

/* override */
MPxGeometryIterator* NuiMayaImageCLData::iterator( MObjectArray & componentList,
											MObject & component,
											bool useComponents,
											bool /*world*/) const
//
// Description
//
{
	NuiMayaImageCLDataIterator * result = NULL;
	if ( useComponents ) {
		result = new NuiMayaImageCLDataIterator( static_cast<void*>(m_pCLData.get()), componentList );
	}
	else {
		result = new NuiMayaImageCLDataIterator( static_cast<void*>(m_pCLData.get()), component );
	}
	return result;
}

/* override */
bool NuiMayaImageCLData::updateCompleteVertexGroup( MObject & component ) const
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

std::shared_ptr<NuiCLMappableData> NuiMayaImageCLData::findDataFromXgmData(const MObject& object)
{
	// Find the internal Maya data letter. Empty data is allowed.
	NuiMayaImageCLData* mayaDataLetter =
		static_cast<NuiMayaImageCLData*>(MFnPluginData(object).data());
	if (mayaDataLetter)
	{
		// Get the wrapped spline data
		return mayaDataLetter->data();
	}
	return nullptr;
}

std::shared_ptr<NuiCLMappableData> NuiMayaImageCLData::findData(const MPlug& plug)
{
	static const std::shared_ptr<NuiCLMappableData> sNullData;

	// Null plug ?
	if (plug.isNull()) return sNullData;

	// Null data ?
	MObject object = plug.asMObject();
	if (object.isNull()) return sNullData;

	return findDataFromXgmData(object);
}

std::shared_ptr<NuiCLMappableData> NuiMayaImageCLData::findData(const MObject& object, const MObject& attribute)
{
	return findData(MPlug(object, attribute));
}

MObject NuiMayaImageCLData::createSharedData(const MObject& data)
{
	MStatus status = MS::kSuccess;

	// Create an empty object always
	MObject object;
	NuiMayaImageCLData* destination = nullptr;
	{
		MFnPluginData fnDataCreator;
		object = fnDataCreator.create(NuiMayaImageCLData::id, &status);
		MSTATS_ERROR_RETURN_VALUE( status, object)

		destination = static_cast<NuiMayaImageCLData*>(fnDataCreator.data(&status));
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

		NuiMayaImageCLData* source = static_cast<NuiMayaImageCLData*>(clDataFn.data(&status));
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
