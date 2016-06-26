#include "NuiMayaMeshData.h"
#include "../api_macros.h"

#include <assert.h>

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
#define kPCDKeyword				"DeviceMeshData"
#define kPCDName				"DeviceMeshDataName"

//////////////////////////////////////////////////////////////////////

const MTypeId NuiMayaMeshData::id( 0x80226 );
const MString NuiMayaMeshData::typeName( "deviceMeshData" );

/* override */
MStatus NuiMayaMeshData::readASCII( const MArgList& argList, unsigned& index )
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

		if ( result && m_pData && (typeStr == kPCDName) ) {
			// TODO: read
		}
	}

	return result;
}

/* override */
MStatus NuiMayaMeshData::readBinary( istream& /*in*/, unsigned /*length*/ )
	//
	// Description
	//     NOT IMPLEMENTED
	//
{
	return MS::kSuccess;
}

/* override */
MStatus NuiMayaMeshData::writeASCII( ostream& out )
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
MStatus NuiMayaMeshData::writeBinary( ostream& /*out*/ )
	//
	// Description
	//    NOT IMPLEMENTED
	//
{
	return MS::kSuccess;
}

/* override */
void NuiMayaMeshData::copy ( const MPxData& other )
{
	m_pData = (((const NuiMayaMeshData &)other).m_pData);
}

/* override */
MTypeId NuiMayaMeshData::typeId() const
//
// Description
//    Binary tag used to identify this kind of data
//
{
	return NuiMayaMeshData::id;
}

/* override */
MString NuiMayaMeshData::name() const
//
// Description
//    String name used to identify this kind of data
//
{
	return NuiMayaMeshData::typeName;
}

void * NuiMayaMeshData::creator()
{
	return new NuiMayaMeshData;
}

std::shared_ptr<NuiMeshShape> NuiMayaMeshData::findDataFromXgmData(const MObject& object)
{
	// Find the internal Maya data letter. Empty data is allowed.
	NuiMayaMeshData* mayaDataLetter =
		static_cast<NuiMayaMeshData*>(MFnPluginData(object).data());
	if (mayaDataLetter)
	{
		// Get the wrapped spline data
		return mayaDataLetter->data();
	}
	return nullptr;
}

std::shared_ptr<NuiMeshShape> NuiMayaMeshData::findData(const MPlug& plug)
{
	static const std::shared_ptr<NuiMeshShape> sNullData;

	// Null plug ?
	if (plug.isNull()) return sNullData;

	// Null data ?
	MObject object = plug.asMObject();
	if (object.isNull()) return sNullData;

	return findDataFromXgmData(object);
}

std::shared_ptr<NuiMeshShape> NuiMayaMeshData::findData(const MObject& object, const MObject& attribute)
{
	return findData(MPlug(object, attribute));
}

MObject NuiMayaMeshData::createSharedData(const MObject& data)
{
	MStatus status = MS::kSuccess;

	// Create an empty object always
	MObject object;
	NuiMayaMeshData* destination = nullptr;
	{
		MFnPluginData fnDataCreator;
		object = fnDataCreator.create(NuiMayaMeshData::id, &status);
		MSTATS_ERROR_RETURN_VALUE( status, object)

		destination = static_cast<NuiMayaMeshData*>(fnDataCreator.data(&status));
		assert(destination);
		MSTATS_ERROR_RETURN_VALUE( status, object)

			if (!destination) return object;
	}

	// Return the object with empty plugin data
	if (data.isNull())
		return object;

	// Find the source data for sharing
	std::shared_ptr<NuiMeshShape> clData;
	{
		MFnPluginData clDataFn(data, &status);
		MSTATS_ERROR_RETURN_VALUE( status, object)

		NuiMayaMeshData* source = static_cast<NuiMayaMeshData*>(clDataFn.data(&status));
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
