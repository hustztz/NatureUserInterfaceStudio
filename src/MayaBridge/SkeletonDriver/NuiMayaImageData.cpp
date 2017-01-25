#include <maya/MIOStream.h>
#include "NuiMayaImageData.h"

const MTypeId NuiMayaImageData::id( 0x85274 );
const MString NuiMayaImageData::typeName( "compositeImageData" );

/* override */
MStatus NuiMayaImageData::readASCII( const MArgList& argList, unsigned& index )
	//
	// Description
	//    ASCII file input method.
	//
{

	return MS::kSuccess;
}

/* override */
MStatus NuiMayaImageData::readBinary( istream& /*in*/, unsigned /*length*/ )
	//
	// Description
	//     NOT IMPLEMENTED
	//
{
	return MS::kSuccess;
}

/* override */
MStatus NuiMayaImageData::writeASCII( ostream& out )
	//
	// Description
	//    ASCII file output method.
	//
{

	return MS::kSuccess;
}

/* override */
MStatus NuiMayaImageData::writeBinary( ostream& /*out*/ )
	//
	// Description
	//    NOT IMPLEMENTED
	//
{
	return MS::kSuccess;
}

/* override */
void NuiMayaImageData::copy ( const MPxData& other )
{
	m_pImageData = (((const NuiMayaImageData &)other).m_pImageData);
}

/* override */
MTypeId NuiMayaImageData::typeId() const
	//
	// Description
	//    Binary tag used to identify this kind of data
	//
{
	return NuiMayaImageData::id;
}

/* override */
MString NuiMayaImageData::name() const
	//
	// Description
	//    String name used to identify this kind of data
	//
{
	return NuiMayaImageData::typeName;
}

void * NuiMayaImageData::creator()
{
	return new NuiMayaImageData;
}