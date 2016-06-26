#include <maya/MIOStream.h>
#include "NuiMayaGestureData.h"

const MTypeId NuiMayaGestureData::id( 0x81474 );
const MString NuiMayaGestureData::typeName( "gestureData" );

NuiMayaGestureData::NuiMayaGestureData() : m_pGestureResult(NULL)
{
	m_pGestureResult = new NuiGestureResult;
}

NuiMayaGestureData::~NuiMayaGestureData()
{
	if ( NULL != m_pGestureResult ) {
		delete m_pGestureResult;
		m_pGestureResult = NULL;
	}
}

/* override */
MStatus NuiMayaGestureData::readASCII( const MArgList& argList, unsigned& index )
//
// Description
//    ASCII file input method.
//
{

	return MS::kSuccess;
}

/* override */
MStatus NuiMayaGestureData::readBinary( istream& /*in*/, unsigned /*length*/ )
//
// Description
//     NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
MStatus NuiMayaGestureData::writeASCII( ostream& out )
//
// Description
//    ASCII file output method.
//
{

	return MS::kSuccess;
}

/* override */
MStatus NuiMayaGestureData::writeBinary( ostream& /*out*/ )
//
// Description
//    NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
void NuiMayaGestureData::copy ( const MPxData& other )
{
	*m_pGestureResult = *((const NuiMayaGestureData &)other).m_pGestureResult;
}

/* override */
MTypeId NuiMayaGestureData::typeId() const
//
// Description
//    Binary tag used to identify this kind of data
//
{
	return NuiMayaGestureData::id;
}

/* override */
MString NuiMayaGestureData::name() const
//
// Description
//    String name used to identify this kind of data
//
{
	return NuiMayaGestureData::typeName;
}

void * NuiMayaGestureData::creator()
{
	return new NuiMayaGestureData;
}