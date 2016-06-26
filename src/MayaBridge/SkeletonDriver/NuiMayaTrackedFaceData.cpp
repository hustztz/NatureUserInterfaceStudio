#include <maya/MIOStream.h>
#include "NuiMayaTrackedFaceData.h"

const MTypeId NuiMayaTrackedFaceData::id( 0x85774 );
const MString NuiMayaTrackedFaceData::typeName( "faceTrackingData" );

NuiMayaTrackedFaceData::NuiMayaTrackedFaceData() : m_pTrackedFace(NULL)
{
	m_pTrackedFace = new NuiTrackedFace;
}

NuiMayaTrackedFaceData::~NuiMayaTrackedFaceData()
{
	if ( NULL != m_pTrackedFace ) {
		delete m_pTrackedFace;
		m_pTrackedFace = NULL;
	}
}

/* override */
MStatus NuiMayaTrackedFaceData::readASCII( const MArgList& argList, unsigned& index )
//
// Description
//    ASCII file input method.
//
{

	return MS::kSuccess;
}

/* override */
MStatus NuiMayaTrackedFaceData::readBinary( istream& /*in*/, unsigned /*length*/ )
//
// Description
//     NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
MStatus NuiMayaTrackedFaceData::writeASCII( ostream& out )
//
// Description
//    ASCII file output method.
//
{

	return MS::kSuccess;
}

/* override */
MStatus NuiMayaTrackedFaceData::writeBinary( ostream& /*out*/ )
//
// Description
//    NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
void NuiMayaTrackedFaceData::copy ( const MPxData& other )
{
	*m_pTrackedFace = *((const NuiMayaTrackedFaceData &)other).m_pTrackedFace;
}

/* override */
MTypeId NuiMayaTrackedFaceData::typeId() const
//
// Description
//    Binary tag used to identify this kind of data
//
{
	return NuiMayaTrackedFaceData::id;
}

/* override */
MString NuiMayaTrackedFaceData::name() const
//
// Description
//    String name used to identify this kind of data
//
{
	return NuiMayaTrackedFaceData::typeName;
}

void * NuiMayaTrackedFaceData::creator()
{
	return new NuiMayaTrackedFaceData;
}