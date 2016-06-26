#include <maya/MIOStream.h>
#include "NuiMayaFacialModelData.h"

const MTypeId NuiMayaFacialModelData::id( 0x85774 );
const MString NuiMayaFacialModelData::typeName( "faceTrackingData" );

NuiMayaFacialModelData::NuiMayaFacialModelData() : m_pFacialModel(NULL)
{
	m_pFacialModel = new NuiFacialModel;
}

NuiMayaFacialModelData::~NuiMayaFacialModelData()
{
	if ( NULL != m_pFacialModel ) {
		delete m_pFacialModel;
		m_pFacialModel = NULL;
	}
}

/* override */
MStatus NuiMayaFacialModelData::readASCII( const MArgList& argList, unsigned& index )
//
// Description
//    ASCII file input method.
//
{

	return MS::kSuccess;
}

/* override */
MStatus NuiMayaFacialModelData::readBinary( istream& /*in*/, unsigned /*length*/ )
//
// Description
//     NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
MStatus NuiMayaFacialModelData::writeASCII( ostream& out )
//
// Description
//    ASCII file output method.
//
{

	return MS::kSuccess;
}

/* override */
MStatus NuiMayaFacialModelData::writeBinary( ostream& /*out*/ )
//
// Description
//    NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
void NuiMayaFacialModelData::copy ( const MPxData& other )
{
	*m_pFacialModel = *((const NuiMayaFacialModelData &)other).m_pFacialModel;
}

/* override */
MTypeId NuiMayaFacialModelData::typeId() const
//
// Description
//    Binary tag used to identify this kind of data
//
{
	return NuiMayaFacialModelData::id;
}

/* override */
MString NuiMayaFacialModelData::name() const
//
// Description
//    String name used to identify this kind of data
//
{
	return NuiMayaFacialModelData::typeName;
}

void * NuiMayaFacialModelData::creator()
{
	return new NuiMayaFacialModelData;
}