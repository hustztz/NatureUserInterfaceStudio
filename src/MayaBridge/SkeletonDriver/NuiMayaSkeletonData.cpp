#include "NuiMayaSkeletonData.h"

const MTypeId NuiMayaSkeletonData::id( 0x80774 );
const MString NuiMayaSkeletonData::typeName( "skeletonData" );

NuiMayaSkeletonData::NuiMayaSkeletonData()
	: m_pSkeletonData(new NuiSkeletonJoints)
{
}

NuiMayaSkeletonData::~NuiMayaSkeletonData()
{
}

/* override */
MStatus NuiMayaSkeletonData::readASCII( const MArgList& argList, unsigned& index )
//
// Description
//    ASCII file input method.
//
{

	return MS::kSuccess;
}

/* override */
MStatus NuiMayaSkeletonData::readBinary( istream& /*in*/, unsigned /*length*/ )
//
// Description
//     NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
MStatus NuiMayaSkeletonData::writeASCII( ostream& out )
//
// Description
//    ASCII file output method.
//
{

	return MS::kSuccess;
}

/* override */
MStatus NuiMayaSkeletonData::writeBinary( ostream& /*out*/ )
//
// Description
//    NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
void NuiMayaSkeletonData::copy ( const MPxData& other )
{
	m_pSkeletonData = ((const NuiMayaSkeletonData &)other).m_pSkeletonData;
}

/* override */
MTypeId NuiMayaSkeletonData::typeId() const
//
// Description
//    Binary tag used to identify this kind of data
//
{
	return NuiMayaSkeletonData::id;
}

/* override */
MString NuiMayaSkeletonData::name() const
//
// Description
//    String name used to identify this kind of data
//
{
	return NuiMayaSkeletonData::typeName;
}

void * NuiMayaSkeletonData::creator()
{
	return new NuiMayaSkeletonData;
}