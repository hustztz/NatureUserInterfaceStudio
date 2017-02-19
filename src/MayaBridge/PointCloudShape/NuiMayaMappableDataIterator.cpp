///////////////////////////////////////////////////////////////////////////////
//
// pointCloudVertexIterator.cpp
//
///////////////////////////////////////////////////////////////////////////////

#include "NuiMayaMappableDataIterator.h"
#include <maya/MIOStream.h>
#include <maya/MPoint.h>

NuiMayaMappableDataIterator::NuiMayaMappableDataIterator( void * geom, MObjectArray & comps )
	: MPxGeometryIterator( geom, comps ),
	geomPtr( reinterpret_cast<NuiCLMappableData*>(geom) )
{
	reset();
}

NuiMayaMappableDataIterator::NuiMayaMappableDataIterator( void * geom, MObject & comps )
	: MPxGeometryIterator( geom, comps ),
	geomPtr( reinterpret_cast<NuiCLMappableData*>(geom) )
{
	reset();
}

/* override */
void NuiMayaMappableDataIterator::reset()
//
// Description
//
//  	
//   Resets the iterator to the start of the components so that another
//   pass over them may be made.
//
{
	MPxGeometryIterator::reset();
	setCurrentPoint( 0 );
	if ( NULL != geomPtr ) {
		int maxVertex = geomPtr->GetPositionNum();
		setMaxPoints( maxVertex );
	}
}

/* override */
MPoint NuiMayaMappableDataIterator::point() const
//
// Description
//
//    Returns the point for the current element in the iteration.
//    This is used by the transform tools for positioning the
//    manipulator in component mode. It is also used by deformers.	 
//
{
	MPoint pnt;
	if ( NULL != geomPtr ) {
		pnt = MPoint( geomPtr->GetPositionValue(index()) );
	}
	return pnt;
}

/* override */
void NuiMayaMappableDataIterator::setPoint( const MPoint & pnt ) const
//
// Description
//
//    Set the point for the current element in the iteration.
//    This is used by deformers.	 
//
{
	if ( NULL != geomPtr ) {
		geomPtr->SetPositionValue(index(), (float)pnt.x, (float)pnt.y, (float)pnt.z);
	}
}

/* override */
int NuiMayaMappableDataIterator::iteratorCount() const
//
// Description
//
//    Return the number of vertices in the iteration.
//    This is used by deformers such as smooth skinning
//
{
	return geomPtr ? geomPtr->GetPositionNum() : 0;
}

/* override */
bool NuiMayaMappableDataIterator::hasPoints() const
//
// Description
//
//    Returns true since the shape data has points.
//
{
	return true;
}

