#pragma once
////////////////////////////////////////////////////////////////////////////////
//
// Point iterator for control-point based geometry
//
// This is used by the translate/rotate/scale manipulators to 
// determine where to place the manipulator when components are
// selected.
//
// As well deformers use this class to deform points of the shape.
//
////////////////////////////////////////////////////////////////////////////////

#include <maya/MPxGeometryIterator.h>

#include "Shape/NuiCLMappableData.h"

class NuiMayaImageCLDataIterator : public MPxGeometryIterator
{
public:
	NuiMayaImageCLDataIterator( void * userGeometry, MObjectArray & components );
	NuiMayaImageCLDataIterator( void * userGeometry, MObject & components );

	//////////////////////////////////////////////////////////
	//
	// Overrides
	//
	//////////////////////////////////////////////////////////

	virtual void		reset();
	virtual MPoint		point() const;
	virtual void		setPoint( const MPoint & ) const;
	virtual int			iteratorCount() const;
	virtual bool        hasPoints() const;

public:
	NuiCLMappableData*	geomPtr;
};
