#pragma once

#include "Shape/NuiSkeletonJoints.h"

#include <maya/MPxData.h>
#include <maya/MTypeId.h>
#include <maya/MString.h>

#include <memory>

class NuiMayaSkeletonData : public MPxData
{
public:
	//////////////////////////////////////////////////////////////////
	//
	// Overrides from MPxData
	//
	//////////////////////////////////////////////////////////////////

	NuiMayaSkeletonData();
	virtual ~NuiMayaSkeletonData();

	virtual MStatus			readASCII( const MArgList& argList, unsigned& idx );
	virtual MStatus			readBinary( istream& in, unsigned length );
	virtual MStatus			writeASCII( ostream& out );
	virtual MStatus			writeBinary( ostream& out );

	virtual	void			copy ( const MPxData& );

	virtual MTypeId         typeId() const;
	virtual MString         name() const;

	static void * creator();

public:
	static const MString	typeName;
	static const MTypeId	id;

	// This is the geometry our data will pass though the DG
	//
	std::shared_ptr<NuiSkeletonJoints>		m_pSkeletonData;
};