#pragma once

#include "Shape/NuiFacialModel.h"

#include <maya/MPxData.h>
#include <maya/MTypeId.h>
#include <maya/MString.h>


class NuiMayaFacialModelData : public MPxData
{
public:
	//////////////////////////////////////////////////////////////////
	//
	// Overrides from MPxData
	//
	//////////////////////////////////////////////////////////////////

	NuiMayaFacialModelData();
	virtual ~NuiMayaFacialModelData();

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
	NuiFacialModel*			m_pFacialModel;
};