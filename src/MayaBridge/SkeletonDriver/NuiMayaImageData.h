#pragma once

#include "Frame/NuiCompositeFrame.h"

#include <memory>

#include <maya/MPxData.h>
#include <maya/MTypeId.h>
#include <maya/MString.h>


class NuiMayaImageData : public MPxData
{
public:
	//////////////////////////////////////////////////////////////////
	//
	// Overrides from MPxData
	//
	//////////////////////////////////////////////////////////////////

	NuiMayaImageData() {}
	virtual ~NuiMayaImageData() {}

	virtual MStatus			readASCII( const MArgList& argList, unsigned& idx );
	virtual MStatus			readBinary( istream& in, unsigned length );
	virtual MStatus			writeASCII( ostream& out );
	virtual MStatus			writeBinary( ostream& out );

	virtual	void			copy ( const MPxData& );

	virtual MTypeId         typeId() const;
	virtual MString         name() const;

	static MObject createSharedData(const MObject& data);

	static void * creator();

public:
	static const MString typeName;
	static const MTypeId id;

private:
	// This is the geometry our data will pass though the DG
	//
	std::shared_ptr<NuiCompositeFrame>	m_pImageData;
};