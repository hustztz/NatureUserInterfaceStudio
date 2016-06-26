#pragma once

#include "Shape/NuiMeshShape.h"

#include <memory>

#include <maya/MPxGeometryData.h>
#include <maya/MTypeId.h>
#include <maya/MString.h>


class NuiMayaMeshData : public MPxData
{
public:
	//////////////////////////////////////////////////////////////////
	//
	// Overrides from MPxData
	//
	//////////////////////////////////////////////////////////////////

	NuiMayaMeshData() {}
	virtual ~NuiMayaMeshData() {}

	virtual MStatus			readASCII( const MArgList& argList, unsigned& idx );
	virtual MStatus			readBinary( istream& in, unsigned length );
	virtual MStatus			writeASCII( ostream& out );
	virtual MStatus			writeBinary( ostream& out );

	virtual	void			copy ( const MPxData& );

	virtual MTypeId         typeId() const;
	virtual MString         name() const;

	//////////////////////////////////////////////////////////////////
	//
	// Helper methods
	//
	//////////////////////////////////////////////////////////////////

	MStatus					readVerticesASCII( const MArgList&, unsigned& );

	MStatus					writeVerticesASCII( ostream& out );

	std::shared_ptr<NuiMeshShape>& data() { return m_pData; }
	void setData(std::shared_ptr<NuiMeshShape>& data) { m_pData = data; }

	// Find a spline data from the specified plug
	static std::shared_ptr<NuiMeshShape> findData(const MPlug& plug);
	static std::shared_ptr<NuiMeshShape> findData(const MObject& object, const MObject& attribute);
	static std::shared_ptr<NuiMeshShape> findDataFromXgmData(const MObject& object);

	static MObject createSharedData(const MObject& data);

	static void * creator();

public:
	static const MString typeName;
	static const MTypeId id;

private:
	// This is the geometry our data will pass though the DG
	//
	std::shared_ptr<NuiMeshShape>	m_pData;
};