#pragma once

#include "Shape/NuiCLMappableData.h"

#include <memory>

#include <maya/MPxGeometryData.h>
#include <maya/MTypeId.h>
#include <maya/MString.h>


class NuiMayaMappableData : public MPxGeometryData
{
public:
	//////////////////////////////////////////////////////////////////
	//
	// Overrides from MPxData
	//
	//////////////////////////////////////////////////////////////////

	NuiMayaMappableData() {}
	virtual ~NuiMayaMappableData() {}

	virtual MStatus			readASCII( const MArgList& argList, unsigned& idx );
	virtual MStatus			readBinary( istream& in, unsigned length );
	virtual MStatus			writeASCII( ostream& out );
	virtual MStatus			writeBinary( ostream& out );

	virtual	void			copy ( const MPxData& );

	virtual MTypeId         typeId() const;
	virtual MString         name() const;

	//////////////////////////////////////////////////////////////////
	//
	// Overrides from MPxGeometryData
	//
	//////////////////////////////////////////////////////////////////

	virtual MPxGeometryIterator* iterator( MObjectArray & componentList,
		MObject & component,
		bool useComponents);
	virtual MPxGeometryIterator* iterator( MObjectArray & componentList,
		MObject & component,
		bool useComponents,
		bool world) const;

	virtual bool	updateCompleteVertexGroup( MObject & component ) const;

	//////////////////////////////////////////////////////////////////
	//
	// Helper methods
	//
	//////////////////////////////////////////////////////////////////

	MStatus					readVerticesASCII( const MArgList&, unsigned& );

	MStatus					writeVerticesASCII( ostream& out );

	std::shared_ptr<NuiCLMappableData>& data() { return m_pCLData; }
	void setData(std::shared_ptr<NuiCLMappableData>& data) { m_pCLData = data; }

	// Find a spline data from the specified plug
	static std::shared_ptr<NuiCLMappableData> findData(const MPlug& plug);
	static std::shared_ptr<NuiCLMappableData> findData(const MObject& object, const MObject& attribute);
	static std::shared_ptr<NuiCLMappableData> findDataFromXgmData(const MObject& object);

	static MObject createSharedData(const MObject& data);

	static void * creator();

public:
	static const MString typeName;
	static const MTypeId id;

private:
	// This is the geometry our data will pass though the DG
	//
	std::shared_ptr<NuiCLMappableData>	m_pCLData;
};