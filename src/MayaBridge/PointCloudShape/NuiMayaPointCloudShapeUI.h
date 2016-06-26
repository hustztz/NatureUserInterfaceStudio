#pragma once

#include <maya/MPxSurfaceShapeUI.h>

class NuiMayaPointCloudShapeUI : public MPxSurfaceShapeUI
{
public:

	static void* creator();

	NuiMayaPointCloudShapeUI();
	virtual ~NuiMayaPointCloudShapeUI();

	/////////////////////////////////////////////////////////////////////
	//
	// Overrides
	//
	/////////////////////////////////////////////////////////////////////

	// Puts draw request on the draw queue
	//
	virtual void	getDrawRequests( const MDrawInfo & info,
		bool objectAndActiveOnly,
		MDrawRequestQueue & requests ){}

	// Main draw routine. Gets called by maya with draw requests.
	//
	virtual void	draw( const MDrawRequest & request,
		M3dView & view ) const{}

	// Main draw routine for UV editor. This is called by maya when the 
	// shape is selected and the UV texture window is visible. 
	// 
	virtual void	drawUV( M3dView &view, const MTextureEditorDrawInfo & ) const{}
	virtual bool	canDrawUV() const{return false;}

	// Main selection routine
	//
	virtual bool	select( MSelectInfo &selectInfo,
		MSelectionList &selectionList,
		MPointArray &worldSpaceSelectPts ) const;

	/////////////////////////////////////////////////////////////////////
	//
	// Helper routines
	//
	/////////////////////////////////////////////////////////////////////
	bool 			selectVertices( MSelectInfo &selectInfo,
		MSelectionList &selectionList,
		MPointArray &worldSpaceSelectPts ) const;

private:
	// Prohibited and not implemented.
	NuiMayaPointCloudShapeUI(const NuiMayaPointCloudShapeUI& obj);
	const NuiMayaPointCloudShapeUI& operator=(const NuiMayaPointCloudShapeUI& obj);
};