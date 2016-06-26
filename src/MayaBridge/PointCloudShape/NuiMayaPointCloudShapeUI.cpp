#include "NuiMayaPointCloudShapeUI.h"
#include "NuiMayaPointCloudShape.h"
#include "Shape/NuiDevicePointCloud.h"

#include <maya/MDagPath.h>
#include <maya/MMatrix.h>
#include <maya/MFnSingleIndexedComponent.h>

void* NuiMayaPointCloudShapeUI::creator()
{
	return new NuiMayaPointCloudShapeUI;
}

NuiMayaPointCloudShapeUI::NuiMayaPointCloudShapeUI()
{}

NuiMayaPointCloudShapeUI::~NuiMayaPointCloudShapeUI()
{}

bool NuiMayaPointCloudShapeUI::selectVertices( MSelectInfo &selectInfo,
										  MSelectionList &selectionList,
										  MPointArray &worldSpaceSelectPts ) const
//
// Description:
//
//     Vertex selection.
//
// Arguments:
//
//     selectInfo           - the selection state information
//     selectionList        - the list of selected items to add to
//     worldSpaceSelectPts  -
//
{
	bool selected = false;
	M3dView view = selectInfo.view();

	MPoint 		xformedPoint;
	MPoint 		selectionPoint;
	double		z,previousZ = 0.0;
	int			closestPointVertexIndex = -1;

	const MDagPath & path = selectInfo.multiPath();

	// Create a component that will store the selected vertices
	//
	MFnSingleIndexedComponent fnComponent;
	MObject surfaceComponent = fnComponent.create( MFn::kMeshVertComponent );
	int vertexIndex;

	// if the user did a single mouse click and we find > 1 selection
	// we will use the alignmentMatrix to find out which is the closest
	//
	MMatrix	alignmentMatrix;
	MPoint singlePoint; 
	bool singleSelection = selectInfo.singleSelection();
	if( singleSelection ) {
		alignmentMatrix = selectInfo.getAlignmentMatrix();
	}

	// Get the geometry information
	//
	NuiMayaPointCloudShape* meshNode = (NuiMayaPointCloudShape*)surfaceShape();
	NuiDevicePointCloud * geom = meshNode->meshGeom();

	// Loop through all vertices of the mesh and
	// see if they lie withing the selection area
	//
	int numVertices = geom->GetPointsNum();
	for ( vertexIndex=0; vertexIndex<numVertices; vertexIndex++ )
	{
		NuiDevicePoint* pt = geom->AccessPoint(vertexIndex);
		if( !pt )
			continue;

		MPoint currentPoint = MPoint( pt->fVertex.data );

		// Sets OpenGL's render mode to select and stores
		// selected items in a pick buffer
		//
		view.beginSelect();

		glBegin( GL_POINTS );
		glVertex3f( (float)currentPoint[0], 
			(float)currentPoint[1], 
			(float)currentPoint[2] );
		glEnd();

		if ( view.endSelect() > 0 )	// Hit count > 0
		{
			selected = true;

			if ( singleSelection ) {
				xformedPoint = currentPoint;
				xformedPoint.homogenize();
				xformedPoint*= alignmentMatrix;
				z = xformedPoint.z;
				if ( closestPointVertexIndex < 0 || z > previousZ ) {
					closestPointVertexIndex = vertexIndex;
					singlePoint = currentPoint;
					previousZ = z;
				}
			} else {
				// multiple selection, store all elements
				//
				fnComponent.addElement( vertexIndex );
			}
		}
	}

	// If single selection, insert the closest point into the array
	//
	if ( selected && selectInfo.singleSelection() ) {
		fnComponent.addElement(closestPointVertexIndex);

		// need to get world space position for this vertex
		//
		selectionPoint = singlePoint;
		selectionPoint *= path.inclusiveMatrix();
	}

	// Add the selected component to the selection list
	//
	if ( selected ) {
		MSelectionList selectionItem;
		selectionItem.add( path, surfaceComponent );

		MSelectionMask mask( MSelectionMask::kSelectComponentsMask );
		selectInfo.addSelection(
			selectionItem, selectionPoint,
			selectionList, worldSpaceSelectPts,
			mask, true );
	}

	return selected;
}

/* override */
bool NuiMayaPointCloudShapeUI::select( MSelectInfo &selectInfo, MSelectionList &selectionList,
								  MPointArray &worldSpaceSelectPts ) const
//
// Description:
//
//     Main selection routine
//
// Arguments:
//
//     selectInfo           - the selection state information
//     selectionList        - the list of selected items to add to
//     worldSpaceSelectPts  -
//
{
	bool selected = false;
	bool componentSelected = false;
	bool hilited = false;

	hilited = (selectInfo.displayStatus() == M3dView::kHilite);
	if ( hilited ) {
		componentSelected = selectVertices( selectInfo, selectionList,
			worldSpaceSelectPts );
		selected = selected || componentSelected;
	}

	if ( !selected ) {

		NuiMayaPointCloudShape* meshNode = (NuiMayaPointCloudShape*)surfaceShape();

		// NOTE: If the geometry has an intersect routine it should
		// be called here with the selection ray to determine if the
		// the object was selected.

		selected = true;
		MSelectionMask priorityMask( MSelectionMask::kSelectNurbsSurfaces );
		MSelectionList item;
		item.add( selectInfo.selectPath() );
		MPoint xformedPt;
		if ( selectInfo.singleSelection() ) {
			MPoint center = meshNode->boundingBox().center();
			xformedPt = center;
			xformedPt *= selectInfo.selectPath().inclusiveMatrix();
		}

		selectInfo.addSelection( item, xformedPt, selectionList,
			worldSpaceSelectPts, priorityMask, false );
	}

	return selected;
}