#include "KinfuHashUtils.cl"


__kernel void projectAndSplitBlocks_kernel(
			__global	int*					d_visibleEntryIDs,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
			__global struct NuiKinfuVoxel*		d_SDFBlocks,
			__constant struct NuiCLCameraParams* cameraParams,
			__global struct NuiCLRigidTransform* rigidTransform,
			const		float					virtualVoxelSize
        )
{
	const uint gidx = get_global_id(0);
	const int entryID = d_visibleEntryIDs[gidx];
	if(entryID < 0)
		return;
	const struct NuiKinfuHashEntry& hashEntry = d_hashEntry[entryID];
	if (hashEntry.ptr < 0)
		return;
}