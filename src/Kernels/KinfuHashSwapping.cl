#include "KinfuHashUtils.cl"
#include "prefixSum.cl"
#include "utils.cl"


__kernel void reAllocateSwappedOutVoxelBlocks_kernel(
			__global	uchar*					d_entriesVisibleType,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
			__global volatile int*				d_lastFreeVoxelBlockId,
			__global	int*					d_allocationList
			)
{
	const uint gidx = get_global_id(0);
	struct NuiKinfuHashEntry hashEntry = d_hashEntry[gidx];

	if (d_entriesVisibleType[gidx] > 0 && hashEntry.ptr == -1) //it is visible and has been previously allocated inside the hash, but deallocated from VBA
	{
		int vbaIdx = atomic_dec(d_lastFreeVoxelBlockId);
		if (vbaIdx >= 0)
			d_hashEntry[gidx].ptr = d_allocationList[vbaIdx];
	}
}


__kernel __attribute__((reqd_work_group_size(WORKGROUP_SIZE, 1, 1)))
void prefix_swapin_scan1_kernel(
    __global uchar8 *d_swapStates,
    __global uint8 *d_Dst,
    __local uint *l_Data,
    uint size
	)
{
    //Load data
    uint8 idata8;
    uchar8 srcs = d_swapStates[get_global_id(0)];
    idata8.s0 = convert_uint(srcs.s0 == 1);
    idata8.s1 = convert_uint(srcs.s1 == 1);
    idata8.s2 = convert_uint(srcs.s2 == 1);
    idata8.s3 = convert_uint(srcs.s3 == 1);
    idata8.s4 = convert_uint(srcs.s4 == 1);
    idata8.s5 = convert_uint(srcs.s5 == 1);
    idata8.s6 = convert_uint(srcs.s6 == 1);
    idata8.s7 = convert_uint(srcs.s7 == 1);


    scanExclusiveLocal1(idata8, d_Dst, l_Data, size);
}

__kernel __attribute__((reqd_work_group_size(WORKGROUP_SIZE, 1, 1)))
void prefix_swapin_scan2_kernel(
    __global uchar *d_swapStates,
    __global uint *d_Buf,
    __global uint *d_Dst,
    __local uint *l_Data,
    uint size
	)
{
    //Load top elements
    uint indata = convert_uint(d_swapStates[(8 * WORKGROUP_SIZE - 1) + (8 * WORKGROUP_SIZE) * get_global_id(0)] == 1);

	scanExclusiveLocal2(indata, d_Buf, d_Dst, l_Data, size);
}

__kernel void integrateOldIntoActiveData_kernel(
			__global struct NuiKinfuHashEntry*	d_hashEntry,
			__global struct NuiKinfuVoxel*		d_localVoxelBlocks,
			__global	int*					d_neededEntryIDs,
			__global struct NuiKinfuVoxel*		d_syncedVoxelBlocks,
			__global	uchar*					d_swapStates,
			const		uchar					integrationWeightMax
        )
{
    const int gid = get_global_id(0);
	const int lid = get_local_id(0);

	const struct NuiKinfuVoxel syncedVoxelBlock = d_syncedVoxelBlocks[gid];

	const int entryID = d_neededEntryIDs[gid / SDF_BLOCK_SIZE3];
	if(entryID < 0)
		return;
	const struct NuiKinfuHashEntry hashEntry = d_hashEntry[entryID];
	if (hashEntry.ptr < 0)
		return;
	const int blockId = mul24(hashEntry.ptr, SDF_BLOCK_SIZE3) + lid;
	const struct NuiKinfuVoxel localVoxelBlock = d_localVoxelBlocks[blockId];

	float oldF = convert_float(syncedVoxelBlock.sdf) / 32767.0f;
	float oldW = convert_float(syncedVoxelBlock.weight);

	float newF = convert_float(localVoxelBlock.sdf) / 32767.0f;
	float newW = convert_float(localVoxelBlock.weight);
	float combinedW = oldW + newW;

	newF = oldW * oldF + newW * newF;
	newF /= combinedW;

	d_localVoxelBlocks[blockId].sdf = convert_short(newF * 32767.0f);
	d_localVoxelBlocks[blockId].weight = min(convert_uchar(combinedW), integrationWeightMax);

	
	// Update color voxel
	float newColorR = (oldW * convert_float(syncedVoxelBlock.color[0]) + newW * convert_float(localVoxelBlock.color[0])) / combinedW;
	float newColorG = (oldW * convert_float(syncedVoxelBlock.color[1]) + newW * convert_float(localVoxelBlock.color[1])) / combinedW;
	float newColorB = (oldW * convert_float(syncedVoxelBlock.color[2]) + newW * convert_float(localVoxelBlock.color[2])) / combinedW;

	d_localVoxelBlocks[blockId].color[0] = convert_uchar( clamp(newColorR, 0.0f, 255.f) );
	d_localVoxelBlocks[blockId].color[1] = convert_uchar( clamp(newColorG, 0.0f, 255.f) );
	d_localVoxelBlocks[blockId].color[2] = convert_uchar( clamp(newColorB, 0.0f, 255.f) );


	if(0 == lid)
		d_swapStates[entryID] = 2;
}

__kernel void buildSwapOutList_kernel(
			__global uchar*			d_flags,
			__global uchar*			d_swapStates,
            __global struct NuiKinfuHashEntry*	d_hash,
			__global uchar*			d_entriesVisibleType
        )
{
    const uint gid = get_global_id(0);

	bool bNeeded = (d_swapStates[gid] == 2) && (d_hash[gid].ptr > 0) && (d_entriesVisibleType[gid] == 0);
	d_flags[gid] = bNeeded;
}

__kernel void moveActiveDataToTransferBuffer_kernel(
			__global struct NuiKinfuVoxel*		d_syncedVoxelBlocks,
			__global struct NuiKinfuVoxel*		d_localVoxelBlocks,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
			__global	int*					d_neededEntryIDs,
			__global	bool*					d_hasSyncedDatas
        )
{
    const int gid = get_global_id(0);
	const int lid = get_local_id(0);
	const int bid = gid / SDF_BLOCK_SIZE3;
	const int entryID = d_neededEntryIDs[bid];
	if(entryID < 0)
		return;
	const struct NuiKinfuHashEntry hashEntry = d_hashEntry[entryID];
	if (hashEntry.ptr < 0)
		return;
	const int blockId = mul24(hashEntry.ptr, SDF_BLOCK_SIZE3) + lid;

	d_syncedVoxelBlocks[gid] = d_localVoxelBlocks[blockId];
	struct NuiKinfuVoxel initVoxel;
	initVoxel.sdf = 0;
	initVoxel.weight = 0;
	initVoxel.color[0] = initVoxel.color[1] = initVoxel.color[2] = 0;
	d_localVoxelBlocks[blockId] = initVoxel;

	if(lid == 0)
		d_hasSyncedDatas[bid] = true;
}

__kernel void cleanVoxelMemory_kernel(
			__global	int*					d_neededEntryIDs,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
			__global	int*					d_voxelAllocationList,
			__global uchar*						d_swapStates,
			__global volatile int*				d_lastFreeVoxelBlockId
        )
{
    const int gid = get_global_id(0);
	if(gid >= (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE))
		return;
	const int entryID = d_neededEntryIDs[gid];
	if(entryID < 0)
		return;

	d_swapStates[entryID] = 0;

	int vbaIdx = atomic_inc(d_lastFreeVoxelBlockId);
	if(vbaIdx < SDF_LOCAL_BLOCK_NUM - 1)
	{
		d_voxelAllocationList[vbaIdx + 1] = d_hashEntry[entryID].ptr;
		d_hashEntry[entryID].ptr = -1;
	}
}