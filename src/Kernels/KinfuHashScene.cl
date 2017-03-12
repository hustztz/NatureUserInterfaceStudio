#include "KinfuHashUtils.cl"

__kernel void reset_voxel_block_kernel(
			__global struct NuiKinfuVoxel*	d_SDFBlocks,
			__global int*					d_allocationList
        )
{
    const uint gidx = get_global_id(0);

	d_allocationList[gidx] = gidx;

	for (uint i = 0; i < SDF_BLOCK_SIZE3; i++)
	{
		d_SDFBlocks[gidx+i].weight = 0;
		d_SDFBlocks[gidx+i].sdf = 0;
		d_SDFBlocks[gidx+i].color[0] = d_SDFBlocks[gidx+i].color[1] = d_SDFBlocks[gidx+i].color[2] = 0;
	}
}

__kernel void reset_hash_entry_kernel(
            __global struct NuiKinfuHashEntry*	d_hash
        )
{
    const uint gidx = get_global_id(0);

	d_hash[gidx].ptr = FREE_ENTRY;
	d_hash[gidx].offset = 0;
	d_hash[gidx].pos[0] = d_hash[gidx].pos[1] = d_hash[gidx].pos[2] = 0;
}

__kernel void reset_hash_excess_allocation_kernel(
            __global int*				d_excessAllocation
        )
{
    const uint gidx = get_global_id(0);

	d_excessAllocation[gidx] = gidx;
}

__kernel void reset_visible_entrys_kernel(
            __global int*			d_visibleEntryIDs,
			__global uchar*			d_entriesVisibleType
        )
{
    const uint gidx = get_global_id(0);

	d_entriesVisibleType[ d_visibleEntryIDs[gidx] ] = 3;
}

__kernel void buildHashAllocAndVisibleType_kernel(
            __global	float*	depths,
			__constant	struct	NuiCLCameraParams* cameraParams,
			__global	struct	NuiCLRigidTransform* rigidTransform,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
			__global	uchar*	d_entriesVisibleType,
			__global	uchar*	d_entriesAllocType,
			__global	short3*	d_blockCoords,
			const		float	truncScale,
			const		float	oneOverVoxelSize
        )
{
    const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
	const uint gsizex = get_global_size(0);

	const uint id = mul24(gidy, gsizex) + gidx;
	float Dp = vload(id, depths);
	if(isnan(Dp))
		return;

	float3 pt_camera = kinectDepthToSkeleton(gidx, gidy, Dp, cameraParams);
	float norm = fast_length(pt_camera);

	float3 rayMin = transform( (pt_camera * (1.0f - truncScale/norm)), rigidTransform) * oneOverVoxelSize;
	float3 rayMax = transform( (pt_camera * (1.0f + truncScale/norm)), rigidTransform) * oneOverVoxelSize;
	float3 rayDir = rayMax - rayMin;
	norm = fast_length(rayDir);
	int numSteps = ceil(2.0f*norm);
	rayDir /= convert_float(numSteps - 1);

	//add neighbouring blocks
	for (int i = 0; i < numSteps; i++)
	{
		short3 blockPos = worldToVoxelPos(rayMin);
		uint hashIdx = computeHashIndex(blockPos);

		//check if hash table contains entry
		bool isFound = false;

		struct NuiKinfuHashEntry hashEntry = d_hashEntry[hashIdx];
		if(hashEntry.pos[0] == blockPos.x && hashEntry.pos[1] == blockPos.y && hashEntry.pos[2] == blockPos.z &&
			hashEntry.ptr >= -1)
		{
			//entry has been streamed out but is visible or in memory and visible
			d_entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;
			isFound = true;
		}

		if (!isFound)
		{
			bool isExcess = false;
			if (hashEntry.ptr >= -1) //seach excess list only if there is no room in ordered part
			{
				while (hashEntry.offset >= 1)
				{
					hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
					hashEntry = d_hashEntry[hashIdx];

					if (hashEntry.pos[0] == blockPos.x && hashEntry.pos[1] == blockPos.y && hashEntry.pos[2] == blockPos.z &&
						hashEntry.ptr >= -1)
					{
						//entry has been streamed out but is visible or in memory and visible
						d_entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;

						isFound = true;
						break;
					}
				}

				isExcess = true;
			}

			if (!isFound) //still not found
			{
				d_entriesAllocType[hashIdx] = isExcess ? 2 : 1; //needs allocation
				d_entriesVisibleType[hashIdx] = 1; //new entry is visible

				d_blockCoords[hashIdx] = blockPos;
			}
		}

		rayMin += rayDir;
	}
}

__kernel void allocateVoxelBlocksList_kernel(
			__global	uchar*			d_entriesAllocType,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
			__global	int*			d_excessAllocation,
			__global volatile int*		d_lastFreeExcessListId,
			__global	int*			d_allocationList,
			__global volatile int*		d_lastFreeVoxelBlockId,
			__global	short3*			d_blockCoords,
			__global	uchar*			d_entriesVisibleType
        )
{
	const uint gidx = get_global_id(0);
	int vbaIdx, exlIdx;
	uchar hashChangeType = d_entriesAllocType[gidx];
	switch (hashChangeType)
	{
	case 1: //needs allocation, fits in the ordered list
		vbaIdx = atomic_dec(d_lastFreeVoxelBlockId);

		if (vbaIdx >= 0) //there is room in the voxel block array
		{
			short3 pt_block_all = d_blockCoords[gidx];

			struct NuiKinfuHashEntry hashEntry;
			hashEntry.pos[0] = pt_block_all.x; hashEntry.pos[1] = pt_block_all.y; hashEntry.pos[2] = pt_block_all.z;
			hashEntry.ptr = d_allocationList[vbaIdx];
			hashEntry.offset = 0;

			d_hashEntry[gidx] = hashEntry;
		}

		break;
	case 2: //needs allocation in the excess list
		vbaIdx = atomic_dec(d_lastFreeVoxelBlockId);
		exlIdx = atomic_dec(d_lastFreeExcessListId);

		if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
		{
			short3 pt_block_all = d_blockCoords[gidx];

			struct NuiKinfuHashEntry hashEntry;
			hashEntry.pos[0] = pt_block_all.x; hashEntry.pos[1] = pt_block_all.y; hashEntry.pos[2] = pt_block_all.z;
			hashEntry.ptr = d_allocationList[vbaIdx];
			hashEntry.offset = 0;

			int exlOffset = d_excessAllocation[exlIdx];

			d_hashEntry[gidx].offset = exlOffset + 1; //connect to child

			d_hashEntry[SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list

			d_entriesVisibleType[SDF_BUCKET_NUM + exlOffset] = 1; //make child visible and in memory
		}

		break;
	}
}

__kernel void buildVisibleList_kernel(
			__global	uchar*			d_entriesVisibleType,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
        )
{
	const uint gidx = get_global_id(0);
	uchar hashVisibleType = d_entriesVisibleType[gidx];
	const struct NuiKinfuHashEntry& hashEntry = d_hashEntry[gidx];
	if (hashVisibleType == 3)
	{


		d_entriesVisibleType[gidx] = hashVisibleType;
	}

	if (hashVisibleType > 0) shouldPrefix = true;
}

__kernel void buildVisibleEnlargedList_kernel(
			__global	uchar*			d_entriesVisibleType,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
        )
{
	const uint gidx = get_global_id(0);
	uchar hashVisibleType = d_entriesVisibleType[gidx];
	const struct NuiKinfuHashEntry& hashEntry = d_hashEntry[gidx];
	if (hashVisibleType == 3)
	{


		d_entriesVisibleType[gidx] = hashVisibleType;
	}

	if (hashVisibleType > 0) shouldPrefix = true;

	if (hashVisibleType > 0 && swapStates[gidx].state != 2) swapStates[gidx].state = 1;
}
