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

inline static bool checkPointVisibility(
	const		float3						pt_image,
	__constant struct NuiCLCameraParams*	cameraParams,
	__global struct NuiCLRigidTransform*	rigidTransform
	)
{
	float3 pt_buff = transformInverse(pt_image, rigidTransform);
	if(_isnan3(pt_buff))
		return false;

	struct NuiCLCameraParams camParams = *cameraParams;

	uint2 screenPos = convert_uint2(cameraToKinectScreen(pt_buff, cameraParams) + (float2)(0.5f, 0.5f));
	return (screenPos.x >= 0 && screenPos.x < camParams.depthImageWidth && screenPos.y >= 0 && screenPos.y < camParams.depthImageHeight);
}

inline static bool checkBlockVisibility(
	const		float3						pt_image,
	const		float						factor;
	__constant struct NuiCLCameraParams*	cameraParams,
	__global struct NuiCLRigidTransform*	rigidTransform
	)
{
	float3 pt = pt_image;
	// 0 0 0
	if(checkPointVisibility(pt, cameraParams, rigidTransform))
		return true;

	// 0 0 1
	pt.z += factor;
	if(checkPointVisibility(pt, cameraParams, rigidTransform))
		return true;

	// 0 1 1
	pt.y += factor;
	if(checkPointVisibility(pt, cameraParams, rigidTransform))
		return true;

	// 1 1 1
	pt.x += factor;
	if(checkPointVisibility(pt, cameraParams, rigidTransform))
		return true;

	// 1 1 0 
	pt.z -= factor;
	if(checkPointVisibility(pt, cameraParams, rigidTransform))
		return true;

	// 1 0 0 
	pt.y -= factor;
	if(checkPointVisibility(pt, cameraParams, rigidTransform))
		return true;

	// 0 1 0
	pt.x -= factor; pt.y += factor;
	if(checkPointVisibility(pt, cameraParams, rigidTransform))
		return true;

	// 1 0 1
	pt.x += factor; pt.y -= factor; pt.z += factor;
	if(checkPointVisibility(pt, cameraParams, rigidTransform))
		return true;

	return false;
}

__kernel void buildVisibleList_kernel(
			__global	uchar*						d_entriesVisibleType,
			__global struct NuiKinfuHashEntry*		d_hashEntry,
			__constant struct NuiCLCameraParams*	cameraParams,
			__global struct NuiCLRigidTransform*	rigidTransform,
			const		float						virtualVoxelSize
        )
{
	const uint gidx = get_global_id(0);
	uchar hashVisibleType = d_entriesVisibleType[gidx];
	const struct NuiKinfuHashEntry& hashEntry = d_hashEntry[gidx];
	if (hashVisibleType == 3)
	{
		float factor = convert_float(SDF_BLOCK_SIZE) * virtualVoxelSize;
		float3 pt_image = (float3)(
			convert_float(hashEntry.pos[0]) * factor,
			convert_float(hashEntry.pos[1]) * factor,
			convert_float(hashEntry.pos[2]) * factor
			);

		if (!checkBlockVisibility(pt_image, factor, cameraParams, rigidTransform))
			d_entriesVisibleType[gidx] = 0;
	}
}

inline static bool checkPointVisibilityEnlarged(
	const		float3						pt_image,
	__constant struct NuiCLCameraParams*	cameraParams,
	__global struct NuiCLRigidTransform*	rigidTransform
	)
{
	float3 pt_buff = transformInverse(pt_image, rigidTransform);
	if(_isnan3(pt_buff))
		return false;

	struct NuiCLCameraParams camParams = *cameraParams;
	uint4 lims = (uint4)(
		- camParams.depthImageWidth / 8,
		camParams.depthImageWidth + camParams.depthImageWidth / 8,
		- camParams.depthImageHeight / 8,
		camParams.depthImageHeight + camParams.depthImageHeight / 8
		);
	uint2 screenPos = convert_uint2(cameraToKinectScreen(pt_buff, cameraParams) + (float2)(0.5f, 0.5f));
	return (screenPos.x >= lims.x && screenPos.x < lims.y && screenPos.y >= lims.z && screenPos.y < lims.w);
}

inline static bool checkBlockVisibilityEnlarged(
	const		float3						pt_image,
	const		float						factor;
	__constant struct NuiCLCameraParams*	cameraParams,
	__global struct NuiCLRigidTransform*	rigidTransform
	)
{
	float3 pt = pt_image;
	// 0 0 0
	if(checkPointVisibilityEnlarged(pt, cameraParams, rigidTransform))
		return true;

	// 0 0 1
	pt.z += factor;
	if(checkPointVisibilityEnlarged(pt, cameraParams, rigidTransform))
		return true;

	// 0 1 1
	pt.y += factor;
	if(checkPointVisibilityEnlarged(pt, cameraParams, rigidTransform))
		return true;

	// 1 1 1
	pt.x += factor;
	if(checkPointVisibilityEnlarged(pt, cameraParams, rigidTransform))
		return true;

	// 1 1 0 
	pt.z -= factor;
	if(checkPointVisibilityEnlarged(pt, cameraParams, rigidTransform))
		return true;

	// 1 0 0 
	pt.y -= factor;
	if(checkPointVisibilityEnlarged(pt, cameraParams, rigidTransform))
		return true;

	// 0 1 0
	pt.x -= factor; pt.y += factor;
	if(checkPointVisibilityEnlarged(pt, cameraParams, rigidTransform))
		return true;

	// 1 0 1
	pt.x += factor; pt.y -= factor; pt.z += factor;
	if(checkPointVisibilityEnlarged(pt, cameraParams, rigidTransform))
		return true;

	return false;
}

__kernel void buildVisibleEnlargedList_kernel(
			__global	uchar*						d_entriesVisibleType,
			__global struct NuiKinfuHashEntry*		d_hashEntry,
			__constant struct NuiCLCameraParams*	cameraParams,
			__global struct NuiCLRigidTransform*	rigidTransform,
			const		float						virtualVoxelSize,
			__global struct NuiKinfuHashSwapState*	d_swapStates
        )
{
	const uint gidx = get_global_id(0);
	uchar hashVisibleType = d_entriesVisibleType[gidx];
	const struct NuiKinfuHashEntry& hashEntry = d_hashEntry[gidx];
	if (hashVisibleType == 3)
	{
		float factor = convert_float(SDF_BLOCK_SIZE) * virtualVoxelSize;
		float3 pt_image = (float3)(
			convert_float(hashEntry.pos[0]) * factor,
			convert_float(hashEntry.pos[1]) * factor,
			convert_float(hashEntry.pos[2]) * factor
			);

		if (!checkBlockVisibilityEnlarged(pt_image, factor, cameraParams, rigidTransform))
			d_entriesVisibleType[gidx] = 0;
	}

	if (hashVisibleType > 0 && d_swapStates[gidx].state != 2)
		d_swapStates[gidx].state = 1;
}

__kernel void compactifyVisibleType_kernel(
	__global	uchar*					d_entriesVisibleType,
	__global	uint*					d_entriesVisibleTypePrefix,
	__global	int*					d_visibleEntryIDs,
	)
{
	const int gidx = get_global_id(0);

	if( d_entriesVisibleType[gidx] == 1 )
	{
		uint prefixIdx = d_entriesVisibleTypePrefix[gidx];
		if(prefixIdx > 0)
			prefixIdx --;
		d_visibleEntryIDs[prefixIdx] = gidx;
	}
}

__kernel void reAllocateSwappedOutVoxelBlocks_kernel(
			__global	uchar*					d_entriesVisibleType,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
			__global volatile int*				d_lastFreeVoxelBlockId,
			__global	int*					d_allocationList
			)
{
	const uint gidx = get_global_id(0);
	struct NuiKinfuHashEntry& hashEntry = d_hashEntry[gidx];

	if (d_entriesVisibleType[gidx] > 0 && hashEntry.ptr == -1) //it is visible and has been previously allocated inside the hash, but deallocated from VBA
	{
		int vbaIdx = atomic_dec(d_lastFreeVoxelBlockId, 1);
		if (vbaIdx >= 0)
			hashEntry.ptr = d_allocationList[vbaIdx];
	}
}

__kernel void integrateIntoScene_kernel(
			__global	int*					d_visibleEntryIDs,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
			__global struct NuiKinfuVoxel*		d_SDFBlocks,
			__global	float*					depths,
			__global	uchar4*					colors,
			__constant struct NuiCLCameraParams* cameraParams,
			__global struct NuiCLRigidTransform* rigidTransform,
			const		float					virtualVoxelSize,
			const		float					truncation,
			const		uchar					integrationWeightMax
        )
{
	const uint gidx = get_global_id(0);
	const int entryID = d_visibleEntryIDs[gidx];
	if(entryID < 0)
		return;
	const struct NuiKinfuHashEntry& hashEntry = d_hashEntry[entryID];
	if (hashEntry.ptr < 0)
		return;
	
	const int lidx = get_local_id(0);
	const int lidy = get_local_id(1);
	const int lidz = get_local_id(2);
	
	const int locId = lidx + mul24(lidy, SDF_BLOCK_SIZE) + mul24(lidz, mul24(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE));
	struct NuiKinfuVoxel& localVoxelBlock = d_SDFBlocks[hashEntry.ptr * SDF_BLOCK_SIZE3 + locId];
	if(integrationWeightMax == localVoxelBlock.weight)
		return;

	int3 globalPose = (int3)(hashEntry.pos[0], hashEntry.pos[1], hashEntry.pos[2]) * SDF_BLOCK_SIZE;
	float3 pt_model = (float3)(convert_float(globalPose.x+lidx), convert_float(globalPose.y+lidy), convert_float(globalPose.z+lidz)) * virtualVoxelSize;
	float3 pCamera = transformInverse( pt_model, rigidTransform );
	if (pt_camera.z <= 0) return;
	uint2 pt_image = convert_uint2(cameraToKinectScreen(pCamera, cameraParams) + (float2)(0.5f, 0.5f));

	struct NuiCLCameraParams camParams = *cameraParams;
	if(pt_image.x < 0 || pt_image.x >= camParams.depthImageWidth || pt_image.y < 0 || pt_image.y >= camParams.depthImageHeight)
		return;

	const uint screenId = mul24(pt_image.y, camParams.depthImageWidth) + pt_image.x;
	float depth = depths[screenId];
	if(isnan(depth))
		return;

	// check whether voxel needs updating
	float eta = depth - pt_camera.z;
	float sdf = eta / truncation;
	if (eta < -truncation)
		return;
	
	float oldF = convert_float(localVoxelBlock.sdf) / 32767.0f;
	float oldW = convert_float(localVoxelBlock.weight);

	float newF = min(1.0f, sdf);
	float newW = 1.0f;
	float combinedW = oldW + newW;

	// Update color voxel
	if (colors && (eta < truncation) && (fabs(sdf) < 0.25f))
	{
		uchar4 color = colors[screenId];
		if(255 == color.w)
		{
			float oldColorR = convert_short(localVoxelBlock.color[0]);
			float oldColorG = convert_short(localVoxelBlock.color[1]);
			float oldColorB = convert_short(localVoxelBlock.color[2]);

			float newColorR = (oldW * oldColorR + newW * convert_short(color.r)) / combinedW;
			float newColorG = (oldW * oldColorG + newW * convert_short(color.g)) / combinedW;
			float newColorB = (oldW * oldColorB + newW * convert_short(color.b)) / combinedW;

			localVoxelBlock.color[0] = convert_uchar( clamp(newColorR, 0.0f, 255.f) );
			localVoxelBlock.color[1] = convert_uchar( clamp(newColorG, 0.0f, 255.f) );
			localVoxelBlock.color[2] = convert_uchar( clamp(newColorB, 0.0f, 255.f) );
		}
	}

	newF = oldW * oldF + newW * newF;
	newF /= combinedW;

	localVoxelBlock.sdf = convert_short(newF * 32767.0f);
	localVoxelBlock.weight = min(convert_uchar(combinedW), integrationWeightMax);
}
