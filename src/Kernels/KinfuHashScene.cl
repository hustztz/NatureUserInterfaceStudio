#include "KinfuHashUtils.cl"
#include "prefixSum.cl"
#include "utils.cl"

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
			__global	uchar*	d_entriesVisibleType,
			__global	uchar*	d_entriesAllocType,
			__global	short3*	d_blockCoords,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
            __global	float*	depths,
			__global	struct	NuiCLRigidTransform* rigidTransform,
			__constant	struct	NuiCLCameraParams* cameraParams,
			const		float	truncScale,
			const		float	oneOverVoxelBlockSize
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
	const float norm = truncScale / fast_length(pt_camera);

	float3 rayMin = transform( (pt_camera * (1.0f - norm)), rigidTransform) * oneOverVoxelBlockSize;
	float3 rayMax = transform( (pt_camera * (1.0f + norm)), rigidTransform) * oneOverVoxelBlockSize;
	float3 rayDir = rayMax - rayMin;
	const float rayNorm = fast_length(rayDir);
	const int numSteps = ceil(2.0f*rayNorm);
	rayDir /= convert_float(numSteps - 1);

	//add neighbouring blocks
	for (int i = 0; i < numSteps; i++)
	{
		short3 blockPos = (short3)(floor(rayMin.x), floor(rayMin.y), floor(rayMin.z));
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

	int2 screenPos = convert_int2(cameraToKinectScreen(pt_buff, cameraParams) + (float2)(0.5f, 0.5f));
	return (screenPos.x >= 0 && screenPos.x < camParams.depthImageWidth && screenPos.y >= 0 && screenPos.y < camParams.depthImageHeight);
}

inline static bool checkBlockVisibility(
	const		float3						pt_image,
	const		float						factor,
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
	const struct NuiKinfuHashEntry hashEntry = d_hashEntry[gidx];
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
	const		float						factor,
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
			__global	uchar*						d_swapStates
        )
{
	const uint gidx = get_global_id(0);
	uchar hashVisibleType = d_entriesVisibleType[gidx];
	const struct NuiKinfuHashEntry hashEntry = d_hashEntry[gidx];
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

	if (hashVisibleType > 0 && d_swapStates[gidx] != 2)
		d_swapStates[gidx] = 1;
}

__kernel __attribute__((reqd_work_group_size(WORKGROUP_SIZE, 1, 1)))
void prefix_flag_scan1_kernel(
    __global uchar8 *d_Src,
    __global uint8 *d_Dst,
    __local uint *l_Data,
    uint size
	)
{
    //Load data
    uint8 idata8;
    uchar8 srcs = d_Src[get_global_id(0)];
    idata8.s0 = convert_uint(srcs.s0 > 0);
    idata8.s1 = convert_uint(srcs.s1 > 0);
    idata8.s2 = convert_uint(srcs.s2 > 0);
    idata8.s3 = convert_uint(srcs.s3 > 0);
    idata8.s4 = convert_uint(srcs.s4 > 0);
    idata8.s5 = convert_uint(srcs.s5 > 0);
    idata8.s6 = convert_uint(srcs.s6 > 0);
    idata8.s7 = convert_uint(srcs.s7 > 0);


    scanExclusiveLocal1(idata8, d_Dst, l_Data, size);
}

__kernel __attribute__((reqd_work_group_size(WORKGROUP_SIZE, 1, 1)))
void prefix_flag_scan2_kernel(
    __global uchar *d_Src,
    __global uint *d_Buf,
    __global uint *d_Dst,
    __local uint *l_Data,
    uint size
	)
{
    //Load top elements
    uint indata = convert_uint(d_Src[(8 * WORKGROUP_SIZE - 1) + (8 * WORKGROUP_SIZE) * get_global_id(0)] > 0);

	scanExclusiveLocal2(indata, d_Buf, d_Dst, l_Data, size);
}

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
			const		float					truncScale,
			const		uchar					integrationWeightMax
        )
{
	const uint gid = get_global_id(0);
	const int lid = get_local_id(0);
	
	const int entryID = d_visibleEntryIDs[gid / SDF_BLOCK_SIZE3];
	if(entryID < 0)
		return;
	const struct NuiKinfuHashEntry hashEntry = d_hashEntry[entryID];
	if (hashEntry.ptr < 0)
		return;
	
	const int blockId = mul24(hashEntry.ptr, SDF_BLOCK_SIZE3) + lid;
	if(integrationWeightMax == d_SDFBlocks[blockId].weight)
		return;

	int3 globalPose = (int3)(hashEntry.pos[0], hashEntry.pos[1], hashEntry.pos[2]) * SDF_BLOCK_SIZE;
	const int locIdx = (lid % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) % SDF_BLOCK_SIZE;
	const int locIdy = (lid % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE;
	const int locIdz = lid / (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE);
	float3 pt_model = (float3)(convert_float(globalPose.x+locIdx), convert_float(globalPose.y+locIdy), convert_float(globalPose.z+locIdz)) * virtualVoxelSize;
	float3 pt_camera = transformInverse( pt_model, rigidTransform );
	if (pt_camera.z <= 0) return;
	int2 pt_image = convert_int2(cameraToKinectScreen(pt_camera, cameraParams) + (float2)(0.5f, 0.5f));

	struct NuiCLCameraParams camParams = *cameraParams;
	if(pt_image.x < 0 || pt_image.x >= camParams.depthImageWidth || pt_image.y < 0 || pt_image.y >= camParams.depthImageHeight)
		return;

	const int screenId = mul24(pt_image.y, convert_int(camParams.depthImageWidth)) + pt_image.x;
	float depth = depths[screenId];
	if(isnan(depth))
		return;

	// check whether voxel needs updating
	float eta = depth - pt_camera.z;
	float sdf = eta / truncScale;
	if (eta < -truncScale)
		return;
	
	float oldF = convert_float(d_SDFBlocks[blockId].sdf) / 32767.0f;
	float oldW = convert_float(d_SDFBlocks[blockId].weight);

	float newF = min(1.0f, sdf);
	float newW = 1.0f;
	float combinedW = oldW + newW;

	// Update color voxel
	if (colors && (eta < truncScale) && (fabs(sdf) < 0.25f))
	{
		uchar4 color = colors[screenId];
		if(255 == color.w)
		{
			float oldColorR = convert_float(d_SDFBlocks[blockId].color[0]);
			float oldColorG = convert_float(d_SDFBlocks[blockId].color[1]);
			float oldColorB = convert_float(d_SDFBlocks[blockId].color[2]);

			float newColorR = (oldW * oldColorR + newW * convert_float(color.s0)) / combinedW;
			float newColorG = (oldW * oldColorG + newW * convert_float(color.s1)) / combinedW;
			float newColorB = (oldW * oldColorB + newW * convert_float(color.s2)) / combinedW;

			d_SDFBlocks[blockId].color[0] = convert_uchar( clamp(newColorR, 0.0f, 255.f) );
			d_SDFBlocks[blockId].color[1] = convert_uchar( clamp(newColorG, 0.0f, 255.f) );
			d_SDFBlocks[blockId].color[2] = convert_uchar( clamp(newColorB, 0.0f, 255.f) );
		}
	}

	newF = oldW * oldF + newW * newF;
	newF /= combinedW;

	d_SDFBlocks[blockId].sdf = convert_short(newF * 32767.0f);
	d_SDFBlocks[blockId].weight = min(convert_uchar(combinedW), integrationWeightMax);
}


__kernel void fetchHashScene_kernel(
            __global float3*					d_vmap,
			__global float4*					d_cmap,
			__global volatile int*				vertex_id,
			__global	int*					d_visibleEntryIDs,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
			__global struct NuiKinfuVoxel*		d_SDFBlocks,
			const		float					virtualVoxelSize,
			const		float					truncScale)
{
	const uint gidx = get_global_id(0);
	const int entryID = d_visibleEntryIDs[gidx];
	if(entryID < 0)
		return;
	const struct NuiKinfuHashEntry hashEntry = d_hashEntry[entryID];
	if (hashEntry.ptr < 0)
		return;

	for (int i = 0; i < SDF_BLOCK_SIZE3; i++)
	{
		const int blockId = mul24(hashEntry.ptr, SDF_BLOCK_SIZE3) + i;
		const struct NuiKinfuVoxel localVoxelBlock = d_SDFBlocks[blockId];
		if ( localVoxelBlock.weight > 0 && abs(localVoxelBlock.sdf) < 32767 * truncScale ){ //mu=0.02
			float3 pt;
			pt.z = (hashEntry.pos[2] * SDF_BLOCK_SIZE + (i / (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE) + 0.5f)) *virtualVoxelSize;
			pt.y = (hashEntry.pos[1] * SDF_BLOCK_SIZE + ((i % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE + 0.5f)) *virtualVoxelSize;
			pt.x = (hashEntry.pos[0] * SDF_BLOCK_SIZE + ((i % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) % SDF_BLOCK_SIZE + 0.5f)) *virtualVoxelSize;

			float4 color_value = (float4)(0.0f, 0.0f, 0.0f, 1.0f);
			if(d_cmap)
			{
				color_value.x = convert_float(localVoxelBlock.color[0]) /255.f;
				color_value.y = convert_float(localVoxelBlock.color[1]) /255.f;
				color_value.z = convert_float(localVoxelBlock.color[2]) /255.f;
				color_value.w = 1.0f;
			}

			int current_id = atomic_inc(vertex_id);
			if(current_id < MAX_OUTPUT_VERTEX_SIZE)
			{
				d_vmap[current_id] = pt;
				d_cmap[current_id] = color_value;
			}
		}
	}
}