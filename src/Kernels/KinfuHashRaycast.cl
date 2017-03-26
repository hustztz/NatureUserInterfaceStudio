#include "KinfuHashUtils.cl"
#include "utils.cl"

__kernel void project_minmax_depths_kernel(
			__global	float*					d_minmaxData,
			__global	int*					d_visibleEntryIDs,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
			__constant struct NuiCLCameraParams* cameraParams,
			__global struct NuiCLRigidTransform* rigidTransform,
			const		float					virtualVoxelSize
        )
{
	const uint gidx = get_global_id(0);

	const int entryID = d_visibleEntryIDs[gidx];
	if(entryID < 0 && entryID >= (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE))
		return;
	const struct NuiKinfuHashEntry hashEntry = d_hashEntry[entryID];
	if (hashEntry.ptr < 0)
		return;
	
	struct NuiCLCameraParams camParams = *cameraParams;

	int2 upperLeft = (int2)(camParams.depthImageWidth, camParams.depthImageHeight);
	int2 lowerRight = (int2)(0, 0);
	float2 zRange = (float2)(FAR_AWAY, VERY_CLOSE);
	for (short corner = 0; corner < 8; ++corner)
	{
		// project all 8 corners down to 2D image
		short3 tmp;
		tmp.x = hashEntry.pos[0];
		tmp.y = hashEntry.pos[1];
		tmp.z = hashEntry.pos[2];
		tmp.x += (corner & 1) ? 1 : 0;
		tmp.y += (corner & 2) ? 1 : 0;
		tmp.z += (corner & 4) ? 1 : 0;

		float3 pt3d = convert_float3(tmp)*SDF_BLOCK_SIZE*virtualVoxelSize;
		pt3d = transformInverse( pt3d, rigidTransform );
		if (_isnan3(pt3d) || pt3d.z < 1e-6) continue;

		float2 pt2d = cameraToKinectScreen(pt3d, cameraParams);
		// remember bounding box, zmin and zmax
		upperLeft.x = min(upperLeft.x, convert_int(floor(pt2d.x)));
		lowerRight.x = max(lowerRight.x, convert_int(ceil(pt2d.x)));
		upperLeft.y = min(upperLeft.y, convert_int(floor(pt2d.y)));
		lowerRight.y = max(lowerRight.y, convert_int(ceil(pt2d.y)));
		zRange.x = fmin(zRange.x, pt3d.z);
		zRange.y = fmax(zRange.x, pt3d.z);
	}

	// do some sanity checks and respect image bounds
	if (upperLeft.x < 0) upperLeft.x = 0;
	if (upperLeft.y < 0) upperLeft.y = 0;
	if (lowerRight.x >= camParams.depthImageWidth) lowerRight.x = camParams.depthImageWidth - 1;
	if (lowerRight.y >= camParams.depthImageHeight) lowerRight.y = camParams.depthImageHeight - 1;
	//if (zRange.y <= VERY_CLOSE) return false; never seems to happen
	if(zRange.x < VERY_CLOSE) zRange.x = VERY_CLOSE;
	//zRange.x = fmax(zRange.x, 0.01f);
	/*zRange.x = fmin(zRange.x, FAR_AWAY);
	zRange.y = fmax(zRange.y, VERY_CLOSE);
	zRange.y = fmin(zRange.y, FAR_AWAY);*/
	//if (zRange.y < camParams.sensorDepthWorldMin) return;
	/*if(lowerRight.y < upperLeft.y || lowerRight.x < upperLeft.x)
		return;*/

	for(int by = upperLeft.y; by <= lowerRight.y; by ++)
	{
		for(int bx = upperLeft.x; bx <= lowerRight.x; bx ++)
		{
			const int locId = mul24(by, convert_int(camParams.depthImageWidth)) + bx;
			float2 minmax = vload2(locId, d_minmaxData);
			vstore2((float2)(fmin(minmax.x, zRange.x), fmax(minmax.y, zRange.y)), locId, d_minmaxData);
		}
	}
}

inline static int findVoxelBlockIndex(const int3 virtualPos,
							__global struct NuiKinfuHashEntry*	d_hashEntry,
							short3* pCachedBlockPos,
							int* pCachedBlockPtr
							)
{
	int3 blockPos = virtualPosToVoxelBlock(virtualPos);
	int linearIdx = virtualPos.x + mul24((virtualPos.y - blockPos.x), SDF_BLOCK_SIZE) + mul24(mul24((virtualPos.z - blockPos.y), SDF_BLOCK_SIZE), SDF_BLOCK_SIZE) - mul24(blockPos.z, SDF_BLOCK_SIZE3);
	short3 shortBlockPos = convert_short3(blockPos);

	if(shortBlockPos.x == pCachedBlockPos->x && shortBlockPos.y == pCachedBlockPos->y && shortBlockPos.z == pCachedBlockPos->z)
	{
		return *pCachedBlockPtr + linearIdx;
	}
	else
	{
		uint hashIdx = computeHashIndex(shortBlockPos);

		while (true) 
		{
			const struct NuiKinfuHashEntry hashEntry = d_hashEntry[hashIdx];
			if(hashEntry.pos[0] == shortBlockPos.x && hashEntry.pos[1] == shortBlockPos.y && hashEntry.pos[2] == shortBlockPos.z && hashEntry.ptr >= 0)
			{
				*pCachedBlockPos = shortBlockPos;
				*pCachedBlockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
				return *pCachedBlockPtr + linearIdx;
			}
			if (hashEntry.offset < 1)
				break;
			hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
		}
	}
	return -1;
}

inline static float readVoxelSDF(
							const int3 virtualPos,
							__global struct NuiKinfuHashEntry*	d_hashEntry,
							__global struct NuiKinfuVoxel*		d_SDFBlocks,
							short3* pCachedBlockPos,
							int* pCachedBlockPtr
							)
{
	const int blockIndex = findVoxelBlockIndex(virtualPos, d_hashEntry, pCachedBlockPos, pCachedBlockPtr);
	return (blockIndex < 0) ? 1.0f : convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
}

inline static float readInterpolatedVoxelSDF(
							const float3 point,
							__global struct NuiKinfuHashEntry*	d_hashEntry,
							__global struct NuiKinfuVoxel*		d_SDFBlocks,
							short3* pCachedBlockPos,
							int* pCachedBlockPtr
							)
{
	int3 virtualPos = (int3)(floor(point.x), floor(point.y), floor(point.z));
	float3 coeff = (float3)(point.x-convert_float(virtualPos.x), point.y-convert_float(virtualPos.y), point.z-convert_float(virtualPos.z));

	float v1 = readVoxelSDF(virtualPos + (int3)(0, 0, 0), d_hashEntry, d_SDFBlocks, pCachedBlockPos, pCachedBlockPtr);
	float v2 = readVoxelSDF(virtualPos + (int3)(1, 0, 0), d_hashEntry, d_SDFBlocks, pCachedBlockPos, pCachedBlockPtr);
	float res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	v1 = readVoxelSDF(virtualPos + (int3)(0, 1, 0), d_hashEntry, d_SDFBlocks, pCachedBlockPos, pCachedBlockPtr);
	v2 = readVoxelSDF(virtualPos + (int3)(1, 1, 0), d_hashEntry, d_SDFBlocks, pCachedBlockPos, pCachedBlockPtr);
	res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

	v1 = readVoxelSDF(virtualPos + (int3)(0, 0, 1), d_hashEntry, d_SDFBlocks, pCachedBlockPos, pCachedBlockPtr);
	v2 = readVoxelSDF(virtualPos + (int3)(1, 0, 1), d_hashEntry, d_SDFBlocks, pCachedBlockPos, pCachedBlockPtr);
	float res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	v1 = readVoxelSDF(virtualPos + (int3)(0, 1, 1), d_hashEntry, d_SDFBlocks, pCachedBlockPos, pCachedBlockPtr);
	v2 = readVoxelSDF(virtualPos + (int3)(1, 1, 1), d_hashEntry, d_SDFBlocks, pCachedBlockPos, pCachedBlockPtr);
	res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

	return ((1.0f - coeff.z) * res1 + coeff.z * res2);
}

__kernel void raycast_kernel(
			__global	float*		d_vmap,
			__global	float*		d_nmap,
			__global	uchar*		d_cmap,
			__global	float*					d_minmaxData,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
			__global struct NuiKinfuVoxel*		d_SDFBlocks,
            __constant struct NuiCLCameraParams* cameraParams,
			__global struct NuiCLRigidTransform* rigidTransform,
			const float				virtualVoxelSize,
			const float				oneOverVoxelSize,
			const	float			truncScale
        )
{
	const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
	const uint gsizex = get_global_size(0);
	const int idx = mul24(gidy, gsizex) + gidx;

	float2 minmax = vload2(idx, d_minmaxData);
	float3 pt_block_s = kinectDepthToSkeleton(gidx, gidy, minmax.x, cameraParams);
	float totalLength = fast_length(pt_block_s) * oneOverVoxelSize;
	pt_block_s = transform( pt_block_s, rigidTransform ) * oneOverVoxelSize;
	float3 pt_block_e = kinectDepthToSkeleton(gidx, gidy, minmax.y, cameraParams);
	float totalLengthMax = fast_length(pt_block_e) * oneOverVoxelSize;
	pt_block_e = transform( pt_block_e, rigidTransform ) * oneOverVoxelSize;

	// Cache
	short3 cachedBlockPos;
	int cachedBlockPtr;

	float stepLength;
	float stepScale = truncScale * oneOverVoxelSize * 0.6f;
	float sdfValue = 1.0f;
	float3 rayDirection = normalize(pt_block_e - pt_block_s);
	float3 pt_result = pt_block_s;
	while (totalLength < totalLengthMax)
	{
		int3 virtualPos = (int3)(round(pt_result.x), round(pt_result.y), round(pt_result.z));
		const int blockIndex = findVoxelBlockIndex(virtualPos, d_hashEntry, &cachedBlockPos, &cachedBlockPtr);
		sdfValue = convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		if ( blockIndex < 0 )
		{
			stepLength = SDF_BLOCK_SIZE;
		}
		else
		{
			if ((sdfValue <= 0.1f) && (sdfValue >= -0.5f))
			{
				sdfValue = readInterpolatedVoxelSDF(pt_result, d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			}
			if (sdfValue <= 0.0f)
				break;
			stepLength = max(sdfValue * stepScale, 1.0f);
		}

		pt_result += stepLength * rayDirection;
		totalLength += stepLength;
	}

	// Find the result
	if (sdfValue <= 0.0f)
	{
		stepLength = sdfValue * stepScale;
		pt_result += stepLength * rayDirection;

		// readInterpolatedVoxel
		uchar4 resultColor = (uchar4)(0, 0, 0, 0);
		float nearestDist = 1.0e20;
		float4 front, back;
		int3 virtualPos = (int3)(floor(pt_result.x), floor(pt_result.y), floor(pt_result.z));
		float3 coeff = (float3)(pt_result.x-convert_float(virtualPos.x), pt_result.y-convert_float(virtualPos.y), pt_result.z-convert_float(virtualPos.z));
		float3 ncoeff = (float3)(1.0f - coeff.x, 1.0f - coeff.y, 1.0f - coeff.z);

		// 0, 0, 0
		int blockIndex = findVoxelBlockIndex(virtualPos + (int3)(0, 0, 0), d_hashEntry, &cachedBlockPos, &cachedBlockPtr);
		front.x = (blockIndex < 0) ? 1.0f : convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		float coeffDist = fast_length(coeff);
		if(d_cmap && blockIndex >= 0)
		{
			nearestDist = coeffDist;
			resultColor.s0 = d_SDFBlocks[blockIndex].color[0];
			resultColor.s1 = d_SDFBlocks[blockIndex].color[1];
			resultColor.s2 = d_SDFBlocks[blockIndex].color[2];
			resultColor.s3 = 255;
		}
		// 1, 0, 0
		blockIndex = findVoxelBlockIndex(virtualPos + (int3)(1, 0, 0), d_hashEntry, &cachedBlockPos, &cachedBlockPtr);
		front.y = (blockIndex < 0) ? 1.0f : convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		coeffDist = fast_length((float3)(ncoeff.x, coeff.yz));
		if(d_cmap && blockIndex >= 0 && nearestDist > coeffDist)
		{
			nearestDist = coeffDist;
			resultColor.s0 = d_SDFBlocks[blockIndex].color[0];
			resultColor.s1 = d_SDFBlocks[blockIndex].color[1];
			resultColor.s2 = d_SDFBlocks[blockIndex].color[2];
			resultColor.s3 = 255;
		}

		// 0, 1, 0
		blockIndex = findVoxelBlockIndex(virtualPos + (int3)(0, 1, 0), d_hashEntry, &cachedBlockPos, &cachedBlockPtr);
		front.z = (blockIndex < 0) ? 1.0f : convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		coeffDist = fast_length((float3)(coeff.x, ncoeff.y, coeff.z));
		if(d_cmap && blockIndex >= 0 && nearestDist > coeffDist)
		{
			nearestDist = coeffDist;
			resultColor.s0 = d_SDFBlocks[blockIndex].color[0];
			resultColor.s1 = d_SDFBlocks[blockIndex].color[1];
			resultColor.s2 = d_SDFBlocks[blockIndex].color[2];
			resultColor.s3 = 255;
		}
		// 1, 1, 0
		blockIndex = findVoxelBlockIndex(virtualPos + (int3)(1, 1, 0), d_hashEntry, &cachedBlockPos, &cachedBlockPtr);
		front.w = (blockIndex < 0) ? 1.0f : convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		coeffDist = fast_length((float3)(ncoeff.xy, coeff.z));
		if(d_cmap && blockIndex >= 0 && nearestDist > coeffDist)
		{
			nearestDist = coeffDist;
			resultColor.s0 = d_SDFBlocks[blockIndex].color[0];
			resultColor.s1 = d_SDFBlocks[blockIndex].color[1];
			resultColor.s2 = d_SDFBlocks[blockIndex].color[2];
			resultColor.s3 = 255;
		}

		// 0, 0, 1
		blockIndex = findVoxelBlockIndex(virtualPos + (int3)(0, 0, 1), d_hashEntry, &cachedBlockPos, &cachedBlockPtr);
		back.x = (blockIndex < 0) ? 1.0f : convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		coeffDist = fast_length((float3)(coeff.xy, ncoeff.z));
		if(d_cmap && blockIndex >= 0 && nearestDist > coeffDist)
		{
			nearestDist = coeffDist;
			resultColor.s0 = d_SDFBlocks[blockIndex].color[0];
			resultColor.s1 = d_SDFBlocks[blockIndex].color[1];
			resultColor.s2 = d_SDFBlocks[blockIndex].color[2];
			resultColor.s3 = 255;
		}
		// 1, 0, 1
		blockIndex = findVoxelBlockIndex(virtualPos + (int3)(1, 0, 1), d_hashEntry, &cachedBlockPos, &cachedBlockPtr);
		back.y = (blockIndex < 0) ? 1.0f : convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		coeffDist = fast_length((float3)(ncoeff.x, coeff.y, ncoeff.z));
		if(d_cmap && blockIndex >= 0 && nearestDist > coeffDist)
		{
			nearestDist = coeffDist;
			resultColor.s0 = d_SDFBlocks[blockIndex].color[0];
			resultColor.s1 = d_SDFBlocks[blockIndex].color[1];
			resultColor.s2 = d_SDFBlocks[blockIndex].color[2];
			resultColor.s3 = 255;
		}

		// 0, 1, 1
		blockIndex = findVoxelBlockIndex(virtualPos + (int3)(0, 1, 1), d_hashEntry, &cachedBlockPos, &cachedBlockPtr);
		back.z = (blockIndex < 0) ? 1.0f : convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		coeffDist = fast_length((float3)(coeff.x, ncoeff.yz));
		if(d_cmap && blockIndex >= 0 && nearestDist > coeffDist)
		{
			nearestDist = coeffDist;
			resultColor.s0 = d_SDFBlocks[blockIndex].color[0];
			resultColor.s1 = d_SDFBlocks[blockIndex].color[1];
			resultColor.s2 = d_SDFBlocks[blockIndex].color[2];
			resultColor.s3 = 255;
		}
		// 1, 1, 1
		blockIndex = findVoxelBlockIndex(virtualPos + (int3)(1, 1, 1), d_hashEntry, &cachedBlockPos, &cachedBlockPtr);
		back.w = (blockIndex < 0) ? 1.0f : convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		coeffDist = fast_length((float3)(ncoeff.xyz));
		if(d_cmap && blockIndex >= 0 && nearestDist > coeffDist)
		{
			nearestDist = coeffDist;
			resultColor.s0 = d_SDFBlocks[blockIndex].color[0];
			resultColor.s1 = d_SDFBlocks[blockIndex].color[1];
			resultColor.s2 = d_SDFBlocks[blockIndex].color[2];
			resultColor.s3 = 255;
		}
		
		// Got the finest sdf value.
		sdfValue =	front.x * ncoeff.x * ncoeff.y * ncoeff.z +
					front.y * coeff.x * ncoeff.y * ncoeff.z +
					front.z * ncoeff.x * coeff.y * ncoeff.z +
					front.w * coeff.x * coeff.y * ncoeff.z +
					back.x * ncoeff.x * ncoeff.y * coeff.z +
					back.y * coeff.x * ncoeff.y * coeff.z +
					back.z * ncoeff.x * coeff.y * coeff.z +
					back.w * coeff.x * coeff.y * coeff.z;
		
		stepLength = sdfValue * stepScale;
		pt_result += stepLength * rayDirection;
		vstore3(pt_result * virtualVoxelSize, idx , d_vmap);

		if (d_nmap)
		{
			float3 ret;
			float4 tmp;
			float p1, p2, v1;
			// gradient x
			p1 = front.x * ncoeff.y * ncoeff.z +
				 front.z *  coeff.y * ncoeff.z +
				 back.x  * ncoeff.y *  coeff.z +
				 back.z  *  coeff.y *  coeff.z;

			tmp.x = readVoxelSDF(virtualPos + (int3)(-1, 0, 0), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.y = readVoxelSDF(virtualPos + (int3)(-1, 1, 0), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.z = readVoxelSDF(virtualPos + (int3)(-1, 0, 1), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.w = readVoxelSDF(virtualPos + (int3)(-1, 1, 1), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			p2 = tmp.x * ncoeff.y * ncoeff.z +
				 tmp.y *  coeff.y * ncoeff.z +
				 tmp.z * ncoeff.y *  coeff.z +
				 tmp.w *  coeff.y *  coeff.z;
			v1 = p1 * coeff.x + p2 * ncoeff.x;

			p1 = front.y * ncoeff.y * ncoeff.z +
				 front.w *  coeff.y * ncoeff.z +
				 back.y  * ncoeff.y *  coeff.z +
				 back.w  *  coeff.y *  coeff.z;
			tmp.x = readVoxelSDF(virtualPos + (int3)(2, 0, 0), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.y = readVoxelSDF(virtualPos + (int3)(2, 1, 0), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.z = readVoxelSDF(virtualPos + (int3)(2, 0, 1), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.w = readVoxelSDF(virtualPos + (int3)(2, 1, 1), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			p2 = tmp.x * ncoeff.y * ncoeff.z +
				 tmp.y *  coeff.y * ncoeff.z +
				 tmp.z * ncoeff.y *  coeff.z +
				 tmp.w *  coeff.y *  coeff.z;

			ret.x = p1 * ncoeff.x + p2 * coeff.x - v1;

			// gradient y
			p1 = front.x * ncoeff.x * ncoeff.z +
				 front.y *  coeff.x * ncoeff.z +
				 back.x  * ncoeff.x *  coeff.z +
				 back.y  *  coeff.x *  coeff.z;
			tmp.x = readVoxelSDF(virtualPos + (int3)(0, -1, 0), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.y = readVoxelSDF(virtualPos + (int3)(1, -1, 0), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.z = readVoxelSDF(virtualPos + (int3)(0, -1, 1), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.w = readVoxelSDF(virtualPos + (int3)(1, -1, 1), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			p2 = tmp.x * ncoeff.x * ncoeff.z +
				 tmp.y *  coeff.x * ncoeff.z +
				 tmp.z * ncoeff.x *  coeff.z +
				 tmp.w *  coeff.x *  coeff.z;
			v1 = p1 * coeff.y + p2 * ncoeff.y;

			p1 = front.z * ncoeff.x * ncoeff.z +
				 front.w *  coeff.x * ncoeff.z +
				 back.z  * ncoeff.x *  coeff.z +
				 back.w  *  coeff.x *  coeff.z;
			tmp.x = readVoxelSDF(virtualPos + (int3)(0, 2, 0), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.y = readVoxelSDF(virtualPos + (int3)(1, 2, 0), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.z = readVoxelSDF(virtualPos + (int3)(0, 2, 1), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.w = readVoxelSDF(virtualPos + (int3)(1, 2, 1), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			p2 = tmp.x * ncoeff.x * ncoeff.z +
				 tmp.y *  coeff.x * ncoeff.z +
				 tmp.z * ncoeff.x *  coeff.z +
				 tmp.w *  coeff.x *  coeff.z;

			ret.y = p1 * ncoeff.y + p2 * coeff.y - v1;

			// gradient z
			p1 = front.x * ncoeff.x * ncoeff.y +
				 front.y *  coeff.x * ncoeff.y +
				 front.z * ncoeff.x *  coeff.y +
				 front.w *  coeff.x *  coeff.y;
			tmp.x = readVoxelSDF(virtualPos + (int3)(0, 0, -1), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.y = readVoxelSDF(virtualPos + (int3)(1, 0, -1), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.z = readVoxelSDF(virtualPos + (int3)(0, 1, -1), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.w = readVoxelSDF(virtualPos + (int3)(1, 1, -1), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			p2 = tmp.x * ncoeff.x * ncoeff.y +
				 tmp.y *  coeff.x * ncoeff.y +
				 tmp.z * ncoeff.x *  coeff.y +
				 tmp.w *  coeff.x *  coeff.y;
			v1 = p1 * coeff.z + p2 * ncoeff.z;

			p1 = back.x * ncoeff.x * ncoeff.y +
				 back.y *  coeff.x * ncoeff.y +
				 back.z * ncoeff.x *  coeff.y +
				 back.w *  coeff.x *  coeff.y;
			tmp.x = readVoxelSDF(virtualPos + (int3)(0, 0, 2), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.y = readVoxelSDF(virtualPos + (int3)(1, 0, 2), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.z = readVoxelSDF(virtualPos + (int3)(0, 1, 2), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			tmp.w = readVoxelSDF(virtualPos + (int3)(1, 1, 2), d_hashEntry, d_SDFBlocks, &cachedBlockPos, &cachedBlockPtr);
			p2 = tmp.x * ncoeff.x * ncoeff.y +
				 tmp.y *  coeff.x * ncoeff.y +
				 tmp.z * ncoeff.x *  coeff.y +
				 tmp.w *  coeff.x *  coeff.y;

			ret.z = p1 * ncoeff.z + p2 * coeff.z - v1;
			ret = normalize(ret);
			vstore3(ret, idx , d_nmap);
		}

		if(d_cmap)
			vstore4(resultColor, idx , d_cmap);
	}
	else
	{
		vstore3((float3)(NAN, NAN, NAN), idx , d_vmap);
		vstore3((float3)(NAN, NAN, NAN), idx , d_nmap);
		vstore4((uchar4)(0, 0, 0, 0), idx , d_cmap);
	}
}

__kernel void forwardProject_kernel(
			__global	float*		d_vmap,
			__global	float*		d_nmap,
			__global	uchar*		d_cmap,
            __constant struct NuiCLCameraParams* cameraParams,
			__global struct NuiCLRigidTransform* rigidTransform,
			const float				virtualVoxelSize
        )
{
	const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
	const uint gsizex = get_global_size(0);
	const int idx = mul24(gidy, gsizex) + gidx;

	float3 pixel = vload3(idx, d_vmap);
	if(_isnan3(pixel))
		return;
	float3 pt3d = transform( pixel, rigidTransform ) * virtualVoxelSize;

	uint2 screenPos = convert_uint2(cameraToKinectScreen(pt3d, cameraParams) + (float2)(0.5f, 0.5f));

	struct NuiCLCameraParams camParams = *cameraParams;
	if(screenPos.x < camParams.depthImageWidth && screenPos.y < camParams.depthImageHeight)
	{
		const uint screenId = mul24(screenPos.y, camParams.depthImageWidth) + screenPos.x;
		vstore3(pixel, screenId, d_vmap);

		float3 norm = vload3(idx, d_nmap);
		vstore3(norm, screenId, d_nmap);

		uchar4 clr = vload4(idx, d_cmap);
		vstore4(clr, screenId, d_cmap);
	}
}