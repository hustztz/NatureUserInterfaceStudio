#include "hashingBlock.cl"

__kernel void alloc_SDFs_kernel(
            __global float* depths,
			__global uint*	d_bitMask,
			__constant struct NuiCLCameraParams* cameraParams,
			__global struct NuiCLRigidTransform* matrix,
			__global struct NuiCLHashEntry*	d_hash,
			__global uint*	d_heap,
			__global uint*	d_heapCounter,
			__global int*	d_hashBucketMutex,
			const float		maxIntegrationDistance,
			const float		truncation,
			const float		truncScale,
			const float		virtualVoxelSize,
			const float3	streamingVoxelExtents,
			const int3		minGridPos,
			const int3		gridDimensions,
			const uint		hashNumBuckets,
			const uint		hashMaxCollisionLinkedListSize
        )
{
    const int gidx = get_global_id(0);
	const int gidy = get_global_id(1);
	
	const int gsizex = get_global_size(0);
    
	const int id = mul24(gidy, gsizex) + gidx;
	float Dp = vload(id, depths);
	if(isnan(Dp) || Dp > maxIntegrationDistance)
		return;
	
	float t = truncation + truncScale * Dp;
	float minDepth = fmin(maxIntegrationDistance, Dp-t);
	float maxDepth = fmin(maxIntegrationDistance, Dp+t);
	if (minDepth >= maxDepth) return;

	struct NuiCLCameraParams camParams = *cameraParams;
	float3 screenToWorld = (float3)((gidx-camParams.cx)*camParams.fx_inv, (gidy-camParams.cy)*camParams.fy_inv, 1.0);
	float3 rayMin = screenToWorld * minDepth;
	rayMin = transform( rayMin, matrix );
	float3 rayMax = screenToWorld * maxDepth;
	rayMax = transform( rayMax, matrix );

	float3 rayDir = normalize(rayMax - rayMin);
	
	int3 idCurrentVoxel = worldToSDFBlock(rayMin, virtualVoxelSize);
	int3 idEnd = worldToSDFBlock(rayMax, virtualVoxelSize);
		
	float3 step = (float3)(sign(rayDir));
	float3 boundaryPos = SDFBlockToWorld(idCurrentVoxel+convert_int3(clamp(step, 0.0, 1.0f)), virtualVoxelSize)-0.5f*virtualVoxelSize;
	float3 tMax = (boundaryPos - rayMin) / rayDir;
	float3 tDelta = (step*SDF_BLOCK_SIZE*virtualVoxelSize) / rayDir;
	int3 idBound = convert_int3(convert_float3(idEnd) + step);

	//#pragma unroll
	//for(int c = 0; c < 3; c++) {
	//	if (rayDir[c] == 0.0f) { tMax[c] = PINF; tDelta[c] = PINF; }
	//	if (boundaryPos[c] - rayMin[c] == 0.0f) { tMax[c] = PINF; tDelta[c] = PINF; }
	//}
	if (rayDir.x == 0.0f) { tMax.x = NAN; tDelta.x = NAN; }
	if (boundaryPos.x - rayMin.x == 0.0f) { tMax.x = NAN; tDelta.x = NAN; }

	if (rayDir.y == 0.0f) { tMax.y = NAN; tDelta.y = NAN; }
	if (boundaryPos.y - rayMin.y == 0.0f) { tMax.y = NAN; tDelta.y = NAN; }

	if (rayDir.z == 0.0f) { tMax.z = NAN; tDelta.z = NAN; }
	if (boundaryPos.z - rayMin.z == 0.0f) { tMax.z = NAN; tDelta.z = NAN; }


	unsigned int iter = 0; // iter < g_MaxLoopIterCount
	unsigned int g_MaxLoopIterCount = 1024;	//TODO MATTHIAS MOVE TO GLOBAL APP STATE
	while(iter < g_MaxLoopIterCount)
	{
		//check if it's in the frustum and not checked out
		if (isSDFBlockInCameraFrustumApprox(idCurrentVoxel, virtualVoxelSize, cameraParams, matrix) &&
			!isSDFBlockStreamedOut(idCurrentVoxel, virtualVoxelSize, streamingVoxelExtents, minGridPos, gridDimensions, d_bitMask))
		{		
			allocBlock(idCurrentVoxel, d_hash, d_heap, d_heapCounter, d_hashBucketMutex, hashNumBuckets, hashMaxCollisionLinkedListSize);
		}

		// Traverse voxel grid
		if(tMax.x < tMax.y && tMax.x < tMax.z)	{
			idCurrentVoxel.x += step.x;
			if(idCurrentVoxel.x == idBound.x) return;
			tMax.x += tDelta.x;
		}
		else if(tMax.z < tMax.y) {
			idCurrentVoxel.z += step.z;
			if(idCurrentVoxel.z == idBound.z) return;
			tMax.z += tDelta.z;
		}
		else	{
			idCurrentVoxel.y += step.y;
			if(idCurrentVoxel.y == idBound.y) return;
			tMax.y += tDelta.y;
		}

		iter++;
	}
}

__kernel void fillDecisionArrayKernel(
	__global uint*	d_hashDecision,
	__global struct NuiCLHashEntry*	d_hash,
	__constant struct NuiCLCameraParams* cameraParams,
	__global struct NuiCLRigidTransform* matrix,
	const float		virtualVoxelSize
	)
{
	const int gidx = get_global_id(0);

	d_hashDecision[gidx] = 0;
	if( d_hash[gidx] .ptr != FREE_ENTRY )
	{
		int3 pos = (int3)(d_hash[gidx].pos[0], d_hash[gidx].pos[1], d_hash[gidx].pos[2]);
		if( isSDFBlockInCameraFrustumApprox(pos, virtualVoxelSize, cameraParams, matrix) )
		{
			d_hashDecision[gidx] = 1;	//yes
		}
	}
}

__kernel void compactifyHashKernel(
	__global struct NuiCLHashEntry*	d_hashCompactified,
	__global uint*	d_hashDecision,
	__global struct NuiCLHashEntry*	d_hash,
	__global int*	d_hashDecisionPrefix
	)
{
	const int gidx = get_global_id(0);

	if( d_hashDecision[gidx] == 1 )
	{
		d_hashCompactified[d_hashDecisionPrefix[gidx]-1] = d_hash[gidx];
	}
}

inline static int3 delinearizeVoxelIndex(uint idx)
{
	int x = idx % SDF_BLOCK_SIZE;
	int y = (idx % (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE;
	int z = idx / (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE);	
	return (int3)(x,y,z);
}

__kernel void integrateDepthMapKernel(
	__global float*		depths,
	__global uchar*		colors,
	__global struct NuiCLVoxel*		d_SDFBlocks,
	__global struct NuiCLHashEntry*	d_hashCompactified,
	__constant struct NuiCLCameraParams* cameraParams,
	__global struct NuiCLRigidTransform* matrix,
	const float			virtualVoxelSize,
	const float			maxIntegrationDistance,
	const float			truncation,
	const float			truncScale,
	const uint			integrationWeightSample,
	const uchar			integrationWeightMax
	)
{
	const int gidx = get_global_id(0);

	//TODO check if we should load this in shared memory
	struct NuiCLHashEntry entry = d_hashCompactified[gidx];
	//if (entry.ptr == FREE_ENTRY) {
	//	printf("invliad integrate");
	//	return; //should never happen since we did the compactification before
	//}

	int3 pi_base = (int3)(entry.pos[0], entry.pos[1], entry.pos[2]) * SDF_BLOCK_SIZE;

	const uint lidx = get_local_id(0);	//inside of an SDF block
	int3 pi = pi_base + delinearizeVoxelIndex(lidx);
	float3 pf = convert_float3(pi) * virtualVoxelSize;
	float3 pCamera = transformInverse( pf, matrix );
	uint2 screenPos = convert_uint2(cameraToKinectScreen(pCamera, cameraParams) + (float2)(0.5f, 0.5f));

	struct NuiCLCameraParams camParams = *cameraParams;
	if(screenPos.x < camParams.depthImageWidth && screenPos.y < camParams.depthImageHeight)
	{
		const uint screenId = mul24(screenPos.y, camParams.depthImageWidth) + screenPos.x;
		float depth = vload(screenId, depths);
		if(isnan(depth) || depth > maxIntegrationDistance)
			return;

		uchar4 color = vload4(screenId, colors);
		if( 0 == color.w )
			return;

		//valid depth and color value
		float depthZeroOne = cameraToKinectProjZ(depth, cameraParams);

		float sdf = depth - pf.z;
		float truncation = truncation + truncScale * depth;
		if (sdf > -truncation) // && depthZeroOne >= 0.0f && depthZeroOne <= 1.0f) //check if in truncation range should already be made in depth map computation
		{
			if (sdf >= 0.0f) {
				sdf = min(truncation, sdf);
			} else {
				sdf = max(-truncation, sdf);
			}

			//float weightUpdate = g_WeightSample;
			//weightUpdate = (1-depthZeroOne)*5.0f + depthZeroOne*0.05f;
			//weightUpdate *= g_WeightSample;
			float weightUpdate = max(integrationWeightSample * 1.5f * (1.0f-depthZeroOne), 1.0f);

			uint idx = entry.ptr + lidx;
			struct NuiCLVoxel oldVoxel = d_SDFBlocks[idx];

			float new_color_r = (convert_float(oldVoxel.color[0] * oldVoxel.weight) + convert_float(color.x*weightUpdate)) / (convert_float(oldVoxel.weight + weightUpdate));
			float new_color_g = (convert_float(oldVoxel.color[1] * oldVoxel.weight) + convert_float(color.y*weightUpdate)) / (convert_float(oldVoxel.weight + weightUpdate));
			float new_color_b = (convert_float(oldVoxel.color[2] * oldVoxel.weight) + convert_float(color.z*weightUpdate)) / (convert_float(oldVoxel.weight + weightUpdate));
			
			struct NuiCLVoxel newVoxel;	//construct current voxel
			newVoxel.sdf = (oldVoxel.sdf * convert_float(oldVoxel.weight) + sdf * convert_float(weightUpdate)) / (convert_float(oldVoxel.weight) + convert_float(weightUpdate));
			newVoxel.weight = min(integrationWeightMax, convert_uchar(oldVoxel.weight + convert_uchar(weightUpdate)));
			newVoxel.color[0] = convert_uchar( clamp(new_color_r, 0.0f, 255.f) );
			newVoxel.color[1] = convert_uchar( clamp(new_color_g, 0.0f, 255.f) );
			newVoxel.color[2] = convert_uchar( clamp(new_color_b, 0.0f, 255.f) );
			
			d_SDFBlocks[idx] = newVoxel;
		}
	}
}

__kernel void starveVoxelsKernel(
	__global struct NuiCLVoxel*		d_SDFBlocks,
	__global struct NuiCLHashEntry*	d_hashCompactified
	)
{
	const int gidx = get_global_id(0);
	const int lidx = get_local_id(0);
	struct NuiCLHashEntry entry = d_hashCompactified[gidx];

	//is typically exectued only every n'th frame
	uchar weight = d_SDFBlocks[entry.ptr + lidx].weight;
	weight = max(0, weight-1);	
	d_SDFBlocks[entry.ptr + lidx].weight = weight;
}

__kernel void garbageCollectIdentifyKernel(
	__global struct NuiCLVoxel*		d_SDFBlocks,
	__global struct NuiCLHashEntry*	d_hashCompactified,
	__global uint*					d_hashDecision,
	__constant struct NuiCLCameraParams* cameraParams,
	const float						truncation,
	const float						truncScale
	)
{
	__local float	shared_MinSDF[SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE / 2];
	__local uchar	shared_MaxWeight[SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE / 2];

	const int gidx = get_global_id(0);
	const int lidx = get_local_id(0);
	const uint lsizex = get_local_size(0);
	struct NuiCLHashEntry entry = d_hashCompactified[gidx];

	//uint h = hashData.computeHashPos(entry.pos);
	//hashData.d_hashDecision[hashIdx] = 1;
	//if (hashData.d_hashBucketMutex[h] == LOCK_ENTRY)	return;

	//if (entry.ptr == FREE_ENTRY) return; //should never happen since we did compactify before
	//const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	const unsigned int idx0 = entry.ptr + 2*lidx+0;
	const unsigned int idx1 = entry.ptr + 2*lidx+1;

	struct NuiCLVoxel v0 = d_SDFBlocks[idx0];
	struct NuiCLVoxel v1 = d_SDFBlocks[idx1];

	shared_MinSDF[lidx] = min(fabs(v0.sdf), fabs(v1.sdf));	//init shared memory
	shared_MaxWeight[lidx] = max(convert_uchar(v0.weight), convert_uchar(v1.weight));

	for (uint stride = 2; stride <= lsizex; stride <<= 1) {
		barrier(CLK_LOCAL_MEM_FENCE);
		if ((lidx  & (stride-1)) == (stride-1)) {
			shared_MinSDF[lidx] = min(shared_MinSDF[lidx-stride/2], shared_MinSDF[lidx]);
			shared_MaxWeight[lidx] = max(shared_MaxWeight[lidx-stride/2], shared_MaxWeight[lidx]);
		}
	}

	barrier(CLK_LOCAL_MEM_FENCE);

	if (lidx == lsizex - 1) {
		float minSDF = shared_MinSDF[lidx];
		uchar maxWeight = shared_MaxWeight[lidx];

		struct NuiCLCameraParams camParams = *cameraParams;
		float t = truncation + truncScale * camParams.sensorDepthWorldMax;	//MATTHIAS TODO check whether this is a reasonable metric

		if (minSDF >= t || maxWeight == 0) {
			d_hashDecision[gidx] = 1;
		} else {
			d_hashDecision[gidx] = 0;
		}
	}
}

__kernel void garbageCollectFreeKernel(
	__global struct NuiCLVoxel*		d_SDFBlocks,
	__global struct NuiCLHashEntry*	d_hash,
	__global struct NuiCLHashEntry*	d_hashCompactified,
	__global uint*					d_hashDecision,
	__global uint*					d_heap,
	__global uint*					d_heapCounter,
	__global int*					d_hashBucketMutex,
	const uint						hashNumBuckets,
	const uint						hashMaxCollisionLinkedListSize
	)
{
	const int gidx = get_global_id(0);

	if (d_hashDecision[gidx] != 0) {	//decision to delete the hash entry

		struct NuiCLHashEntry entry = d_hashCompactified[gidx];
		//if (entry.ptr == FREE_ENTRY) return; //should never happen since we did compactify before

		int3 sdfBlock = (int3)(entry.pos[0], entry.pos[1], entry.pos[2]);
		if (deleteHashEntryElement(sdfBlock, d_hash, d_heap, d_heapCounter, d_hashBucketMutex, hashNumBuckets, hashMaxCollisionLinkedListSize)) {	//delete hash entry from hash (and performs heap append)
			const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

			for (uint i = 0; i < linBlockSize; i++) {	//clear sdf block: CHECK TODO another kernel?
				deleteSDFVoxel(entry.ptr + i, d_SDFBlocks);
			}
		}
	}
}
