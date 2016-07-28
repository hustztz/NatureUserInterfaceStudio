#include "hashingSDFUtils.cl"
#include "utils.cl"

inline static bool isSDFBlockInCameraFrustumApprox(
			int3			sdfBlock,
			__constant struct NuiCLCameraParams* cameraParams,
			__global struct NuiCLRigidTransform* matrix,
			float			virtualVoxelSize)
{
	float3 posWorld = SDFBlockToWorld(sdfBlock, virtualVoxelSize) + virtualVoxelSize * 0.5f * (SDF_BLOCK_SIZE - 1.0f);
	float3 pCamera = transformInverse( posWorld, matrix );
	float3 pProj = cameraToKinectProj(pCamera, cameraParams);
	//pProj *= 1.5f;	//TODO THIS IS A HACK FIX IT :)
	pProj *= 0.95f;
	return !(pProj.x < -1.0f || pProj.x > 1.0f || pProj.y < -1.0f || pProj.y > 1.0f || pProj.z < 0.0f || pProj.z > 1.0f);
}

inline static bool isSDFBlockStreamedOut(
			int3			sdfBlock,
			const float		virtualVoxelSize,
			const float3	streamingVoxelExtents,
			const int3		minGridPos,
			const int3		gridDimensions,
			__global uint*	d_bitMask)
{
	if(!d_bitMask)
		return false;
	float3 posWorld = SDFBlockToWorld(sdfBlock, virtualVoxelSize); // sdfBlock is assigned to chunk by the bottom right sample pos
	posWorld = posWorld / streamingVoxelExtents;
	float3 signPosWorld;
	signPosWorld.x = convert_float(sign(posWorld.x));
	signPosWorld.y = convert_float(sign(posWorld.y));
	signPosWorld.z = convert_float(sign(posWorld.z));
	int3 chunkPos = convert_int3(posWorld + signPosWorld * 0.5f);

	chunkPos = chunkPos - minGridPos;
	uint index = chunkPos.z * gridDimensions.x * gridDimensions.y +
			chunkPos.y * gridDimensions.x +
			chunkPos.x;

	uint nBitsInT = 32;
	return ((d_bitMask[index/nBitsInT] & (0x1 << (index%nBitsInT))) != 0x0);
}

__kernel void alloc_SDFs_kernel(
            __global float* depths,
			__constant struct NuiCLCameraParams* cameraParams,
			__global struct NuiCLRigidTransform* matrix,
			__global struct NuiCLHashEntry*	d_hash,
			__global uint*	d_heap,
			__global uint*	d_heapCounter,
			__global int*	d_hashBucketMutex,
			__global uint*	d_bitMask,
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
    const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
	
	const uint gsizex = get_global_size(0);
    
	const uint id = mul24(gidy, gsizex) + gidx;
	float Dp = vload(id, depths);
	if(isnan(Dp))
		return;
	
	float t = truncation + truncScale * Dp;
	float minDepth = Dp-t;
	float maxDepth = Dp+t;
	if (minDepth >= maxDepth) return;

	float3 rayMin = kinectDepthToSkeleton(gidx, gidy, minDepth, cameraParams);
	rayMin = transform( rayMin, matrix );
	float3 rayMax = kinectDepthToSkeleton(gidx, gidy, maxDepth, cameraParams);
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
	if (rayDir.x == 0.0f) { tMax.x = FP_PINF; tDelta.x = FP_PINF; }
	if (boundaryPos.x - rayMin.x == 0.0f) { tMax.x = FP_PINF; tDelta.x = FP_PINF; }

	if (rayDir.y == 0.0f) { tMax.y = FP_PINF; tDelta.y = FP_PINF; }
	if (boundaryPos.y - rayMin.y == 0.0f) { tMax.y = FP_PINF; tDelta.y = FP_PINF; }

	if (rayDir.z == 0.0f) { tMax.z = FP_PINF; tDelta.z = FP_PINF; }
	if (boundaryPos.z - rayMin.z == 0.0f) { tMax.z = FP_PINF; tDelta.z = FP_PINF; }


	uint iter = 0; // iter < g_MaxLoopIterCount
	uint g_MaxLoopIterCount = 1024;	//TODO MATTHIAS MOVE TO GLOBAL APP STATE
	while(iter < g_MaxLoopIterCount)
	{
		//check if it's in the frustum and not checked out
		if (isSDFBlockInCameraFrustumApprox(idCurrentVoxel, cameraParams, matrix, virtualVoxelSize) &&
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
	__constant struct NuiCLCameraParams* cameraParams,
	__global struct NuiCLRigidTransform* matrix,
	__global uint*	d_hashDecision,
	__global struct NuiCLHashEntry*	d_hash,
	const float		virtualVoxelSize
	)
{
	const int gidx = get_global_id(0);

	d_hashDecision[gidx] = 0;
	if( d_hash[gidx].ptr != FREE_ENTRY )
	{
		int3 pos = (int3)(d_hash[gidx].pos[0], d_hash[gidx].pos[1], d_hash[gidx].pos[2]);
		if( isSDFBlockInCameraFrustumApprox(pos, cameraParams, matrix, virtualVoxelSize) )
		{
			d_hashDecision[gidx] = 1;	//yes
		}
	}
}

__kernel void compactifyHashKernel(
	__global uint*	d_hashDecision,
	__global uint*	d_hashDecisionPrefix,
	__global struct NuiCLHashEntry*	d_hash,
	__global struct NuiCLHashEntry*	d_hashCompactified
	)
{
	const int gidx = get_global_id(0);

	if( d_hashDecision[gidx] == 1 )
	{
		uint prefixIdx = d_hashDecisionPrefix[gidx];
		if(prefixIdx > 0)
			prefixIdx --;
		d_hashCompactified[prefixIdx] = d_hash[gidx];
	}
}

__kernel void integrateDepthMapKernel(
	__global float*		depths,
	__global uchar*		colors,
	__constant struct NuiCLCameraParams* cameraParams,
	__global struct NuiCLRigidTransform* matrix,
	__global struct NuiCLVoxel*		d_SDFBlocks,
	__global struct NuiCLHashEntry*	d_hashCompactified,
	const float			virtualVoxelSize,
	const float			truncation,
	const float			truncScale,
	const uint			integrationWeightSample,
	const uchar			integrationWeightMax
	)
{
	const uint blockIdx = get_group_id(0);

	//TODO check if we should load this in shared memory
	struct NuiCLHashEntry entry = d_hashCompactified[blockIdx];
	//if (entry.ptr == FREE_ENTRY) {
	//	printf("invliad integrate");
	//	return; //should never happen since we did the compactification before
	//}

	int3 pi_base = SDFBlockToVirtualVoxelPos( (int3)(entry.pos[0], entry.pos[1], entry.pos[2]) );

	const uint lidx = get_local_id(0);	//inside of an SDF block
	int3 pi = pi_base + delinearizeVoxelIndex(lidx);
	float3 pf = virtualVoxelPosToWorld(pi, virtualVoxelSize);
	float3 pCamera = transformInverse( pf, matrix );
	uint2 screenPos = convert_uint2(cameraToKinectScreen(pCamera, cameraParams) + (float2)(0.5f, 0.5f));

	struct NuiCLCameraParams camParams = *cameraParams;
	if(screenPos.x < camParams.depthImageWidth && screenPos.y < camParams.depthImageHeight)
	{
		const uint screenId = mul24(screenPos.y, camParams.depthImageWidth) + screenPos.x;
		float depth = vload(screenId, depths);
		if(isnan(depth))
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
			
			struct NuiCLVoxel newVoxel;	//construct current voxel
			newVoxel.sdf = (oldVoxel.sdf * convert_float(oldVoxel.weight) + sdf * convert_float(weightUpdate)) / (convert_float(oldVoxel.weight) + convert_float(weightUpdate));
			newVoxel.weight = min(integrationWeightMax, convert_uchar(oldVoxel.weight + convert_uchar(weightUpdate)));

			float new_color_r = (convert_float(oldVoxel.color[0] * oldVoxel.weight) + convert_float(color.x*weightUpdate)) / (convert_float(oldVoxel.weight + weightUpdate));
			float new_color_g = (convert_float(oldVoxel.color[1] * oldVoxel.weight) + convert_float(color.y*weightUpdate)) / (convert_float(oldVoxel.weight + weightUpdate));
			float new_color_b = (convert_float(oldVoxel.color[2] * oldVoxel.weight) + convert_float(color.z*weightUpdate)) / (convert_float(oldVoxel.weight + weightUpdate));
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
	const uint blockIdx = get_group_id(0);
	const int lidx = get_local_id(0);
	struct NuiCLHashEntry entry = d_hashCompactified[blockIdx];

	//is typically exectued only every n'th frame
	uchar weight = d_SDFBlocks[entry.ptr + lidx].weight;
	if(weight > 0)
		weight --;
	d_SDFBlocks[entry.ptr + lidx].weight = weight;
}

__kernel void garbageCollectIdentifyKernel(
	__global struct NuiCLVoxel*		d_SDFBlocks,
	__global struct NuiCLHashEntry*	d_hashCompactified,
	__global uint*					d_hashDecision,
	__constant struct NuiCLCameraParams* cameraParams,
	const float						truncation,
	const float						truncScale,
	__local float*					l_shared_MinSDF,
	__local uchar*					l_shared_MaxWeight
	)
{
	const uint blockIdx = get_group_id(0);
	const uint lidx = get_local_id(0);
	const uint lsizex = get_local_size(0);
	struct NuiCLHashEntry entry = d_hashCompactified[blockIdx];

	//uint h = hashData.computeHashPos(entry.pos);
	//hashData.d_hashDecision[hashIdx] = 1;
	//if (hashData.d_hashBucketMutex[h] == LOCK_ENTRY)	return;

	//if (entry.ptr == FREE_ENTRY) return; //should never happen since we did compactify before
	//const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	const uint idx0 = entry.ptr + 2*lidx+0;
	const uint idx1 = entry.ptr + 2*lidx+1;

	struct NuiCLVoxel v0 = d_SDFBlocks[idx0];
	struct NuiCLVoxel v1 = d_SDFBlocks[idx1];

	if (v0.weight == 0)	v0.sdf = FP_PINF;
	if (v1.weight == 0)	v1.sdf = FP_PINF;

	//init shared memory
	l_shared_MinSDF[lidx] = fmin(fabs(v0.sdf), fabs(v1.sdf));
	l_shared_MaxWeight[lidx] = max(convert_uchar(v0.weight), convert_uchar(v1.weight));

	for (uint stride = 2; stride <= lsizex; stride <<= 1) {
		barrier(CLK_LOCAL_MEM_FENCE);
		if ((lidx  & (stride-1)) == (stride-1)) {
			l_shared_MinSDF[lidx] = min(l_shared_MinSDF[lidx-stride/2], l_shared_MinSDF[lidx]);
			l_shared_MaxWeight[lidx] = max(l_shared_MaxWeight[lidx-stride/2], l_shared_MaxWeight[lidx]);
		}
	}

	barrier(CLK_LOCAL_MEM_FENCE);

	if (lidx == lsizex - 1) {
		float minSDF = l_shared_MinSDF[lidx];
		uchar maxWeight = l_shared_MaxWeight[lidx];

		struct NuiCLCameraParams camParams = *cameraParams;
		float t = truncation + truncScale * camParams.sensorDepthWorldMax;	//MATTHIAS TODO check whether this is a reasonable metric

		if (minSDF >= t || maxWeight == 0) {
			d_hashDecision[blockIdx] = 1;
		} else {
			d_hashDecision[blockIdx] = 0;
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
