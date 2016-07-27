#include "utils.cl"
#include "hashingSDFUtils.cl"

inline static float3 frac(float3 val)
{
	return (val - floor(val));
}

inline static struct NuiCLVoxel trilinearInterpolationSimpleFastFast(
	const float3	pos,
	__global struct NuiCLHashEntry*	d_hash,
	__global struct NuiCLVoxel*		d_SDFBlocks,
	const float		virtualVoxelSize,
	const uint		hashNumBuckets,
	const uint		hashMaxCollisionLinkedListSize)
{
	const float oSet = virtualVoxelSize;
	const float3 posDual = pos - (float3)(oSet/2.0f, oSet/2.0f, oSet/2.0f);
	float3 weight = frac(pos / virtualVoxelSize);

	struct NuiCLVoxel out;
	out.weight = 0;
	float dist = 0.0f;
	float3 colorFloat = (float3)(0.0f, 0.0f, 0.0f);
	// 0.0f, 0.0f, 0.0f
	struct NuiCLVoxel v = getVoxel(posDual+(float3)(0.0f, 0.0f, 0.0f), d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
	if(v.weight == 0)
		return out;
	float3 vColor = (float3)(v.color[0], v.color[1], v.color[2]);
	dist += (1.0f-weight.x)*(1.0f-weight.y)*(1.0f-weight.z)*v.sdf;
	colorFloat += (1.0f-weight.x)*(1.0f-weight.y)*(1.0f-weight.z)*vColor;

	// oSet, 0.0f, 0.0f
	v = getVoxel(posDual+(float3)(oSet, 0.0f, 0.0f), d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
	if(v.weight == 0)
		return out;
	vColor = (float3)(v.color[0], v.color[1], v.color[2]);
	dist+=	   weight.x *(1.0f-weight.y)*(1.0f-weight.z)*v.sdf;
	colorFloat+=	   weight.x *(1.0f-weight.y)*(1.0f-weight.z)*vColor;

	// 0.0f, oSet, 0.0f
	v = getVoxel(posDual+(float3)(0.0f, oSet, 0.0f), d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
	if(v.weight == 0)
		return out;
	vColor = (float3)(v.color[0], v.color[1], v.color[2]);
	dist += (1.0f-weight.x)*	   weight.y *(1.0f-weight.z)*v.sdf;
	colorFloat += (1.0f-weight.x)*	   weight.y *(1.0f-weight.z)*vColor;

	// 0.0f, 0.0f, oSet
	v = getVoxel(posDual+(float3)(0.0f, 0.0f, oSet), d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
	if(v.weight == 0)
		return out;
	vColor = (float3)(v.color[0], v.color[1], v.color[2]);
	dist += (1.0f-weight.x)*(1.0f-weight.y)*	   weight.z *v.sdf;
	colorFloat += (1.0f-weight.x)*(1.0f-weight.y)*	   weight.z *vColor;

	// oSet, oSet, 0.0f
	v = getVoxel(posDual+(float3)(oSet, oSet, 0.0f), d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
	if(v.weight == 0)
		return out;
	vColor = (float3)(v.color[0], v.color[1], v.color[2]);
	dist +=	   weight.x *	   weight.y *(1.0f-weight.z)*v.sdf;
	colorFloat +=	   weight.x *	   weight.y *(1.0f-weight.z)*vColor;

	// 0.0f, oSet, oSet
	v = getVoxel(posDual+(float3)(0.0f, oSet, oSet), d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
	if(v.weight == 0)
		return out;
	vColor = (float3)(v.color[0], v.color[1], v.color[2]);
	dist += (1.0f-weight.x)*	   weight.y *	   weight.z *v.sdf;
	colorFloat += (1.0f-weight.x)*	   weight.y *	   weight.z *vColor;

	// oSet, 0.0f, oSet
	v = getVoxel(posDual+(float3)(oSet, 0.0f, oSet), d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
	if(v.weight == 0) return out;
	vColor = (float3)(v.color[0], v.color[1], v.color[2]);
	dist +=	   weight.x *(1.0f-weight.y)*	   weight.z *v.sdf;
	colorFloat +=	   weight.x *(1.0f-weight.y)*	   weight.z *vColor;

	// oSet, oSet, oSet
	v = getVoxel(posDual+(float3)(oSet, oSet, oSet), d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
	if(v.weight == 0)
		return out;
	vColor = (float3)(v.color[0], v.color[1], v.color[2]);
	dist +=	   weight.x *	   weight.y *	   weight.z *v.sdf;
	colorFloat +=	   weight.x *	   weight.y *	   weight.z *vColor;

	out.weight = 1;
	out.sdf = dist;
	out.color[0] = colorFloat.x;
	out.color[1] = colorFloat.y;
	out.color[2] = colorFloat.z;
	
	return out;
}

inline static float findIntersectionLinear(float tNear, float tFar, float dNear, float dFar)
{
	return tNear+(dNear/(dNear-dFar))*(tFar-tNear);
}

// d0 near, d1 far
inline static struct NuiCLVoxel findIntersectionBisection(
	float3 worldCamPos,
	float3 worldDir,
	__global struct NuiCLHashEntry*	d_hash,
	__global struct NuiCLVoxel*		d_SDFBlocks,
	const float		virtualVoxelSize,
	const uint		hashNumBuckets,
	const uint		hashMaxCollisionLinkedListSize,
	float d0, float r0, float d1, float r1
	)
{
	float a = r0; float aDist = d0;
	float b = r1; float bDist = d1;
	float c = 0.0f;
	const unsigned int nIterationsBisection = 3;

	struct NuiCLVoxel out;
	for(uint i = 0; i<nIterationsBisection; i++)
	{
		c = findIntersectionLinear(a, b, aDist, bDist);

		out = trilinearInterpolationSimpleFastFast(worldCamPos+c*worldDir, d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
		if(0 == out.weight)
			return out;

		if(aDist*out.sdf > 0.0)
		{
			a = c;
			aDist = out.sdf;
		}
		else
		{
			b = c;
			bDist = out.sdf;
		}
	}

	out.sdf = c;
	return out;
}

inline static float3 gradientForPoint(
	float3			pos,
	__global struct NuiCLHashEntry*	d_hash,
	__global struct NuiCLVoxel*		d_SDFBlocks,
	const float		virtualVoxelSize,
	const uint		hashNumBuckets,
	const uint		hashMaxCollisionLinkedListSize
	)
{
	float3 offset = (float3)(virtualVoxelSize, virtualVoxelSize, virtualVoxelSize);

	struct NuiCLVoxel vp00 = trilinearInterpolationSimpleFastFast(pos-(float3)(0.5f*offset.x, 0.0f, 0.0f), d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
	struct NuiCLVoxel v0p0 = trilinearInterpolationSimpleFastFast(pos-(float3)(0.0f, 0.5f*offset.y, 0.0f), d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
	struct NuiCLVoxel v00p = trilinearInterpolationSimpleFastFast(pos-(float3)(0.0f, 0.0f, 0.5f*offset.z), d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);

	struct NuiCLVoxel v100 = trilinearInterpolationSimpleFastFast(pos+(float3)(0.5f*offset.x, 0.0f, 0.0f), d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
	struct NuiCLVoxel v010 = trilinearInterpolationSimpleFastFast(pos+(float3)(0.0f, 0.5f*offset.y, 0.0f), d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
	struct NuiCLVoxel v001 = trilinearInterpolationSimpleFastFast(pos+(float3)(0.0f, 0.0f, 0.5f*offset.z), d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);

	float3 grad = (float3)((vp00.sdf-v100.sdf)/offset.x, (v0p0.sdf-v010.sdf)/offset.y, (v00p.sdf-v001.sdf)/offset.z);

	float l = length(grad);
	if(l == 0.0f) {
		return (float3)(0.0f, 0.0f, 0.0f);
	}

	return -grad / l;
}

__kernel void renderKernel(
            __constant struct NuiCLCameraParams* cameraParams,
			__global struct NuiCLRigidTransform* matrix,
			__global float* vmap,
			__global float* nmap,
			__global float* colormap,
			__global struct NuiCLHashEntry*	d_hash,
			__global struct NuiCLVoxel*		d_SDFBlocks,
			const float		virtualVoxelSize,
			const uint		hashNumBuckets,
			const uint		hashMaxCollisionLinkedListSize,
			const float		thresSampleDist,
			const float		thresDist,
			const float		rayIncrement
        )
{
    const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
	const uint gsizex = get_global_size(0);
	const int idx = mul24(gidy, gsizex) + gidx;
						
	/*vstore3(NAN, idx, vmap);
	vstore3(NAN, idx, nmap);
	vstore4(NAN, idx, colormap);*/

	float3 camDir = normalize(kinectProjToCamera(gidx, gidy, 1.0f, cameraParams));
	float3 worldCamPos = transform( (float3)(0.0f, 0.0f, 0.0f), matrix );
	float3 worldDir = normalize( rotation(camDir, matrix) );

	//float minInterval = tex2D(rayMinTextureRef, x, y);
	//float maxInterval = tex2D(rayMaxTextureRef, x, y);
	struct NuiCLCameraParams camParams = *cameraParams;
	float minInterval = camParams.sensorDepthWorldMin;
	float maxInterval = camParams.sensorDepthWorldMax;

	//if (minInterval == 0 || minInterval == MINF) minInterval = rayCastParams.m_minDepth;
	//if (maxInterval == 0 || maxInterval == MINF) maxInterval = rayCastParams.m_maxDepth;
	//TODO MATTHIAS: shouldn't this return in the case no interval is found?
	if (minInterval == 0 || minInterval == FP_MINF) return;
	if (maxInterval == 0 || maxInterval == FP_MINF) return;

	// debugging 
	//if (maxInterval < minInterval) {
	//	printf("ERROR (%d,%d): [ %f, %f ]\n", x, y, minInterval, maxInterval);
	//}

	// Last Sample
	float lastSampleSdf = 0.0f;
	float lastSampleAlpha = 0.0f;
	float lastSampleWeight = 0;
	const float depthToRayLength = 1.0f/camDir.z; // scale factor to convert from depth to ray length

	float rayCurrent = depthToRayLength * max(camParams.sensorDepthWorldMin, minInterval);	// Convert depth to raylength
	float rayEnd = depthToRayLength * min(camParams.sensorDepthWorldMax, maxInterval);		// Convert depth to raylength
	//float rayCurrent = depthToRayLength * rayCastParams.m_minDepth;	// Convert depth to raylength
	//float rayEnd = depthToRayLength * rayCastParams.m_maxDepth;		// Convert depth to raylength

	while(rayCurrent < rayEnd)
	{
		float3 currentPosWorld = worldCamPos + rayCurrent * worldDir;
		struct NuiCLVoxel interp = trilinearInterpolationSimpleFastFast(currentPosWorld, d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
		if(interp.weight)
		{
			if(lastSampleWeight > 0 && lastSampleSdf > 0.0f && interp.sdf < 0.0f) // current sample is always valid here 
			{
				// = findIntersectionLinear(lastSample.alpha, rayCurrent, lastSample.sdf, dist);
				struct NuiCLVoxel intersection = findIntersectionBisection( worldCamPos, worldDir, d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize, lastSampleSdf, lastSampleAlpha, interp.sdf, rayCurrent);
					
				float3 currentIso = worldCamPos + intersection.sdf * worldDir;
				if(intersection.weight && fabs(lastSampleSdf - interp.sdf) < thresSampleDist)
				{
					if(fabs(interp.sdf) < thresDist)
					{
						float depth = intersection.sdf / depthToRayLength; // Convert ray length to depth depthToRayLength

						vstore3( kinectDepthToSkeleton(gidx, gidy, depth, cameraParams) , idx, vmap);
						if(colormap)
							vstore4( (float4)(intersection.color[0]/255.f, intersection.color[1]/255.f, intersection.color[2]/255.f, 1.0f) , idx, colormap);

						//if(useGradients)
						{
							float3 normal = -gradientForPoint(currentIso, d_hash, d_SDFBlocks, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
							vstore3( rotationInverse(normal, matrix) , idx, nmap);
						}

						return;
					}
				}
			}

			lastSampleSdf = interp.sdf;
			lastSampleAlpha = rayCurrent;
			// lastSample.color = color;
			lastSampleWeight = 1;
			rayCurrent += rayIncrement;
		} else {
			lastSampleWeight = 0;
			rayCurrent += rayIncrement;
		}
	}
}

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

__kernel void rayIntervalSplatKernel(
			__global float*					rayMin,
			__global float*					rayMax,
            __constant struct NuiCLCameraParams* cameraParams,
			__global struct NuiCLRigidTransform* matrix,
			__global struct NuiCLHashEntry*	d_hashCompactified,
			const float		virtualVoxelSize
        )
{
	const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
	const uint gsizex = get_global_size(0);
	const int idx = mul24(gidy, gsizex) + gidx;

	struct NuiCLHashEntry entry = d_hashCompactified[gidx];
	if (entry.ptr == FREE_ENTRY)
		return;

	int3 block;
	block.x = entry.pos[0];
	block.y = entry.pos[1];
	block.z = entry.pos[2];
	if (!isSDFBlockInCameraFrustumApprox(block, cameraParams, matrix, virtualVoxelSize))
		return;

	float3 worldCurrentVoxel = SDFBlockToWorld(block, virtualVoxelSize);
	float3 MINV = worldCurrentVoxel - virtualVoxelSize / 2.0f;
	float3 maxv = MINV + SDF_BLOCK_SIZE * virtualVoxelSize;

	float3 proj000 = cameraToKinectProj(transformInverse((float3)(MINV.x, MINV.y, MINV.z), matrix), cameraParams);
	float3 proj100 = cameraToKinectProj(transformInverse((float3)(maxv.x, MINV.y, MINV.z), matrix), cameraParams);
	float3 proj010 = cameraToKinectProj(transformInverse((float3)(MINV.x, maxv.y, MINV.z), matrix), cameraParams);
	float3 proj001 = cameraToKinectProj(transformInverse((float3)(MINV.x, MINV.y, maxv.z), matrix), cameraParams);
	float3 proj110 = cameraToKinectProj(transformInverse((float3)(maxv.x, maxv.y, MINV.z), matrix), cameraParams);
	float3 proj011 = cameraToKinectProj(transformInverse((float3)(MINV.x, maxv.y, maxv.z), matrix), cameraParams);
	float3 proj101 = cameraToKinectProj(transformInverse((float3)(maxv.x, MINV.y, maxv.z), matrix), cameraParams);
	float3 proj111 = cameraToKinectProj(transformInverse((float3)(maxv.x, maxv.y, maxv.z), matrix), cameraParams);

	// Tree Reduction Min
	float3 min00 = fmin(proj000, proj100);
	float3 min01 = fmin(proj010, proj001);
	float3 min10 = fmin(proj110, proj011);
	float3 min11 = fmin(proj101, proj111);

	float3 min0 = fmin(min00, min01);
	float3 min1 = fmin(min10, min11);

	float3 minFinal = fmin(min0, min1);
	vstore(minFinal.z, idx, rayMin);

	// Tree Reduction Max
	float3 max00 = fmax(proj000, proj100);
	float3 max01 = fmax(proj010, proj001);
	float3 max10 = fmax(proj110, proj011);
	float3 max11 = fmax(proj101, proj111);

	float3 max0 = fmax(max00, max01);
	float3 max1 = fmax(max10, max11);

	float3 maxFinal = fmax(max0, max1);
	vstore(maxFinal.z, idx, rayMax);
}
