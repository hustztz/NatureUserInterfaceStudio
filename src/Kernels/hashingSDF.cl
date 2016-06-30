#include "hashingBlock.cl"

__kernel void alloc_SDFs_kernel(
            __global float* depths,
			const float		maxIntegrationDistance,
			const float		truncation,
			const float		truncScale,
			const float		virtualVoxelSize,
			__constant struct CameraParams* cameraParams,
			__global struct RigidTransform* matrix
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

	struct CameraParams camParams = *cameraParams;
	float3 screenToWorld = float3((gidx-camParams.cx)*camParams.fx_inv, (gidy-camParams.cy)*camParams.fy_inv, 1.0);
	float3 rayMin = screenToWorld * minDepth;
	rayMin = transform( rayMin, matrix );
	float3 rayMax = screenToWorld * maxDepth;
	rayMax = transform( rayMax, matrix );

	float3 rayDir = normalize(rayMax - rayMin);
	
	int3 idCurrentVoxel = worldToSDFBlock(rayMin, virtualVoxelSize);
	int3 idEnd = worldToSDFBlock(rayMax, virtualVoxelSize);
		
	float3 step = float3(sign(rayDir));
	float3 boundaryPos = SDFBlockToWorld(idCurrentVoxel+int3(clamp(step, 0.0, 1.0f)), virtualVoxelSize)-0.5f*virtualVoxelSize;
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
		if (hashData.isSDFBlockInCameraFrustumApprox(idCurrentVoxel, virtualVoxelSize, camParams, matrix) && !isSDFBlockStreamedOut(idCurrentVoxel, hashData, d_bitMask)) {		
			hashData.allocBlock(idCurrentVoxel);
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
