#include "utils.cl"
#include "hashing_gpu_def.h"

#pragma OPENCL EXTENSION cl_khr_global_int32_base_atomics : enable

#define LOCK(a) atom_cmpxchg(a, 0, 1)
#define UNLOCK(a) atom_xchg(a, 0)

inline static void deleteVoxel(uint idx, __global struct NuiVoxel*	d_SDFBlocks)
{
	struct NuiVoxel voxel;
	voxel.sdf = 0.0f;
	voxel.weight = 0;
	voxel.color[0] = voxel.color[1] = voxel.color[2] = 0;

	d_SDFBlocks[idx] = voxel;
}

inline static void deleteHashEntry(uint idx, __global struct NuiHashEntry*	d_hash)
{
	struct NuiHashEntry hashEntry;
	hashEntry.ptr = FREE_ENTRY;
	hashEntry.offset = 0;
	hashEntry.pos[0] = hashEntry.pos[1] = hashEntry.pos[2] = 0;

	d_hash[idx] = hashEntry;
}

inline static int3 worldToSDFBlock(float3 pos, float virtualVoxelSize)
{
	float3 p = pos / virtualVoxelSize;
	int3 virtualVoxelPos = convert_int3(p + sign(p)*0.5f);

	if (virtualVoxelPos.x < 0) virtualVoxelPos.x -= SDF_BLOCK_SIZE-1;
	if (virtualVoxelPos.y < 0) virtualVoxelPos.y -= SDF_BLOCK_SIZE-1;
	if (virtualVoxelPos.z < 0) virtualVoxelPos.z -= SDF_BLOCK_SIZE-1;

	return (int3)(
		virtualVoxelPos.x/SDF_BLOCK_SIZE,
		virtualVoxelPos.y/SDF_BLOCK_SIZE,
		virtualVoxelPos.z/SDF_BLOCK_SIZE);
}

inline static float3 SDFBlockToWorld(int3 sdfBlock, float virtualVoxelSize)
{
	return convert_float3(sdfBlock * SDF_BLOCK_SIZE) * virtualVoxelSize;
}

inline static float2 cameraToKinectScreen(
			float3			pos,
			__constant struct CameraParams* cameraParams)
{
	struct CameraParams camParams = *cameraParams;
	return (float2)(
			pos.x*camParams.fx/pos.z + camParams.cx,			
			pos.y*camParams.fy/pos.z + camParams.cy);
}

inline static float3 cameraToKinectProj(
			float3			pos,
			__constant struct CameraParams* cameraParams)
{
	float2 proj = cameraToKinectScreen(pos, cameraParams);

	float3 pImage = (float3)(proj.x, proj.y, pos.z);

	struct CameraParams camParams = *cameraParams;
	pImage.x = (2.0f*pImage.x - (camParams.depthImageWidth- 1.0f))/(camParams.depthImageWidth- 1.0f);
	//pImage.y = (2.0f*pImage.y - (c_depthCameraParams.m_imageHeight-1.0f))/(c_depthCameraParams.m_imageHeight-1.0f);
	pImage.y = ((camParams.depthImageHeight-1.0f) - 2.0f*pImage.y)/(camParams.depthImageHeight-1.0f);
	pImage.z = (pImage.z - camParams.sensorDepthWorldMin)/(camParams.sensorDepthWorldMax - camParams.sensorDepthWorldMin);

	return pImage;
}

inline static bool isSDFBlockInCameraFrustumApprox(
			int3			sdfBlock,
			float			virtualVoxelSize,
			__constant struct CameraParams* cameraParams,
			__global struct RigidTransform* matrix)
{
	int3 pos = sdfBlock * SDF_BLOCK_SIZE + virtualVoxelSize * 0.5f * (SDF_BLOCK_SIZE - 1.0f);
	float3 p = convert_float3(pos) * virtualVoxelSize;
	float3 pCamera = transformInverse( p, matrix );
	float3 pProj = cameraToKinectProj(pCamera, cameraParams);
	//pProj *= 1.5f;	//TODO THIS IS A HACK FIX IT :)
	pProj *= 0.95;
	return !(pProj.x < -1.0f || pProj.x > 1.0f || pProj.y < -1.0f || pProj.y > 1.0f || pProj.z < 0.0f || pProj.z > 1.0f);
}