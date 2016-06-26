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
			const float		intr_fx,
            const float		intr_fy,
			const float		intr_cx,
            const float		intr_cy)
{
	return (float2)(
			pos.x*intr_fx/pos.z + intr_cx,			
			pos.y*intr_fy/pos.z + intr_cy);
}

inline static float3 cameraToKinectProj(
			float3			pos,
			const float		intr_fx,
            const float		intr_fy,
			const float		intr_cx,
            const float		intr_cy,
			const float		imageWidth,
			const float		imageHeight)
{
	float2 proj = cameraToKinectScreen(pos, intr_fx, intr_fy, intr_cx, intr_cy);

	float3 pImage = (float3)(proj.x, proj.y, pos.z);

	pImage.x = (2.0f*pImage.x - (imageWidth- 1.0f))/(imageWidth- 1.0f);
	//pImage.y = (2.0f*pImage.y - (c_depthCameraParams.m_imageHeight-1.0f))/(c_depthCameraParams.m_imageHeight-1.0f);
	pImage.y = ((imageHeight-1.0f) - 2.0f*pImage.y)/(imageHeight-1.0f);
	pImage.z = (pImage.z - c_depthCameraParams.m_sensorDepthWorldMin)/(c_depthCameraParams.m_sensorDepthWorldMax - c_depthCameraParams.m_sensorDepthWorldMin);

	return pImage;
}

inline static bool isSDFBlockInCameraFrustumApprox(
			int3			sdfBlock,
			float			virtualVoxelSize,
			const float8	Rcurr_inv1,
            const float		Rcurr_inv2,
			const float3	tcurr,)
{
	int3 pos = sdfBlock * SDF_BLOCK_SIZE + virtualVoxelSize * 0.5f * (SDF_BLOCK_SIZE - 1.0f);
	float3 p = convert_float3(pos) * virtualVoxelSize;
	float3 pCamera = rotate3( p, Rcurr_inv1, Rcurr_inv2 ) - tcurr;
	float3 pProj = cameraToKinectProj(pCamera);
	//pProj *= 1.5f;	//TODO THIS IS A HACK FIX IT :)
	pProj *= 0.95;
	return !(pProj.x < -1.0f || pProj.x > 1.0f || pProj.y < -1.0f || pProj.y > 1.0f || pProj.z < 0.0f || pProj.z > 1.0f);
}