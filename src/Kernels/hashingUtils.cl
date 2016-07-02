#include "utils.cl"
#include "hashing_gpu_def.h"

#pragma OPENCL EXTENSION cl_khr_global_int32_base_atomics : enable

#define LOCK(a) atom_cmpxchg(a, 0, 1)
#define UNLOCK(a) atom_xchg(a, 0)



inline static float2 cameraToKinectScreen(
			float3			pos,
			__constant struct NuiCLCameraParams* cameraParams)
{
	struct NuiCLCameraParams camParams = *cameraParams;
	return (float2)(
			pos.x*camParams.fx/pos.z + camParams.cx,			
			pos.y*camParams.fy/pos.z + camParams.cy);
}

inline static float3 cameraToKinectProj(
			float3			pos,
			__constant struct NuiCLCameraParams* cameraParams)
{
	float2 proj = cameraToKinectScreen(pos, cameraParams);

	float3 pImage = (float3)(proj.x, proj.y, pos.z);

	struct NuiCLCameraParams camParams = *cameraParams;
	pImage.x = (2.0f*pImage.x - (camParams.depthImageWidth- 1.0f))/(camParams.depthImageWidth- 1.0f);
	//pImage.y = (2.0f*pImage.y - (c_depthCameraParams.m_imageHeight-1.0f))/(c_depthCameraParams.m_imageHeight-1.0f);
	pImage.y = ((camParams.depthImageHeight-1.0f) - 2.0f*pImage.y)/(camParams.depthImageHeight-1.0f);
	pImage.z = (pImage.z - camParams.sensorDepthWorldMin)/(camParams.sensorDepthWorldMax - camParams.sensorDepthWorldMin);

	return pImage;
}

inline static float cameraToKinectProjZ(
	float z,
	__constant struct NuiCLCameraParams* cameraParams)
{
	struct NuiCLCameraParams camParams = *cameraParams;
	return (z - camParams.sensorDepthWorldMin)/(camParams.sensorDepthWorldMax - camParams.sensorDepthWorldMin);
}

inline static float3 kinectDepthToSkeleton(uint ux, uint uy, float depth, __constant struct NuiCLCameraParams* cameraParams)
{
	struct NuiCLCameraParams camParams = *cameraParams;
	const float x = ((float)ux-camParams.cx) / camParams.fx;
	const float y = ((float)uy-camParams.cy) / camParams.fy;
	//const float y = (c_depthCameraParams.my-(float)uy) / c_depthCameraParams.fy;
	return (float3)(depth*x, depth*y, depth);
}

inline static float kinectProjToCameraZ(float z, __constant struct NuiCLCameraParams* cameraParams)
{
	struct NuiCLCameraParams camParams = *cameraParams;
	return z * (camParams.sensorDepthWorldMax - camParams.sensorDepthWorldMin) + camParams.sensorDepthWorldMin;
}

inline static float3 kinectProjToCamera(uint ux, uint uy, float z, __constant struct NuiCLCameraParams* cameraParams)
{
	float fSkeletonZ = kinectProjToCameraZ(z, cameraParams);
	return kinectDepthToSkeleton(ux, uy, fSkeletonZ, cameraParams);
}

inline static bool isSDFBlockInCameraFrustumApprox(
			int3			sdfBlock,
			float			virtualVoxelSize,
			__constant struct NuiCLCameraParams* cameraParams,
			__global struct NuiCLRigidTransform* matrix)
{
	int3 virtualVoxelPos = sdfBlock * SDF_BLOCK_SIZE;
	float3 posWorld = convert_float3(virtualVoxelPos) * virtualVoxelSize;
	posWorld = posWorld + virtualVoxelSize * 0.5f * (SDF_BLOCK_SIZE - 1.0f);
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
	int3 virtualVoxelPos = sdfBlock * SDF_BLOCK_SIZE;
	float3 posWorld = convert_float3(virtualVoxelPos) * virtualVoxelSize;
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