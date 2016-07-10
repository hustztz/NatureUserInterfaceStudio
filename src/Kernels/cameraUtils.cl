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
