#include "cameraUtils.cl"
#include "hashing_gpu_def.h"

inline static short3 worldToVoxelPos(float3 pos)
{
	return convert_int3(pos + convert_float3(sign(pos))*0.5f);
}

inline static uint computeHashIndex(short3 virtualVoxelPos)
{
	const uint p0 = 73856093u;
	const uint p1 = 19349669u;
	const uint p2 = 83492791u;

	return ((convert_uint(virtualVoxelPos.x) * p0) ^ (convert_uint(virtualVoxelPos.y) * p1) ^ (convert_uint(virtualVoxelPos.z) * p2));
}

inline static int3 virtualPosToVoxelBlock(int3 virtualVoxelPos)
{
	if (virtualVoxelPos.x < 0) virtualVoxelPos.x -= SDF_BLOCK_SIZE-1;
	if (virtualVoxelPos.y < 0) virtualVoxelPos.y -= SDF_BLOCK_SIZE-1;
	if (virtualVoxelPos.z < 0) virtualVoxelPos.z -= SDF_BLOCK_SIZE-1;

	return (int3)(
		virtualVoxelPos.x/SDF_BLOCK_SIZE,
		virtualVoxelPos.y/SDF_BLOCK_SIZE,
		virtualVoxelPos.z/SDF_BLOCK_SIZE);
}
