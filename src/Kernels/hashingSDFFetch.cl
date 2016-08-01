#include "utils.cl"
#include "hashingSDFUtils.cl"

__kernel void fetch_SDFs_kernel(
			__global struct NuiCLHashEntry*	d_hash,
			__global struct NuiCLVoxel*		d_SDFBlocks,
			const float						virtualVoxelSize,
			const float						thresSDF,
            __global float*					d_vmap,
			__global float*					d_colormap,
			__global volatile int*			vertex_id
        )
{
	const uint idxBlock = get_group_id(0);

	struct NuiCLHashEntry entry = d_hash[idxBlock];
	if (entry.ptr == FREE_ENTRY)
		return;
	
	const uint lidx = get_local_id(0);	//inside of an SDF block
	const uint voxelIdx = entry.ptr + lidx;
	struct NuiCLVoxel voxel = d_SDFBlocks[voxelIdx];
	if (voxel.weight == 0 || voxel.sdf > thresSDF)
		return;
	
	const int3 sdfBlock;
	sdfBlock.x = entry.pos[0];
	sdfBlock.y = entry.pos[1];
	sdfBlock.z = entry.pos[2];
	int3 pi_base = SDFBlockToVirtualVoxelPos(sdfBlock);

	int3 pi = pi_base + delinearizeVoxelIndex(lidx);
	float3 worldPos = virtualVoxelPosToWorld(pi, virtualVoxelSize);
	worldPos.y = - worldPos.y;
	float4 color_value = (float4)(convert_float(voxel.color[0]) /255.f, convert_float(voxel.color[1]) /255.f, convert_float(voxel.color[2]) /255.f, 1.0f);

	int current_id = atomic_inc(vertex_id);
	if(current_id < MAX_OUTPUT_VERTEX_SIZE)
	{
		vstore3(worldPos, current_id, d_vmap);
		vstore4(color_value, current_id, d_colormap);
	}
}
