#include "utils.cl"
#include "gpu_def.h"

__kernel void fetchVolumeKernel(
                    __global short* volume,
					__global uchar* color_volume,
                    __constant struct TsdfParams* params,
					int			range_z,
                    __global float* vmap,
					__global float* colormap,
					__global volatile int* vertex_id
                    )
{
	const int voxel_x = get_global_id(0);
	const int voxel_y = get_global_id(1);
	
	struct TsdfParams l_params = *params;
	const int resolution_x = convert_int(l_params.resolution[0]);
	const int resolution_y = convert_int(l_params.resolution[1]);
	const int resolution_z = convert_int(l_params.resolution[2]);

	const float half_x = convert_float(resolution_x >> 1);
	const float half_y = convert_float(resolution_y >> 1);

	const int voxel_id = mul24((mul24(voxel_y, resolution_x) + voxel_x), resolution_z);

	float tsdf = NAN;
	for (int voxel_z = 0; voxel_z < range_z; ++ voxel_z)
	{
		float tsdf_prev = tsdf;
		const idx = voxel_id + voxel_z;
		short2 value = vload2(idx, volume);
		if(unpack_tsdf_weight(value) <= 0)
		{
			tsdf = NAN;
			continue;
		}

		tsdf = unpack_tsdf_dp( value );
		if(0.f == tsdf)
		{
			float3 vertex_value = (float3)((convert_float(voxel_x) + 0.5f - half_x)*l_params.cell_size[0], (-convert_float(voxel_y) - 0.5f + half_y)*l_params.cell_size[1], (convert_float(voxel_z) + 0.5f)*l_params.cell_size[2]);
			float4 color_value = (float4)(0.0f, 0.0f, 0.0f, 1.0f);
			if(color_volume)
			{
				uchar4 color_gbra = vload4(idx, color_volume);
				color_value = (float4)(convert_float(color_gbra.x) /255.f, convert_float(color_gbra.y) /255.f, convert_float(color_gbra.z) /255.f, 1.0f);
			}

			int current_id = atomic_inc(vertex_id);
			if(current_id < MAX_OUTPUT_VERTEX_SIZE)
			{
				vstore3(vertex_value, current_id, vmap);
				vstore4(color_value, current_id, colormap);
			}
		}
		else if ((tsdf_prev < 0.f && tsdf > 0.f) || (tsdf_prev > 0.f && tsdf < 0.f))
		{
			float tsdf_inv = 1.f / fabs(tsdf);
			float tsdf_prev_inv = 1.f / fabs(tsdf_prev);
			float dp = (tsdf_inv * convert_float(voxel_z) + tsdf_prev_inv * convert_float(voxel_z - 1)) / (tsdf_inv + tsdf_prev_inv);
			float3 vertex_value = (float3)((convert_float(voxel_x) + 0.5f - half_x)*l_params.cell_size[0], (-convert_float(voxel_y) - 0.5f + half_y)*l_params.cell_size[1], (dp + 0.5f)*l_params.cell_size[2]);
			float4 color_value = (float4)(0.0f, 0.0f, 0.0f, 1.0f);
			if(color_volume)
			{
				uchar4 color_gbra = vload4(idx, color_volume);
				color_value = (float4)(convert_float(color_gbra.z) /255.f, convert_float(color_gbra.y) /255.f, convert_float(color_gbra.x) /255.f, 1.0f);
			}

			int current_id = atomic_inc(vertex_id);
			if(current_id < MAX_OUTPUT_VERTEX_SIZE)
			{
				vstore3(vertex_value, current_id, vmap);
				vstore4(color_value, current_id, colormap);
			}
		}
	}
}

__kernel void fetchShiftVolumeKernel(
                    __global short* volume,
					__global uchar* color_volume,
                    __constant struct TsdfParams* params,
					int			range_z,
					const int3  voxelWrap,
					const int3  voxelOffset,
                    __global float* vmap,
					__global float* colormap,
					__global volatile int* vertex_id
                    )
{
	const int voxel_x = get_global_id(0);
	const int voxel_y = get_global_id(1);
	
	struct TsdfParams l_params = *params;
	const int resolution_x = convert_int(l_params.resolution[0]);
	const int resolution_y = convert_int(l_params.resolution[1]);
	const int resolution_z = convert_int(l_params.resolution[2]);

	const float half_x = convert_float(resolution_x >> 1);
	const float half_y = convert_float(resolution_y >> 1);

	const int voxel_id = mul24((mul24((voxel_y+voxelWrap.y)%resolution_y, resolution_x) + (voxel_x+voxelWrap.x)%resolution_x), resolution_z);

	int3 offsetBase = (int3)((voxelOffset.x/resolution_x)*resolution_x, (voxelOffset.y/resolution_y)*resolution_y, (voxelOffset.z/resolution_z)*resolution_z);
	if(voxelOffset.x > 0)
	{
		if(voxel_x < (voxelOffset.x % resolution_x))
			offsetBase.x += resolution_x;
	}
	else
	{
		if(voxel_x > (resolution_x + voxelOffset.x % resolution_x))
			offsetBase.x -= resolution_x;
	}
	if(voxelOffset.y > 0)
	{
		if(voxel_y < (voxelOffset.y % resolution_y))
			offsetBase.y += resolution_y;
	}
	else
	{
		if(voxel_y > (resolution_y + voxelOffset.y % resolution_y))
			offsetBase.y -= resolution_y;
	}

	range_z += voxelWrap.z;
	float tsdf = NAN;
	for (int voxel_z = voxelWrap.z; voxel_z < range_z; ++ voxel_z)
	{
		float tsdf_prev = tsdf;
		const idx = voxel_id+(voxel_z+voxelWrap.z)%resolution_z;
		short2 value = vload2(idx, volume);
		if(unpack_tsdf_weight(value) <= 0)
		{
			tsdf = NAN;
			continue;
		}

		int3 offset = offsetBase;
		if(voxelOffset.z > 0)
		{
			if(voxel_z < (voxelOffset.z % resolution_z))
				offset.z += resolution_z;
		}
		else
		{
			if(voxel_z > (resolution_z + voxelOffset.z % resolution_z))
				offset.z -= resolution_z;
		}

		tsdf = unpack_tsdf_dp( value );
		if(0.f == tsdf)
		{
			float3 vertex_value = (float3)((convert_float(voxel_x+offset.x) + 0.5f - half_x)*l_params.cell_size[0], (-convert_float(voxel_y+offset.y) - 0.5f + half_y)*l_params.cell_size[1], (convert_float(voxel_z+offset.z) + 0.5f)*l_params.cell_size[2]);
			float4 color_value = (float4)(0.0f, 0.0f, 0.0f, 1.0f);
			if(color_volume)
			{
				uchar4 color_gbra = vload4(idx, color_volume);
				color_value = (float4)(convert_float(color_gbra.x) /255.f, convert_float(color_gbra.y) /255.f, convert_float(color_gbra.z) /255.f, 1.0f);
			}

			int current_id = atomic_inc(vertex_id);
			if(current_id < MAX_OUTPUT_VERTEX_SIZE)
			{
				vstore3(vertex_value, current_id, vmap);
				vstore4(color_value, current_id, colormap);
			}
		}
		else if ((tsdf_prev < 0.f && tsdf > 0.f) || (tsdf_prev > 0.f && tsdf < 0.f))
		{
			float tsdf_inv = 1.f / fabs(tsdf);
			float tsdf_prev_inv = 1.f / fabs(tsdf_prev);
			float dp = (tsdf_inv * convert_float(voxel_z+offset.z) + tsdf_prev_inv * convert_float(voxel_z+offset.z - 1)) / (tsdf_inv + tsdf_prev_inv);
			float3 vertex_value = (float3)((convert_float(voxel_x+offset.x) + 0.5f - half_x)*l_params.cell_size[0], (-convert_float(voxel_y+offset.y) - 0.5f + half_y)*l_params.cell_size[1], (dp + 0.5f)*l_params.cell_size[2]);
			float4 color_value = (float4)(0.0f, 0.0f, 0.0f, 1.0f);
			if(color_volume)
			{
				uchar4 color_gbra = vload4(idx, color_volume);
				color_value = (float4)(convert_float(color_gbra.z) /255.f, convert_float(color_gbra.y) /255.f, convert_float(color_gbra.x) /255.f, 1.0f);
			}

			int current_id = atomic_inc(vertex_id);
			if(current_id < MAX_OUTPUT_VERTEX_SIZE)
			{
				vstore3(vertex_value, current_id, vmap);
				vstore4(color_value, current_id, colormap);
			}
		}
	}
}