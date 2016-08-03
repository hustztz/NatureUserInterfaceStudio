#include "utils.cl"
#include "gpu_def.h"

static inline float getMinTime (float* volume_max, float3 origin, float3 dir)
{
	float txmin = (fabs(dir.x) < 1e-14) ? 0.f : (((dir.x > 0 ? 0.f : volume_max[0]) - origin.x) / dir.x);
	float tymin = (fabs(dir.y) < 1e-14) ? 0.f : (((dir.y > 0 ? 0.f : volume_max[1]) - origin.y) / dir.y);
	float tzmin = (fabs(dir.z) < 1e-14) ? 0.f : (((dir.z > 0 ? 0.f : volume_max[2]) - origin.z) / dir.z);

	return max ( max (txmin, tymin), tzmin);
}

static inline float getMaxTime (float* volume_max, float3 origin, float3 dir)
{
	float txmax = (fabs(dir.x) < 1e-14) ? 1e14 : (((dir.x > 0 ? volume_max[0] : 0.f) - origin.x) / dir.x);
	float tymax = (fabs(dir.y) < 1e-14) ? 1e14 : (((dir.y > 0 ? volume_max[1] : 0.f) - origin.y) / dir.y);
	float tzmax = (fabs(dir.z) < 1e-14) ? 1e14 : (((dir.z > 0 ? volume_max[2] : 0.f) - origin.z) / dir.z);

	return min (min (txmax, tymax), tzmax);
}

static inline int3 getVoxel (float3 point, float* cell_size)
{
	int vx = round (point.x / cell_size[0]);        // round to negative infinity
	int vy = round (point.y / cell_size[1]);
	int vz = round (point.z / cell_size[2]);

	return (int3)(vx, vy, vz);
}

static inline bool  checkInds (int3 g, int3 resolution)
{
return (g.x >= 0 && g.y >= 0 && g.z >= 0 && g.x < resolution.x && g.y < resolution.y && g.z < resolution.z);
}

static float interpolateTrilineary (float3 point,
										   __global short* volume, int3 resolution, float* cell_size, int3  voxelWrap)
{
	int3 g = getVoxel (point, cell_size);

	if (g.x <= 0 || g.x >= resolution.x - 1)
	  return NAN;

	if (g.y <= 0 || g.y >= resolution.y - 1)
	  return NAN;

	if (g.z <= 0 || g.z >= resolution.z - 1)
	  return NAN;

	float vx = (g.x + 0.5f) * cell_size[0];
	float vy = (g.y + 0.5f) * cell_size[1];
	float vz = (g.z + 0.5f) * cell_size[2];

	g.x = (point.x < vx) ? (g.x - 1) : g.x;
	g.y = (point.y < vy) ? (g.y - 1) : g.y;
	g.z = (point.z < vz) ? (g.z - 1) : g.z;

	float a = (point.x - (g.x + 0.5f) * cell_size[0]) / cell_size[0];
	float b = (point.y - (g.y + 0.5f) * cell_size[1]) / cell_size[1];
	float c = (point.z - (g.z + 0.5f) * cell_size[2]) / cell_size[2];

	int idx = mul24((mul24((g.y+voxelWrap.y)%resolution.y, resolution.x) + (g.x+voxelWrap.x)%resolution.x), resolution.z) + (g.z+voxelWrap.z)%resolution.z;
	short2 value = vload2(idx, volume);
	float tsdf000 = unpack_tsdf_dp( value );
	if(unpack_tsdf_weight(value) <= 0 || isnan(tsdf000))
		return NAN;

	idx ++;
	value = vload2(idx, volume);
	float tsdf001 = unpack_tsdf_dp( value );
	if(unpack_tsdf_weight(value) <= 0 || isnan(tsdf001))
		return NAN;

	idx += resolution.z;
	value = vload2(idx, volume);
	float tsdf011 = unpack_tsdf_dp( value );
	if(unpack_tsdf_weight(value) <= 0 || isnan(tsdf011))
		return NAN;

	idx --;
	value = vload2(idx, volume);
	float tsdf010 = unpack_tsdf_dp( value );
	if(unpack_tsdf_weight(value) <= 0 || isnan(tsdf010))
		return NAN;

	idx += mul24(resolution.x, resolution.z);
	value = vload2(idx, volume);
	float tsdf110 = unpack_tsdf_dp( value );
	if(unpack_tsdf_weight(value) <= 0 || isnan(tsdf110))
		return NAN;

	idx ++;
	value = vload2(idx, volume);
	float tsdf111 = unpack_tsdf_dp( value );
	if(unpack_tsdf_weight(value) <= 0 || isnan(tsdf111))
		return NAN;

	idx = idx - resolution.z;
	value = vload2(idx, volume);
	float tsdf101 = unpack_tsdf_dp( value );
	if(unpack_tsdf_weight(value) <= 0 || isnan(tsdf101))
		return NAN;

	idx --;
	value = vload2(idx, volume);
	float tsdf100 = unpack_tsdf_dp( value );
	if(unpack_tsdf_weight(value) <= 0 || isnan(tsdf100))
		return NAN;

	return (
		tsdf000 * (1 - a) * (1 - b) * (1 - c) +
		tsdf001 * (1 - a) * (1 - b) * c +
		tsdf010 * (1 - a) * b * (1 - c) +
		tsdf011 * (1 - a) * b * c +
		tsdf100 * a * (1 - b) * (1 - c) +
		tsdf101 * a * (1 - b) * c +
		tsdf110 * a * b * (1 - c) +
		tsdf111 * a * b * c
			);
}

static float interpolateTrilineary2 (float3 origin, float3 dir, float time,
										   __global short* volume, int3 resolution, float* cell_size, int3  voxelWrap)
{
	return interpolateTrilineary (origin + dir * time, volume, resolution, cell_size, voxelWrap);
}

__kernel void raycastKernel(
                    __global	short* volume,
					__global	uchar* color_volume,
					__constant	struct TsdfParams* params,
                    __constant	struct  NuiCLCameraParams* cameraParams,
					__global	struct	NuiCLRigidTransform* matrix,
                    __global	float*	vmap,
					__global	float*	nmap,
					__global	uchar*	colormap,
					const		int3	voxelWrap
                    )
{
	const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
	
	const uint gsizex = get_global_size(0);
    
	uint id = mul24(gidy, gsizex) + gidx;

	vstore3(NAN, id, vmap);
	vstore3(NAN, id, nmap);
	if(colormap)
		vstore4(NAN, id, colormap);
	
	struct TsdfParams l_params = *params;
	float3 volumeOffset = (float3)(l_params.dimension[0]/2.0, l_params.dimension[1]/2.0, l_params.dimension[2]/4.0);

	struct NuiCLCameraParams camParams = *cameraParams;
	struct NuiCLRigidTransform mat = *matrix;
	float3 ray_start = (float3)(mat.t[0], mat.t[1], mat.t[2]);
	ray_start += volumeOffset;
	float3 ray_next = (float3)((convert_float(gidx) - camParams.cx) * camParams.fx_inv, (convert_float(gidy) - camParams.cy) * camParams.fy_inv, 1);
    ray_next = transform(ray_next, matrix);
	ray_next += volumeOffset;
	float3 ray_dir = normalize (ray_next - ray_start);
	
	//ensure that it isn't a degenerate case
	/*ray_dir.x = (ray_dir.x == 0.f) ? 1e-15 : ray_dir.x;
	ray_dir.y = (ray_dir.y == 0.f) ? 1e-15 : ray_dir.y;
	ray_dir.z = (ray_dir.z == 0.f) ? 1e-15 : ray_dir.z;*/

	// computer time when entry and exit volume
	float time_start_volume = getMinTime (l_params.dimension, ray_start, ray_dir);
	float time_exit_volume = getMaxTime (l_params.dimension, ray_start, ray_dir);

	time_start_volume = max (time_start_volume, 0.f); //in meters
	if (time_start_volume >= time_exit_volume)
	  return;
	
	const int3 resolutions = (int3)(convert_int(l_params.resolution[0]), convert_int(l_params.resolution[1]), convert_int(l_params.resolution[2]));
	float time_curr = time_start_volume;
	int3 g = getVoxel (ray_start + ray_dir * time_curr, l_params.cell_size);
	g.x = max (0, min (g.x, resolutions.x - 1));
	g.y = max (0, min (g.y, resolutions.y - 1));
	g.z = max (0, min (g.z, resolutions.z - 1));
	float tsdf = NAN;
	int idx = mul24((mul24((g.y+voxelWrap.y)%resolutions.y, resolutions.x) + (g.x+voxelWrap.x)%resolutions.x), resolutions.z) + (g.z+voxelWrap.z)%resolutions.z;
	short2 value = vload2(idx, volume);
	if(unpack_tsdf_weight(value) > 0)
		tsdf = unpack_tsdf_dp( value );

	//infinite loop guard
	const float max_time = 3 * (l_params.dimension[0] + l_params.dimension[1] + l_params.dimension[2]);
	const float time_step = l_params.tranc_dist * 0.8f;

	for (; time_curr < max_time; time_curr += time_step)
	{
		float tsdf_prev = tsdf;

		int3 g = getVoxel (  ray_start + ray_dir * (time_curr + time_step), l_params.cell_size  );
		if (!checkInds (g, resolutions))
			break;

		int idx = mul24((mul24((g.y+voxelWrap.y)%resolutions.y, resolutions.x) + (g.x+voxelWrap.x)%resolutions.x), resolutions.z) + (g.z+voxelWrap.z)%resolutions.z;
		short2 value = vload2(idx, volume);
		if(unpack_tsdf_weight(value) > 0)
			tsdf = unpack_tsdf_dp( value );
		else
			tsdf = NAN;

		if(isnan(tsdf) || isnan(tsdf_prev))
			continue;

		if (tsdf_prev < 0.f && tsdf > 0.f)
			break;

		if (tsdf_prev >= 0.f && tsdf <= 0.f)           //zero crossing
		{
			float Ftdt = interpolateTrilineary2 (ray_start, ray_dir, time_curr + time_step, volume, resolutions, l_params.cell_size, voxelWrap);
			if (isnan (Ftdt))
				break;

			float Ft = interpolateTrilineary2 (ray_start, ray_dir, time_curr, volume, resolutions, l_params.cell_size, voxelWrap);
			if (isnan (Ft))
				break;

			//float Ts = time_curr - time_step * Ft/(Ftdt - Ft);
			float Ts = time_curr - time_step * Ft / (Ftdt - Ft);

			float3 vetex_found = ray_start + ray_dir * Ts;
			vstore3(vetex_found - volumeOffset, id, vmap);

			if(color_volume && colormap)
			{
				uchar4 rgba = vload4( idx, color_volume );
				vstore4(rgba, id, colormap);
			}

			int3 g = getVoxel ( ray_start + ray_dir * time_curr, l_params.cell_size );
			if (g.x > 1 && g.y > 1 && g.z > 1 && g.x < resolutions.x - 2 && g.y < resolutions.y - 2 && g.z < resolutions.z - 2)
			{
				float3 t;
				float3 n;

				t = vetex_found;
				t.x += l_params.cell_size[0];
				float Fx1 = interpolateTrilineary (t, volume, resolutions, l_params.cell_size, voxelWrap);

				t = vetex_found;
				t.x -= l_params.cell_size[0];
				float Fx2 = interpolateTrilineary (t, volume, resolutions, l_params.cell_size, voxelWrap);

				n.x = (Fx1 - Fx2);

				t = vetex_found;
				t.y += l_params.cell_size[1];
				float Fy1 = interpolateTrilineary (t, volume, resolutions, l_params.cell_size, voxelWrap);

				t = vetex_found;
				t.y -= l_params.cell_size[1];
				float Fy2 = interpolateTrilineary (t, volume, resolutions, l_params.cell_size, voxelWrap);

				n.y = (Fy1 - Fy2);

				t = vetex_found;
				t.z += l_params.cell_size[2];
				float Fz1 = interpolateTrilineary (t, volume, resolutions, l_params.cell_size, voxelWrap);

				t = vetex_found;
				t.z -= l_params.cell_size[2];
				float Fz2 = interpolateTrilineary (t, volume, resolutions, l_params.cell_size, voxelWrap);

				n.z = (Fz1 - Fz2);

				vstore3(normalize(n), id, nmap);
			}
			break;
		}
	}          /* for(;;)  */
}


__kernel void volume2vmapKernel(
                    __global short* volume,
                    __constant struct TsdfParams* params,
					__global uchar* color_volume,
                    __global float* vmap,
					__global float* colormap,
					const int max_vertex_id,
					volatile __global int* mutex,
					__global int* vertex_id
                    )
{
	const uint voxel_x = get_global_id(0);
	const uint voxel_y = get_global_id(1);
	
	const uint resolution_x = get_global_size(0);

	struct TsdfParams l_params = *params;
	const uint resolution_z = convert_int(l_params.resolution[2]);
	const uint voxel_id = mul24((mul24(voxel_y, resolution_x) + voxel_x), resolution_z);

	float tsdf = NAN;
	for (uint voxel_z = 0; voxel_z < resolution_z; ++ voxel_z)
	{
		float tsdf_prev = tsdf;

		short2 value = vload2(voxel_id+voxel_z, volume);
		if(unpack_tsdf_weight(value) <= 0)
		{
			tsdf = NAN;
			continue;
		}

		tsdf = unpack_tsdf_dp( value );

		if(0.f == tsdf)
		{
			float3 vertex_value = (float3)(convert_float(voxel_x)*l_params.cell_size[0], convert_float(voxel_y)*l_params.cell_size[1], convert_float(voxel_z)*l_params.cell_size[2]);
			float4 color_value = (float4)(0.0f, 0.0f, 0.0f, 1.0f);
			if(color_volume)
			{
				uchar4 color_gbra = vload4(voxel_id+voxel_z, color_volume);
				color_value = (float4)(convert_float(color_gbra.x) /255.f, convert_float(color_gbra.y) /255.f, convert_float(color_gbra.z) /255.f, 1.0f);
			}

			while(LOCK(mutex));
			int current_id = *vertex_id;
			if(current_id < max_vertex_id)
			{
				vstore3(vertex_value, current_id, vmap);
				vstore4(color_value, current_id, colormap);
				*vertex_id = current_id + 1;
			}
			UNLOCK(mutex);
		}
		else if ((tsdf_prev < 0.f && tsdf > 0.f) || (tsdf_prev > 0.f && tsdf < 0.f))
		{
			float tsdf_inv = 1.f / fabs(tsdf);
			float tsdf_prev_inv = 1.f / fabs(tsdf_prev);
			float dp = (tsdf_inv * convert_float(voxel_z) + tsdf_prev_inv * convert_float(voxel_z - 1)) / (tsdf_inv + tsdf_prev_inv);
			float3 vertex_value = (float3)(convert_float(voxel_x)*l_params.cell_size[0], convert_float(voxel_y)*l_params.cell_size[1], dp*l_params.cell_size[2]);
			float4 color_value = (float4)(0.0f, 0.0f, 0.0f, 1.0f);
			if(color_volume)
			{
				uchar4 color_gbra = vload4(voxel_id+voxel_z, color_volume);
				color_value = (float4)(convert_float(color_gbra.z) /255.f, convert_float(color_gbra.y) /255.f, convert_float(color_gbra.x) /255.f, 1.0f);
			}

			while(LOCK(mutex));
			int current_id = *vertex_id;
			if(current_id < max_vertex_id)
			{
				vstore3(vertex_value, current_id, vmap);
				vstore4(color_value, current_id, colormap);
				*vertex_id = current_id + 1;
			}
			UNLOCK(mutex);
		}
	}
}
