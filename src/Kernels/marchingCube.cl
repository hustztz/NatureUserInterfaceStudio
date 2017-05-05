#include "utils.cl"
#include "gpu_def.h"

__kernel void volume2vmapKernel(
                    __global short* volume,
                    __constant struct TsdfParams* params,
					__global uchar* color_volume,
                    __global float* vmap,
					__global float* colormap,
					__global volatile int* vertex_id
                    )
{
	const uint voxel_x = get_global_id(0);
	const uint voxel_y = get_global_id(1);
	
	struct TsdfParams l_params = *params;
	const uint resolution_x = convert_uint(l_params.resolution[0]);
	const uint resolution_y = convert_uint(l_params.resolution[1]);
	const uint resolution_z = convert_uint(l_params.resolution[2]);
	
	const float half_x = convert_float(resolution_x >> 1);
	const float half_y = convert_float(resolution_y >> 1);
	const float half_z = convert_float(resolution_z >> 1);

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
			float3 vertex_value = (float3)((convert_float(voxel_x) + 0.5f - half_x)*l_params.cell_size[0], (-convert_float(voxel_y) - 0.5f + half_y)*l_params.cell_size[1], (convert_float(voxel_z) + 0.5f - half_z)*l_params.cell_size[2]);
			float4 color_value = (float4)(0.0f, 0.0f, 0.0f, 1.0f);
			if(color_volume)
			{
				uchar4 color_gbra = vload4(voxel_id+voxel_z, color_volume);
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
			float3 vertex_value = (float3)((convert_float(voxel_x) + 0.5f - half_x)*l_params.cell_size[0], (-convert_float(voxel_y) - 0.5f + half_y)*l_params.cell_size[1], (dp + 0.5f - half_z)*l_params.cell_size[2]);
			float4 color_value = (float4)(0.0f, 0.0f, 0.0f, 1.0f);
			if(color_volume)
			{
				uchar4 color_gbra = vload4(voxel_id+voxel_z, color_volume);
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

inline static float3 getNodeCoo (uint x, uint y, uint z, float* cell_size, float half_x, float half_y, float half_z)
{
	return (float3)((convert_float(x) + 0.5f - half_x) * cell_size[0], (-convert_float(y) - 0.5f + half_y) * cell_size[1], (convert_float(z) + 0.5f - half_z) * cell_size[2]);
}

inline static float3 vertex_interp (float3 p0, float3 p1, float f0, float f1)
{
	float t = (0.f - f0) / (f1 - f0 + 1e-15f);
	t = clamp(t, 0.0f, 1.0f);
	return mix(p0, p1, t);
}

inline static float4 color_interp (uchar4 c0, uchar4 c1, float f0, float f1)
{
	return ( fabs(f0) < fabs(f1) ) ? 
		(float4)(convert_float(c0.z) /255.f, convert_float(c0.y) /255.f, convert_float(c0.x) /255.f, 1) : 
		(float4)(convert_float(c1.z) /255.f, convert_float(c1.y) /255.f, convert_float(c1.x) /255.f, 1);
}

__kernel void marchingCubeKernel(
                    __global short* volume,
                    __constant struct TsdfParams* params,
					__global uchar* color_volume,
                    __global float* vmap,
					__global float* colormap,
					__constant int* numVertsTable,
					__constant int* triTable,
					__global volatile int* vertex_id
                    )
{
	const uint voxel_x = get_global_id(0);
	const uint voxel_y = get_global_id(1);
	
	struct TsdfParams l_params = *params;
	const uint resolution_x = convert_uint(l_params.resolution[0]);
	const uint resolution_y = convert_uint(l_params.resolution[1]);
	const uint resolution_z = convert_uint(l_params.resolution[2]);
	
	const float half_x = convert_float(resolution_x >> 1);
	const float half_y = convert_float(resolution_y >> 1);
	const float half_z = convert_float(resolution_z >> 1);

	const uint voxel_id0 = mul24((mul24(voxel_y, resolution_x) + voxel_x), resolution_z);
	const uint voxel_id1 = voxel_id0 + resolution_z;
	const uint voxel_id3 = voxel_id0 + mul24(resolution_x, resolution_z);
	const uint voxel_id2 = voxel_id3 + resolution_z;

	const uint range_z = resolution_z - 1;
	for (uint voxel_z = 0; voxel_z < range_z; ++ voxel_z)
	{
		short2 value0 = vload2(voxel_id0+voxel_z, volume);
		if(unpack_tsdf_weight(value0) <= 0)
			continue;
		short2 value1 = vload2(voxel_id1+voxel_z, volume);
		if(unpack_tsdf_weight(value1) <= 0)
			continue;
		short2 value2 = vload2(voxel_id2+voxel_z, volume);
		if(unpack_tsdf_weight(value2) <= 0)
			continue;
		short2 value3 = vload2(voxel_id3+voxel_z, volume);
		if(unpack_tsdf_weight(value3) <= 0)
			continue;
		short2 value4 = vload2(voxel_id0+voxel_z+1, volume);
		if(unpack_tsdf_weight(value4) <= 0)
			continue;
		short2 value5 = vload2(voxel_id1+voxel_z+1, volume);
		if(unpack_tsdf_weight(value5) <= 0)
			continue;
		short2 value6 = vload2(voxel_id2+voxel_z+1, volume);
		if(unpack_tsdf_weight(value6) <= 0)
			continue;
		short2 value7 = vload2(voxel_id3+voxel_z+1, volume);
		if(unpack_tsdf_weight(value7) <= 0)
			continue;
		
		float tsdf0 = unpack_tsdf_dp( value0 );
		float tsdf1 = unpack_tsdf_dp( value1 );
		float tsdf2 = unpack_tsdf_dp( value2 );
		float tsdf3 = unpack_tsdf_dp( value3 );
		float tsdf4 = unpack_tsdf_dp( value4 );
		float tsdf5 = unpack_tsdf_dp( value5 );
		float tsdf6 = unpack_tsdf_dp( value6 );
		float tsdf7 = unpack_tsdf_dp( value7 );

		// calculate flag indicating if each vertex is inside or outside isosurface
		int cubeindex;
		cubeindex = convert_int(tsdf0 < 0);
		cubeindex += convert_int(tsdf1 < 0) * 2;
		cubeindex += convert_int(tsdf2 < 0) * 4;
		cubeindex += convert_int(tsdf3 < 0) * 8;
		cubeindex += convert_int(tsdf4 < 0) * 16;
		cubeindex += convert_int(tsdf5 < 0) * 32;
		cubeindex += convert_int(tsdf6 < 0) * 64;
		cubeindex += convert_int(tsdf7 < 0) * 128;

		// read number of vertices from texture
        int numVerts = vload(cubeindex, numVertsTable);
		if(0 == numVerts)
			continue;

		// calculate cell vertex positions
        float3 v[8];
        v[0] = getNodeCoo (voxel_x,     voxel_y,     voxel_z,     l_params.cell_size, half_x, half_y, half_z);
        v[1] = getNodeCoo (voxel_x + 1, voxel_y,     voxel_z,     l_params.cell_size, half_x, half_y, half_z);
        v[2] = getNodeCoo (voxel_x + 1, voxel_y + 1, voxel_z,     l_params.cell_size, half_x, half_y, half_z);
        v[3] = getNodeCoo (voxel_x,     voxel_y + 1, voxel_z,     l_params.cell_size, half_x, half_y, half_z);
        v[4] = getNodeCoo (voxel_x,     voxel_y,     voxel_z + 1, l_params.cell_size, half_x, half_y, half_z);
        v[5] = getNodeCoo (voxel_x + 1, voxel_y,     voxel_z + 1, l_params.cell_size, half_x, half_y, half_z);
        v[6] = getNodeCoo (voxel_x + 1, voxel_y + 1, voxel_z + 1, l_params.cell_size, half_x, half_y, half_z);
        v[7] = getNodeCoo (voxel_x,     voxel_y + 1, voxel_z + 1, l_params.cell_size, half_x, half_y, half_z);

		float3 vertlist[12];
        vertlist[0] = vertex_interp (v[0], v[1], tsdf0, tsdf1);
        vertlist[1] = vertex_interp (v[1], v[2], tsdf1, tsdf2);
        vertlist[2] = vertex_interp (v[2], v[3], tsdf2, tsdf3);
        vertlist[3] = vertex_interp (v[3], v[0], tsdf3, tsdf0);
        vertlist[4] = vertex_interp (v[4], v[5], tsdf4, tsdf5);
        vertlist[5] = vertex_interp (v[5], v[6], tsdf5, tsdf6);
        vertlist[6] = vertex_interp (v[6], v[7], tsdf6, tsdf7);
        vertlist[7] = vertex_interp (v[7], v[4], tsdf7, tsdf4);
        vertlist[8] = vertex_interp (v[0], v[4], tsdf0, tsdf4);
        vertlist[9] = vertex_interp (v[1], v[5], tsdf1, tsdf5);
        vertlist[10] = vertex_interp (v[2], v[6], tsdf2, tsdf6);
        vertlist[11] = vertex_interp (v[3], v[7], tsdf3, tsdf7);

		// calculate colors
		uchar4 c[8];
		if(color_volume)
		{
			c[0] = vload4(voxel_id0+voxel_z, color_volume);
			c[1] = vload4(voxel_id1+voxel_z, color_volume);
			c[2] = vload4(voxel_id2+voxel_z, color_volume);
			c[3] = vload4(voxel_id3+voxel_z, color_volume);
			c[4] = vload4(voxel_id0+voxel_z+1, color_volume);
			c[5] = vload4(voxel_id1+voxel_z+1, color_volume);
			c[6] = vload4(voxel_id2+voxel_z+1, color_volume);
			c[7] = vload4(voxel_id3+voxel_z+1, color_volume);
		}

		float4 color[12];
		color[0] = color_interp (c[0], c[1], tsdf0, tsdf1);
        color[1] = color_interp (c[1], c[2], tsdf1, tsdf2);
        color[2] = color_interp (c[2], c[3], tsdf2, tsdf3);
        color[3] = color_interp (c[3], c[0], tsdf3, tsdf0);
        color[4] = color_interp (c[4], c[5], tsdf4, tsdf5);
        color[5] = color_interp (c[5], c[6], tsdf5, tsdf6);
        color[6] = color_interp (c[6], c[7], tsdf6, tsdf7);
        color[7] = color_interp (c[7], c[4], tsdf7, tsdf4);
        color[8] = color_interp (c[0], c[4], tsdf0, tsdf4);
        color[9] = color_interp (c[1], c[5], tsdf1, tsdf5);
        color[10] = color_interp (c[2], c[6], tsdf2, tsdf6);
        color[11] = color_interp (c[3], c[7], tsdf3, tsdf7);

		int current_id = atomic_add(vertex_id, numVerts);
		if(current_id+numVerts > MAX_OUTPUT_VERTEX_SIZE)
		{
			break;
		}
		for (int i = 0; i < numVerts; i += 3)
        {
			int table_index = mul24(cubeindex, 16) + i;
			int v1 = vload(table_index    , triTable);
			int v2 = vload(table_index + 1, triTable);
			int v3 = vload(table_index + 2, triTable);
			if(v1 < 0 || v1 >= 12)
				break;

			vstore3(vertlist[v1], current_id + i   , vmap);
			vstore3(vertlist[v2], current_id + i + 1, vmap);
			vstore3(vertlist[v3], current_id + i + 2, vmap);

			vstore4(color[v1], current_id + i   , colormap);
			vstore4(color[v2], current_id + i + 1, colormap);
			vstore4(color[v3], current_id + i + 2, colormap);
		}
	}
}

__kernel void volumeTraversalKernel(
                    __global short* volume,
                    const uint resolution_z,
                    __global short* voxel_nums
                    )
{
	const uint voxel_x = get_global_id(0);
	const uint voxel_y = get_global_id(1);
	
	const uint resolution_x = get_global_size(0);
	const uint id = mul24(voxel_y, resolution_x) + voxel_x;

	uint voxel_id = mul24(id, resolution_z);
	short sum = 0;

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

		if ((0.f == tsdf) || (tsdf_prev < 0.f && tsdf > 0.f) || (tsdf_prev > 0.f && tsdf < 0.f))
		{
			sum ++;
			if(SHORT_MAX == sum)
				break;
		}
	}
	vstore3(sum, id, voxel_nums);
}

__kernel void volume2VertexKernel(
                    __global short* volume,
					__constant struct TsdfParams* params,
                    __global uint* voxel_sums,
					const uint voxel_offset_x,
					const uint voxel_offset_y,
					__global float* vmap
                    )
{
	const uint voxel_x = get_global_id(0) + voxel_offset_x;
	const uint voxel_y = get_global_id(1) + voxel_offset_y;
	
	const uint resolution_x = get_global_size(0);
	const uint id = mul24(voxel_y, resolution_x) + voxel_x;

	struct TsdfParams l_params = *params;
	const uint resolution_z = convert_uint(l_params.resolution[2]);

	const uint voxel_id = mul24(id, resolution_z);
	uint vertex_id = vload(voxel_id + mul24(voxel_offset_y, resolution_x) + voxel_offset_x, voxel_sums);

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
			vstore3((float3)(convert_float(voxel_x)*l_params.cell_size[0], convert_float(voxel_y)*l_params.cell_size[1], convert_float(voxel_z)*l_params.cell_size[2]), vertex_id++, vmap);
		}
		else if ((tsdf_prev < 0.f && tsdf > 0.f) || (tsdf_prev > 0.f && tsdf < 0.f))
		{
			float tsdf_inv = 1.f / fabs(tsdf);
			float tsdf_prev_inv = 1.f / fabs(tsdf_prev);
			float dp = (tsdf_inv * convert_float(voxel_z) + tsdf_prev_inv * convert_float(voxel_z - 1)) / (tsdf_inv + tsdf_prev_inv);
			vstore3((float3)(convert_float(voxel_x)*l_params.cell_size[0], convert_float(voxel_y)*l_params.cell_size[1], dp*l_params.cell_size[2]), vertex_id++, vmap);
		}
	}
}