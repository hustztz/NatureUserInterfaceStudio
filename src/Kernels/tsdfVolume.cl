#include "utils.cl"
#include "gpu_def.h"

#define INV_DIV 3.051850947599719e-5f
#define MAX_WEIGHT 128
#define RGB_VIEW_ANGLE_WEIGHT 0.75f

__kernel void scaleDepthsKernel(
                    __global	float*	depths,
					__constant	struct  NuiCLCameraParams* cameraParams
                    )
{
    const int gidx = get_global_id(0);
	const int gidy = get_global_id(1);
	
	const int gsizex = get_global_size(0);
    
	const int id = mul24(gidy, gsizex) + gidx;
	float Dp = vload(id, depths);
	if(isnan(Dp))
		return;

	struct NuiCLCameraParams camParams = *cameraParams;
    float xl = (convert_float(gidx) - camParams.cx) * camParams.fx_inv;
	float yl = (convert_float(gidy) - camParams.cy) * camParams.fy_inv;
	float lambda = sqrt (xl * xl + yl * yl + 1.f);

	vstore(Dp * lambda, id, depths); //meters
}

__kernel void integrateTsdfVolumeKernel(
                    __global float* depths,
					__global float* normals,
					__global uchar* colors,
                    __constant	struct  NuiCLCameraParams* cameraParams,
					__global	struct	NuiCLRigidTransform* matrix,
					int3  voxelWrap,
					__global short* volume,
					__constant struct TsdfParams* params,
					__global uchar* color_volume,
					uchar max_color_weight
                    )
{
	struct TsdfParams l_params = *params;
	
	const int resolution_x = convert_int(l_params.resolution[0]);
	const int resolution_y = convert_int(l_params.resolution[1]);
	const int resolution_z = convert_int(l_params.resolution[2]);

    const int voxel_x = get_global_id(0);
	const int voxel_y = get_global_id(1);
	
	struct NuiCLRigidTransform mat = *matrix;
	const float v_g_x = (convert_float(voxel_x - resolution_x/2) + 0.5f) * l_params.cell_size[0] - mat.t[0];
	const float v_g_y = (convert_float(voxel_y - resolution_y/2) + 0.5f) * l_params.cell_size[1] - mat.t[1];
	float v_g_z = (0 + 0.5f) * l_params.cell_size[2] - mat.t[2];

	float v_g_part_norm = v_g_x * v_g_x + v_g_y * v_g_y;

	struct NuiCLCameraParams camParams = *cameraParams;
	float v_x = (mat.R_inv[0] * v_g_x + mat.R_inv[1] * v_g_y + mat.R_inv[2] * v_g_z) * camParams.fx;
	float v_y = (mat.R_inv[3] * v_g_x + mat.R_inv[4] * v_g_y + mat.R_inv[5] * v_g_z) * camParams.fy;
	float v_z = (mat.R_inv[6] * v_g_x + mat.R_inv[7] * v_g_y + mat.R_inv[8] * v_g_z);

	float z_scaled = 0;
	float tranc_dist_inv = 1.f / l_params.tranc_dist;

	float Rcurr_inv_0_z_scaled = mat.R_inv[2] * l_params.cell_size[2] * camParams.fx;
	float Rcurr_inv_1_z_scaled = mat.R_inv[5] * l_params.cell_size[2] * camParams.fy;

	int idxBase = mul24(((voxel_x+voxelWrap.x)%resolution_x + mul24((voxel_y+voxelWrap.y)%resolution_y, resolution_x)), resolution_z);

	//#pragma unroll
	for (int voxel_z = 0; voxel_z < resolution_z;
		++voxel_z,
		v_g_z += l_params.cell_size[2],
		z_scaled += l_params.cell_size[2],
		v_x += Rcurr_inv_0_z_scaled,
		v_y += Rcurr_inv_1_z_scaled)
	{
		float inv_z = 1.0f / (v_z + mat.R_inv[8] * z_scaled);
		// behind the camera
		if (inv_z < 0.f)
			continue;

		// project to current cam
		int2 coo = (int2)
		(
			round (v_x * inv_z + camParams.cx),
			round (v_y * inv_z + camParams.cy)
		);
		const int nWidth = camParams.depthImageWidth;
		const int nHeight = camParams.depthImageHeight;
		if (coo.x >= 0 && coo.y >= 0 && coo.x < nWidth && coo.y < nHeight)
		{
			int coo_id = mul24(coo.y, nWidth) + coo.x;
			float Dp_scaled = vload(coo_id, depths); //meters

			float sdf = Dp_scaled - sqrt (v_g_z * v_g_z + v_g_part_norm);

			if (!isnan(Dp_scaled) && sdf >= -l_params.tranc_dist) //meters
			{
				float tsdf = min (1.0f, sdf * tranc_dist_inv);
				const int idx = idxBase + (voxel_z + voxelWrap.z) % resolution_z;
				/*bool integrate = true;
				if ((voxel_x > 0 &&  voxel_x < volume_resolutions.x-2) && (voxel_y > 0 && voxel_y < volume_resolutions.y-2) && (voxel_z < volume_resolutions.z-2))
				{
					float3 normal = (float3)(NAN, NAN, NAN);

					short2 n = vload2(idx + 1, volume);
					short2 p = vload2(idx - 1, volume);
					if (unpack_tsdf_weight(n) > 16 && unpack_tsdf_weight(p) > 16) 
						normal.z = (unpack_tsdf_dp(n) - unpack_tsdf_dp(p)) / cell_size.z;

					const uint step_y = mul24(volume_resolutions.x, volume_resolutions.z);
					n = vload2(idx + step_y, volume);
					p = vload2(idx - step_y, volume);
					if (unpack_tsdf_weight(n) > 16 && unpack_tsdf_weight(p) > 16) 
						normal.y = (unpack_tsdf_dp(n) - unpack_tsdf_dp(p)) / cell_size.y;

					n = vload2(idx + volume_resolutions.z, volume);
					p = vload2(idx - volume_resolutions.z, volume);
					if (unpack_tsdf_weight(n) > 16 && unpack_tsdf_weight(p) > 16) 
						normal.x = (unpack_tsdf_dp(n) - unpack_tsdf_dp(p)) / cell_size.x;

					if(!_isnan3(normal))
					{
						float norm2 = dot(normal, normal);
						if (norm2 >= 1e-10)
						{
							normal *= rsqrt(norm2);

							float nt = v_g_x * normal.x + v_g_y * normal.y + v_g_z * normal.z;
							float cosine = nt * rsqrt(v_g_x * v_g_x + v_g_y * v_g_y + v_g_z * v_g_z);

							if (cosine < 0.5)
								integrate = false;
						}
					}
				}

				if (integrate)*/
				{
					//read and unpack
					short2 prev_value = vload2(idx, volume);

					const int Wrk = 1;

					float tsdf_new = (unpack_tsdf_dp(prev_value) * unpack_tsdf_weight(prev_value) + Wrk * tsdf) / (unpack_tsdf_weight(prev_value) + Wrk);
					short weight_new = min(unpack_tsdf_weight(prev_value) + Wrk, MAX_WEIGHT);

					vstore2( pack_tsdf(tsdf_new, weight_new), idx, volume );
				}
				// Color Volume
				if(color_volume)
				{
					if( fabs(sdf) < l_params.tranc_dist )
					{
						float3 ncurr = vload3(coo_id, normals);
						if( !_isnan3(ncurr) )
						{
							uchar4 new_color = vload4(coo_id, colors);
							if( 0 != new_color.w )
							{
								uchar4 volume_color = vload4(idx, color_volume);

								const float Wrk = min(1.0f, fabs(ncurr.z) / RGB_VIEW_ANGLE_WEIGHT) * 2.f;
								float new_x = (convert_float(volume_color.x * volume_color.w) + Wrk * convert_float(new_color.x)) / (convert_float(volume_color.w) + Wrk);
								float new_y = (convert_float(volume_color.y * volume_color.w) + Wrk * convert_float(new_color.y)) / (convert_float(volume_color.w) + Wrk);
								float new_z = (convert_float(volume_color.z * volume_color.w) + Wrk * convert_float(new_color.z)) / (convert_float(volume_color.w) + Wrk);
								uchar weight_new = volume_color.w + 1;

								uchar4 volume_rgbw_new;
								volume_rgbw_new.x = convert_uchar( clamp(new_x, 0.0f, 255.f) );
								volume_rgbw_new.y = convert_uchar( clamp(new_y, 0.0f, 255.f) );
								volume_rgbw_new.z = convert_uchar( clamp(new_z, 0.0f, 255.f) );
								volume_rgbw_new.w = min (max_color_weight, weight_new);

								vstore4( volume_rgbw_new, idx, color_volume );
							}
						}
					}
				}
			}
		}
	}       // for(int z = 0; z < VOLUME_Z; ++z)
}

__kernel void initializeVolume(__global short* volume,
							   __global char* color_volume,
							   __constant struct TsdfParams* params,
							   const int3			voxel_offset)
{
    const int gidx = get_global_id(0);
	const int gidy = get_global_id(1);
	const int gidz = get_global_id(2);

	struct TsdfParams l_params = *params;
	const int resolution_x = convert_int(l_params.resolution[0]);
	const int resolution_y = convert_int(l_params.resolution[1]);
	const int resolution_z = convert_int(l_params.resolution[2]);

	int idx = mul24((mul24((gidy+voxel_offset.y)%resolution_y, resolution_x) + (gidx+voxel_offset.x)%resolution_x), resolution_z) + (gidz+voxel_offset.z)%resolution_z;
	if(volume)
		vstore2((short2)(SHORT_NAN, 0), idx, volume);
	if(color_volume)
		vstore4((char4)(0, 0, 0, 0), idx, color_volume);
}

__kernel void initializeColorVolume( __global char* volume )
{
    const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
	const uint gidz = get_global_id(2);

	const uint gsizex = get_global_size(0);
	const uint gsizey = get_global_size(1);
    
	uint id = mul24((mul24(gidz, gsizey) + gidy), gsizex) + gidx;
	vstore4((char4)(0, 0, 0, 0), id, volume);
}
