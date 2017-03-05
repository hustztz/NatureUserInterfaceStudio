
#include "utils.cl"

#define COLOR_CORESPS_NUM 28+1
#define WORK_GROUP_SIZE 128

__kernel void color_difference_kernel(
	const			int		div,
	__global		float*	vertices,
	__global		uchar*	colors,
	__global		uchar*	level_colors,
	__constant		struct  NuiCLCameraParams* cameraParams,
	const			float8	Rcurr1,
	const			float	Rcurr2,
	const			float3	tcurr,
	__global volatile float* color_dist,
	__global volatile float* validNum
	)
{
	const int gidx = get_global_id(0);

	float3 pt_model = vload3(gidx, vertices);
	uchar4 color_known = vload4(gidx, colors);

	if( _isnan3(pt_model) || color_known.w < 254 )
		return;

	float3 pt_camera = rotate3(pt_model, Rcurr1, Rcurr2) + tcurr;

	struct NuiCLCameraParams camParams = *cameraParams;
	const float intr_fx = camParams.fx / div;
	const float intr_fy = camParams.fy / div;
	const float intr_cx = camParams.cx / div;
	const float intr_cy = camParams.cy / div;
	const int nImageWidth = camParams.depthImageWidth / div;
	const int nImageHeight = camParams.depthImageHeight / div;
	int2 projPixel = (int2)(round(pt_camera.x * intr_fx / pt_camera.z + intr_cx), round(pt_camera.y * intr_fy / pt_camera.z + intr_cy));
	if(projPixel.x < 0 || 
		projPixel.x >= nImageWidth ||
		projPixel.y < 0 ||
		projPixel.y >= nImageHeight)
		return;

	const int level_idx = mul24(projPixel.y, nImageWidth)+projPixel.x;
	uchar4 color_obs = vload4(level_idx, level_colors);
	if(color_obs.w < 254)
		return;

	float3 color_diff = (float3)(
		convert_float(color_obs.x) - convert_float(color_known.x),
		convert_float(color_obs.y) - convert_float(color_known.y),
		convert_float(color_obs.z) - convert_float(color_known.z)
		);
	float dist = color_diff.x*color_diff.x + color_diff.y*color_diff.y + color_diff.z*color_diff.z;
	atomicAdd(color_dist, dist);
	atomicAdd(validNum, 1.0f);
}


static void reduce(	uint lid,
					uint gid,
					__local float* localBuffer,
					__global float* sumBuffer,
					uint local_size)
{
	for(uint d = local_size >> 1; d > 0; d >>= 1)
	{
		barrier(CLK_LOCAL_MEM_FENCE);
		
		if(lid < d)
		{
			uint a_stride = mul24(lid, (uint)COLOR_CORESPS_NUM);
			uint b_stride = mul24(lid + d, (uint)COLOR_CORESPS_NUM);
			for(uint i = 0; i < COLOR_CORESPS_NUM; i++)
				localBuffer[a_stride+i] += localBuffer[b_stride+i];
		}
	}
	
	barrier(CLK_LOCAL_MEM_FENCE);
	
	if(lid == 0)
	{
		uint group_stride = mul24(get_group_id(0), COLOR_CORESPS_NUM);
		// the last elements in tempA and tempB contain the sum of the work-group
		for(uint i = 0; i < COLOR_CORESPS_NUM; i++)
			sumBuffer[group_stride+i] = localBuffer[i];
	}
}


__kernel void color_icp_block_kernel(
			const			int		div,
			__constant		struct  NuiCLCameraParams* cameraParams,
            __global		float*	vertices,
            __global		uchar*	colors,
			__global		uchar*	level_colors,
			__global		uchar*	level_gradientXs,
			__global		uchar*	level_gradientYs,
			const			float8	Rcurr1,
			const			float	Rcurr2,
			const			float3	tcurr,
			const			int numPara,
			const			int startPara,
			__global		float*	corespsSumBuffer
        )
{
	__local float localBuffer[ WORK_GROUP_SIZE*COLOR_CORESPS_NUM ];	// hold the prefix sum for the current work group

	const uint lid = get_local_id(0);
	const uint gid = get_global_id(0);

	const uint local_id = mul24(lid, (uint)COLOR_CORESPS_NUM);
	// Initialize
	for (uint i = 0; i < COLOR_CORESPS_NUM; ++i)
    {
		vstore(0.0f, local_id + i, localBuffer);
    }

	float3 pt_model = vload3(gid, vertices);
	uchar4 color_known = vload4(gid, colors);

	if( !_isnan3(pt_model) && color_known.w == 255 )
	{
		float3 pt_camera = rotate3(pt_model, Rcurr1, Rcurr2) + tcurr;

		struct NuiCLCameraParams camParams = *cameraParams;
		const float intr_fx = camParams.fx / div;
		const float intr_fy = camParams.fy / div;
		const float intr_cx = camParams.cx / div;
		const float intr_cy = camParams.cy / div;
		const int nImageWidth = camParams.depthImageWidth / div;
		const int nImageHeight = camParams.depthImageHeight / div;
		int2 projPixel = (int2)(round(pt_camera.x * intr_fx / pt_camera.z + intr_cx), round(pt_camera.y * intr_fy / pt_camera.z + intr_cy));
		if(projPixel.x >= 0 && projPixel.x < nImageWidth && projPixel.y >= 0 && projPixel.y < nImageHeight)
		{
			const int level_idx = mul24(projPixel.y, nImageWidth)+projPixel.x;
			uchar4 gx_obs = vload4(level_idx, level_gradientXs);
			uchar4 gy_obs = vload4(level_idx, level_gradientYs);
			uchar4 color_obs = vload4(level_idx, level_colors);

			if(color_obs.w == 255 && gx_obs.w == 255 && gy_obs.w == 255)
			{
				float3 color_diff = (float3)(
						convert_float(color_obs.x) - convert_float(color_known.x),
						convert_float(color_obs.y) - convert_float(color_known.y),
						convert_float(color_obs.z) - convert_float(color_known.z)
						);

				float3 d_pt_cam_dpi;
				float3 d[6];
				int counter = 0;
				for (int para = 0; para < numPara; para++)
				{
					switch (para + startPara)
					{
					case 0: d_pt_cam_dpi.x = pt_camera.w;  d_pt_cam_dpi.y = 0.0f;         d_pt_cam_dpi.z = 0.0f;         break;
					case 1: d_pt_cam_dpi.x = 0.0f;         d_pt_cam_dpi.y = pt_camera.w;  d_pt_cam_dpi.z = 0.0f;         break;
					case 2: d_pt_cam_dpi.x = 0.0f;         d_pt_cam_dpi.y = 0.0f;         d_pt_cam_dpi.z = pt_camera.w;  break;
					case 3: d_pt_cam_dpi.x = 0.0f;         d_pt_cam_dpi.y = -pt_camera.z;  d_pt_cam_dpi.z = pt_camera.y;  break;
					case 4: d_pt_cam_dpi.x = pt_camera.z;  d_pt_cam_dpi.y = 0.0f;         d_pt_cam_dpi.z = -pt_camera.x;  break;
					default:
					case 5: d_pt_cam_dpi.x = -pt_camera.y;  d_pt_cam_dpi.y = pt_camera.x;  d_pt_cam_dpi.z = 0.0f;         break;
					};

					float2 d_proj_dpi;
					d_proj_dpi.x = intr_fx * ((pt_camera.z * d_pt_cam_dpi.x - d_pt_cam_dpi.z * pt_camera.x) / (pt_camera.z * pt_camera.z));
					d_proj_dpi.y = intr_fy * ((pt_camera.z * d_pt_cam_dpi.y - d_pt_cam_dpi.z * pt_camera.y) / (pt_camera.z * pt_camera.z));

					d[para].x = d_proj_dpi.x * convert_float(gx_obs.x) + d_proj_dpi.y * convert_float(gy_obs.x);
					d[para].y = d_proj_dpi.x * convert_float(gx_obs.y) + d_proj_dpi.y * convert_float(gy_obs.y);
					d[para].z = d_proj_dpi.x * convert_float(gx_obs.z) + d_proj_dpi.y * convert_float(gy_obs.z);

					for (int col = 0; col <= para; col++)
						localBuffer[counter++] = 2.0f * dot(d[para], d[col]);//(d[para].x * d[col].x + d[para].y * d[col].y + d[para].z * d[col].z);
					
					localBuffer[counter++] = dot(d[para], color_diff); //d[para].x * color_diff.x + d[para].y * color_diff.y + d[para].z * color_diff.z;
				}
				localBuffer[counter++] = 1.0f;
			}
		}
	}
	
	reduce(lid, gid, localBuffer, corespsSumBuffer, WORK_GROUP_SIZE);
}

__kernel void compute_color_sums(
									__global float* corespsBlocks,
									__global float* corespsBuffer,
												uint size
								)
{
	__local float localBuffer[ WORK_GROUP_SIZE * COLOR_CORESPS_NUM ];
	
	const uint lid = get_local_id(0);
	const uint thid = get_global_id(0);
	const uint lid_stride = mul24(lid, (uint)COLOR_CORESPS_NUM);
	const uint thid_stride = mul24(thid, (uint)COLOR_CORESPS_NUM);

	if(thid < size)
	{
		for(int i = 0; i < COLOR_CORESPS_NUM; i++)
		{
			localBuffer[lid_stride+i] = corespsBlocks[thid_stride+i];
		}
	}
	else 
	{
		for(int i = 0; i < COLOR_CORESPS_NUM; i++)
		{
			localBuffer[lid_stride+i] = 0.0f;
		}
	}

	barrier(CLK_LOCAL_MEM_FENCE);
	
	reduce(lid, thid, localBuffer, corespsBuffer, WORK_GROUP_SIZE);
}