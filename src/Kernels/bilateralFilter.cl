#include "utils.cl"

#define sigma_space2_inv_half 0.0408f // 0.5f / (sigma_space * sigma_space) //const float sigma_space = 3.5;     // in pixels
#define sigma_depth2_inv_half 5.55555555f // 0.5f / (sigma_color * sigma_color) // const float sigma_color = 30;     //in mm
#define FILTER_RADIUS 5 //static_cast<int>(sigma_space * 1.5);

__kernel void bilateral_filter_kernel(
            __global float* vertices,
            __global float* filteredVertices,
			float           depthThreshold
        )
{
    const int gidx = get_global_id(0);
	const int gidy = get_global_id(1);
    const int gsizex = get_global_size(0);
	const int gsizey = get_global_size(1);

	const int centerId = mul24(gidy, gsizex)+gidx;
	float3 center = vload3(centerId, vertices);
	if( center.z > 0 )
	{
		const uint tx_min = max(gidx-FILTER_RADIUS, 0);
		const uint tx_max = min(gidx+FILTER_RADIUS+1, gsizex);
		const uint ty_min = max(gidy-FILTER_RADIUS, 0);
		const uint ty_max = min(gidy+FILTER_RADIUS+1, gsizey);

		float sumDepth = 0;
        float sumWeight = 0;

		for(int cy = ty_min; cy < ty_max; ++ cy)
		{
			for(int cx = tx_min; cx < tx_max; ++ cx)
			{
				const uint nearId = mul24(cy, gsizex)+cx;
				float3 near = vload3(nearId, vertices);
				if(!_isnan3(near) && fabs(center.z - near.z) < depthThreshold)
				{
					 float space2 = hypot(near.x - center.x, near.y - center.y);
					 float depth2 = (near.z - center.z) * (near.z - center.z);

					 float weight = exp (-(space2 * sigma_space2_inv_half + depth2 * sigma_depth2_inv_half));

					 sumDepth += near.z * weight;
					 sumWeight += weight;
				}
			}
		}
		float3 filteredPos = (float3)(center.xy, sumDepth/sumWeight);
		vstore3(filteredPos, centerId, filteredVertices);
	}
	else
	{
		float3 invalidPos = (float3)(NAN, NAN, NAN);
		vstore3(invalidPos, centerId, filteredVertices);
	}
}

__kernel void generate_gaussian_kernel(
	__global float* out,
	float			sigma_space,
	uint			radius)
{
	const int gidx = get_global_id(0);
    int x = gidx - (int)radius;
    vstore( exp(-(x * x) * sigma_space), gidx, out);
}

__kernel void bilateral_filter_depth_kernel(
            __global float* depths,
            __global float* filteredDepths,
			__constant float* gaussian,
			uint			radius,
			float			sigma_depth,
			float           depthThreshold
        )
{
    const int gidx = get_global_id(0);
	const int gidy = get_global_id(1);
    const int gsizex = get_global_size(0);
	const int gsizey = get_global_size(1);

	const int centerId = mul24(gidy, gsizex)+gidx;
	float center = vload(centerId, depths);
	if( !isnan(center) )
	{
		const int tx_min = max(gidx-convert_int(radius), 0);
		const int tx_max = min(gidx+convert_int(radius)+1, gsizex);
		const int ty_min = max(gidy-convert_int(radius), 0);
		const int ty_max = min(gidy+convert_int(radius)+1, gsizey);

		float sumDepth = 0;
        float sumWeight = 0;

		for(int cy = ty_min; cy < ty_max; ++ cy)
		{
			for(int cx = tx_min; cx < tx_max; ++ cx)
			{
				const int nearId = mul24(cy, gsizex)+cx;
				float near = vload(nearId, depths);
				if( isnan(near) )
					continue;
				float diff = fabs(center - near);
				if(near > 0.0f && diff < depthThreshold)
				{
					 float depth2 = diff * diff;
					 float weight = vload(cx+radius-gidx, gaussian) * vload(cy+radius-gidy, gaussian) * exp(-(depth2 * sigma_depth));

					 sumDepth += near * weight;
					 sumWeight += weight;
				}
			}
		}
		vstore(sumDepth/sumWeight, centerId, filteredDepths);
	}
	else
	{
		vstore(NAN, centerId, filteredDepths);
	}
}

