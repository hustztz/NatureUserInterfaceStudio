
#include "utils.cl"

inline static bool is_valid_position(float depth, float trunc_min, float trunc_max)
{
	return (depth > trunc_min) && (depth < trunc_max);
}

__kernel void depth_passing_filter_kernel(
            __global ushort*		depths,
			__global float* filteredDepths,
			float			trunc_min,
			float			trunc_max
        )
{
    const uint gidx = get_global_id(0);
	float dp = convert_float(vload(gidx, depths))*0.001f;
	if( !is_valid_position(dp, trunc_min, trunc_max) )
	{
		dp = NAN;
	}
	vstore(dp, gidx, filteredDepths);
}
