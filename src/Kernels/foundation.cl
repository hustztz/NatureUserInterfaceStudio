

__kernel void reset_uchar_kernel(
			__global uchar*	d_out
        )
{
    d_out[get_global_id(0)] = 0;
}

__kernel void set_float2_kernel(
			__global float2*	d_out,
			const	float		first,
			const	float		second
        )
{
    const uint gidx = get_global_id(0);
	d_out[gidx].x = first;
	d_out[gidx].y = second;
}