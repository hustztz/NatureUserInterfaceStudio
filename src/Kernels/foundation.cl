

__kernel void reset_uchar_kernel(
			__global uchar*	d_out
        )
{
    d_out[get_global_id(0)] = 0;
}

__kernel void reset_short3_kernel(
			__global short3*	d_out
        )
{
    d_out[get_global_id(0)] = (short3)(0,0,0);
}

__kernel void set_float2_kernel(
			__global float2*	d_out,
			const	float		first,
			const	float		second
        )
{
    const uint gidx = get_global_id(0);
	d_out[gidx] = (float2)(first, second);
}

__kernel void invalid_float3_kernel(
			__global float3*	d_out
        )
{
    const uint gidx = get_global_id(0);
	d_out[gidx] = (float3)(NAN, NAN, NAN);
}