#pragma OPENCL EXTENSION cl_khr_global_int32_base_atomics : enable

#define LOCK(a) atom_cmpxchg(a, 0, 1)
#define UNLOCK(a) atom_xchg(a, 0)

#define SHORT_NAN 32767
#define SHORT_MAX 32766


// m is column wise, i.e m[0] is column 0, m[1] column 1, etc.
inline static bool _isnan3(float3 v)
{
	return isnan(v.x) || isnan(v.y) || isnan(v.z);
}

inline static bool _isnan4(float4 v)
{
	return isnan(v.x) || isnan(v.y) || isnan(v.z) || isnan(v.w);
}

inline static bool _isnan2(float2 v)
{
	return isnan(v.x) || isnan(v.y);
}

inline static void atomicAdd(__global volatile float* address, const float operand)
{
    union { uint uintVal; float floatVal; } newVal;
    union { uint uintVal; float floatVal; } localVal;
    do
    {
        localVal.floatVal = *address;
        newVal.floatVal = localVal.floatVal + operand;
    } while (atomic_cmpxchg((__global volatile uint*)address, localVal.uintVal, newVal.uintVal) != localVal.uintVal);
}

static float3 rotate3(float3 vec, float8 R1, float R2)
{
    return (float3)(dot(R1.s012, vec), dot(R1.s345, vec), dot((float3)(R1.s67,R2), vec));
}

static inline short2 pack_tsdf (float tsdf, short weight)
{
	short fixedp = convert_short (tsdf * convert_float(SHORT_MAX));
	//int fixedp = __float2int_rz(tsdf * DIVISOR);
	return (short2)(fixedp, weight);
}

static inline float unpack_tsdf_dp (short2 value)
{
	return (convert_float(value.x) / convert_float(SHORT_MAX));
}

static inline short unpack_tsdf_weight (short2 value)
{
	return value.y;
}
