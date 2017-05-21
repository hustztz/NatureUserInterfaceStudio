
#include "utils.cl"

__kernel void mem_copy_kernel(
            __global float* src,
            __global float* dst
        )
{
    const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
	const uint gsizex = get_global_size(0);
	const uint index = mul24(gidy, gsizex)+gidx;

	float3 value = vload3(index, src);
	vstore3(value, index, dst);
}

__kernel void depth2vertex_kernel(
            __global float*			depths,
            __global float*			vertices,
			__constant struct NuiCLCameraParams* cameraParams,
			const int				div
        )
{
    const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
	const uint gsizex = get_global_size(0);
	const uint index = mul24(gidy, gsizex)+gidx;
	
	float3 vert = (float3)(NAN, NAN, NAN);
	float dp = vload(index, depths);
	if(!isnan(dp))
	{
		struct NuiCLCameraParams camParams = *cameraParams;
		const float intr_fx_inv = div * camParams.fx_inv;
		const float intr_fy_inv = div * camParams.fy_inv;
		const float intr_cx = camParams.cx / div;
		const float intr_cy = camParams.cy / div;
		vert = -dp * (float3)((convert_float(gidx)-intr_cx)*intr_fx_inv, (convert_float(gidy)-intr_cy)*intr_fy_inv, -1.0f);
	}
	vstore3(vert, index, vertices);
}

__kernel void half_sample_kernel(
            __global float* src,
            __global float* dst,
			uint			radius,
			float           depthThreshold
        )
{
    const int gidx = get_global_id(0);
	const int gidy = get_global_id(1);
    const int gsizex = get_global_size(0);
	const int gsizey = get_global_size(1);

	const int dstId = mul24(gidy, gsizex)+gidx;

	const int src_x = gidx << 1;
	const int src_y = gidy << 1;
	const int src_size_x = gsizex << 1;
	const int src_size_y = gsizey << 1;
	const int srcId = mul24(src_y, src_size_x)+src_x;
	float center = vload(srcId, src);
	
	const int tx_min = max(src_x-convert_int(radius), 0);
	const int tx_max = min(src_x+convert_int(radius)+1, src_size_x);
	const int ty_min = max(src_y-convert_int(radius), 0);
	const int ty_max = min(src_y+convert_int(radius)+1, src_size_y);

	float sumDepth = 0.0f;
    int sumWeight = 0;

	for(int cy = ty_min; cy < ty_max; ++ cy)
	{
		for(int cx = tx_min; cx < tx_max; ++ cx)
		{
			const int nearId = mul24(cy, src_size_x)+cx;
			float near = vload(nearId, src);
			if(!_isnan3(near) && (isnan(center) || fabs(center - near) < depthThreshold))
			{
				sumDepth += near;
				sumWeight += 1;
			}
		}
	}
	float outDepth = (sumWeight > 0) ? sumDepth/sumWeight : NAN;
	vstore(outDepth, dstId, dst);
}


__kernel void transform_maps_kernel(
            __global		float*	vertices,
            __global		float*	normals,
			__global		float*	verticesDst,
            __global		float*	normalsDst,
			__global struct NuiCLRigidTransform* matrix
        )
{
	const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
    const uint gsizex = get_global_size(0);

	uint id = mul24(gidy, gsizex)+gidx;
	float3 vmap_src = vload3(id, vertices);
	if( _isnan3(vmap_src) )
	{
		float3 invalidPos = (float3)(NAN, NAN, NAN);
		vstore3(invalidPos, id, verticesDst);
		float3 invalidNorm = (float3)(NAN, NAN, NAN);
		vstore3(invalidNorm, id, normalsDst);
		return;
	}

	float3 vmap_dst = transform(vmap_src, matrix);
	vstore3(vmap_dst, id, verticesDst);

	float3 nmap_src = vload3(id, normals);
	if( _isnan3(nmap_src) )
	{
		float3 invalidNorm = (float3)(NAN, NAN, NAN);
		vstore3(invalidNorm, id, normalsDst);
		return;
	}

	float3 nmap_dst = rotation(nmap_src, matrix);
	vstore3(nmap_dst, id, normalsDst);
}

__kernel void resize_maps_kernel(
            __global		float*	vertices,
            __global		float*	normals,
			__global		float*	intensities,
			__global		float*	verticesDst,
            __global		float*	normalsDst,
			__global		float*	intensitiesDst
        )
{
	const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
    const uint gsizex = get_global_size(0);
	const uint dstId = mul24(gidy, gsizex)+gidx;

	const uint src_x = gidx << 1;
	const uint src_y = gidy << 1;
	const uint src_size_x = gsizex << 1;
	
	uint vcount = 0;
	float3 vmap = (float3)(0.0f, 0.0f, 0.0f);
	uint ncount = 0;
	float3 nmap = (float3)(0.0f, 0.0f, 0.0f);
	uint icount = 0;
	float3 imap = (float3)(0.0f, 0.0f, 0.0f);

	uint srcId = mul24(src_y, src_size_x)+src_x;
	float3 vmap_src = vload3(srcId, vertices);
	if( !_isnan3(vmap_src) )
	{
		vmap += vmap_src;
		vcount ++;
		float3 nmap_src = vload3(srcId, normals);
		if( !_isnan3(nmap_src) )
		{
			nmap += nmap_src;
			ncount ++;
		}
		if(intensities)
		{
			float3 imap_src = vload3(srcId, intensities);
			if( !_isnan3(imap_src) )
			{
				imap += imap_src;
				icount ++;
			}
		}
	}

	srcId ++;
	vmap_src = vload3(srcId, vertices);
	if( !_isnan3(vmap_src) )
	{
		vmap += vmap_src;
		vcount ++;
		float3 nmap_src = vload3(srcId, normals);
		if( !_isnan3(nmap_src) )
		{
			nmap += nmap_src;
			ncount ++;
		}
		if(intensities)
		{
			float3 imap_src = vload3(srcId, intensities);
			if( !_isnan3(imap_src) )
			{
				imap += imap_src;
				icount ++;
			}
		}
	}

	srcId += src_size_x;
	vmap_src = vload3(srcId, vertices);
	if( !_isnan3(vmap_src) )
	{
		vmap += vmap_src;
		vcount ++;
		float3 nmap_src = vload3(srcId, normals);
		if( !_isnan3(nmap_src) )
		{
			nmap += nmap_src;
			ncount ++;
		}
		if(intensities)
		{
			float3 imap_src = vload3(srcId, intensities);
			if( !_isnan3(imap_src) )
			{
				imap += imap_src;
				icount ++;
			}
		}
	};
	
	srcId --;
	vmap_src = vload3(srcId, vertices);
	if( !_isnan3(vmap_src) )
	{
		vmap += vmap_src;
		vcount ++;
		float3 nmap_src = vload3(srcId, normals);
		if( !_isnan3(nmap_src) )
		{
			nmap += nmap_src;
			ncount ++;
		}
		if(intensities)
		{
			float3 imap_src = vload3(srcId, intensities);
			if( !_isnan3(imap_src) )
			{
				imap += imap_src;
				icount ++;
			}
		}
	};

	if( 0 == vcount )
	{
		vstore3((float3)(NAN, NAN, NAN), dstId, verticesDst);
	}
	else
	{
		vstore3(vmap / vcount, dstId, verticesDst);
	}

	if( 0 == ncount )
	{
		vstore3((float3)(NAN, NAN, NAN), dstId, normalsDst);
	}
	else
	{
		vstore3(nmap / ncount, dstId, normalsDst);
	}

	if(intensitiesDst)
	{
		if( 0 == icount )
		{
			vstore3((float3)(NAN, NAN, NAN), dstId, intensitiesDst);
		}
		else
		{
			vstore3(imap / icount, dstId, intensitiesDst);
		}
	}
}

__kernel void UV2color(
					__global float* UVs,
					__global uchar* colorImage,
					int color_width,
					int color_height,
					__global uchar* colors
						)
{
	const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
    const uint gsizex = get_global_size(0);
	const uint idx = mul24(gidy, gsizex)+gidx;

	float2 coord = vload2(idx, UVs);
	if( !_isnan2(coord) )
	{
		int colorX = convert_int(coord.x + 0.5f);
		int colorY = convert_int(coord.y + 0.5f);
		if ((colorX >= 0 && colorX < color_width) && (colorY >= 0 && colorY < color_height))
		{
			int color_id = mul24(colorY, color_width) + colorX;
			uchar4 new_color = vload4(color_id, colorImage);
			new_color.w = 255;
			vstore4(new_color, idx, colors);
			return;
		}
	}
	vstore4((uchar4)(0, 0, 0, 0), idx, colors);
}

__kernel void color_to_texture_kernel(
					__global uchar* colors,
					write_only image2d_t tex
						)
{
	const int gidx = get_global_id(0);
	const int gidy = get_global_id(1);
    const int gsizex = get_global_size(0);
	const int idx = mul24(gidy, gsizex)+gidx;

	uchar4 color = vload4(idx, colors);
	float4 floatColor;
	if(color.w > 0)
		floatColor = (float4)(convert_float(color.z) /255.f, convert_float(color.y) /255.f, convert_float(color.x) /255.f, 1.0f);
	else
		floatColor = (float4)(0.0f,0.0f,0.0f,0.0f);
	write_imagef(tex, (int2)(gidx, gidy), floatColor);
}

__kernel void float3_to_texture_kernel(
					__global float* floatColors,
					write_only image2d_t tex
						)
{
	const int gidx = get_global_id(0);
	const int gidy = get_global_id(1);
    const int gsizex = get_global_size(0);
	const int idx = mul24(gidy, gsizex)+gidx;

	float3 color = vload3(idx, floatColors);
	write_imagef(tex, (int2)(gidx, gidy), (float4)(color, 1.0f));
}

__kernel void float2_to_texture_kernel(
					__global float2* floatColors,
					write_only image2d_t tex
						)
{
	const int gidx = get_global_id(0);
	const int gidy = get_global_id(1);
    const int gsizex = get_global_size(0);
	const int idx = mul24(gidy, gsizex)+gidx;

	float2 color = floatColors[idx];
	color.y = color.y / 5.f;
	write_imagef(tex, (int2)(gidx, gidy), (float4)(color.xy, 0.0f, 1.0f));
}

__kernel void intensity_to_float4_kernel(
					__global float* intensities,
					__global float* floatColors
						)
{
	const uint idx = get_global_id(0);

	float intensity = vload(idx, intensities);
	float4 new_color = (float4)(
		intensity,
		intensity,
		intensity,
		1.0
		);
	vstore4(new_color, idx, floatColors);
}

__kernel void bgra_to_intensity_kernel(
					__global uchar* bgras,
					__global float* intensities
						)
{
	const uint idx = get_global_id(0);

	uchar4 color = vload4(idx, bgras);
	float intensity = NAN;
	if(color.w > 0)
		intensity = 0.299f*(convert_float(color.z)/255.0f) + 0.587f*(convert_float(color.y)/255.0f) + 0.114f*(convert_float(color.x)/255.0f);
	vstore(intensity, idx, intensities);
}

__kernel void bgra_to_color_kernel(
					__global float4* d_colors,
					__global uchar4* d_rgbas
						)
{
	const uint idx = get_global_id(0);

	uchar4 rgba = d_rgbas[idx];
	float4 color = (float4)(0.0f, 0.0f, 0.0f, 0.0f);
	if(rgba.w == 255)
	{
		color.x = convert_float(rgba.x) / 255.0f;
		color.y = convert_float(rgba.y) / 255.0f;
		color.z = convert_float(rgba.z) / 255.0f;
		color.w = 1.0f;
	}
	d_colors[idx] = color;
}

__kernel void half_sample_bgra_kernel(
            __global uchar* src,
            __global uchar* dst,
			uint			radius
        )
{
    const int gidx = get_global_id(0);
	const int gidy = get_global_id(1);
    const int gsizex = get_global_size(0);
	const int gsizey = get_global_size(1);

	const int dstId = mul24(gidy, gsizex)+gidx;

	const int src_x = gidx << 1;
	const int src_y = gidy << 1;
	const int src_size_x = gsizex << 1;
	const int src_size_y = gsizey << 1;
	const int srcId = mul24(src_y, src_size_x)+src_x;
	uchar4 center = vload4(srcId, src);
	
	const int tx_min = max(src_x-convert_int(radius), 0);
	const int tx_max = min(src_x+convert_int(radius)+1, src_size_x);
	const int ty_min = max(src_y-convert_int(radius), 0);
	const int ty_max = min(src_y+convert_int(radius)+1, src_size_y);

	int4 sum = (int4)(0,0,0,0);
    int sumWeight = 0;

	for(int cy = ty_min; cy < ty_max; ++ cy)
	{
		for(int cx = tx_min; cx < tx_max; ++ cx)
		{
			const int nearId = mul24(cy, src_size_x)+cx;
			uchar4 near = vload4(nearId, src);
			sum += (int4)(convert_int(near.x), convert_int(near.y), convert_int(near.z), convert_int(near.w));
			sumWeight ++;
		}
	}
	int4 outColor = (sumWeight > 0) ? sum/sumWeight : (int4)(0,0,0,0);
	outColor.w = 255;
	vstore4((uchar4)(convert_uchar(outColor.x), convert_uchar(outColor.y), convert_uchar(outColor.z), convert_uchar(outColor.w)), dstId, dst);
}

__kernel void intensity_derivatives_kernel(
					__global float* intensities,
					__global float* intensityDerivs
						)
{
	const int gidx = get_global_id(0);
	const int gidy = get_global_id(1);
    const int gsizex = get_global_size(0);
	const int gsizey = get_global_size(1);
	int idx = mul24(gidy, gsizex)+gidx;

	float intensity = vload(idx, intensities);
	if( isnan(intensity) )
		return;

	if(gidx > 0 && gidx < gsizex-1 && gidy > 0 && gidy < gsizey-1)
	{
		idx --;
		float intensity01 = vload(idx, intensities);
		if( isnan(intensity01) )
			return;
		idx += 2;
		float intensity21 = vload(idx, intensities);
		if( isnan(intensity21) )
			return;
		idx = idx - gsizey;
		float intensity20 = vload(idx, intensities);
		if( isnan(intensity20) )
			return;
		idx --;
		float intensity10 = vload(idx, intensities);
		if( isnan(intensity10) )
			return;
		idx --;
		float intensity00 = vload(idx, intensities);
		if( isnan(intensity00) )
			return;
		idx = idx + 2*gsizey;
		float intensity02 = vload(idx, intensities);
		if( isnan(intensity02) )
			return;
		idx ++;
		float intensity12 = vload(idx, intensities);
		if( isnan(intensity12) )
			return;
		idx ++;
		float intensity22 = vload(idx, intensities);
		if( isnan(intensity22) )
			return;

		float2 derivative;
		derivative.x = (-1.0f)*intensity00 + (1.0f)*intensity20 +
						  (-2.0f)*intensity01 + (2.0f)*intensity21 +
						  (-1.0f)*intensity02 + (1.0f)*intensity22;
		derivative.x /= 8.0f;
			
		derivative.y = (-1.0f)*intensity00 + (-2.0f)*intensity10 + (-1.0f)*intensity20 + 
						  ( 1.0f)*intensity02 + ( 2.0f)*intensity12 + ( 1.0f)*intensity22;
		derivative.y /= 8.0f;
		
		vstore2(derivative, idx, intensityDerivs);
	}
	
}

__kernel void bgra_gradient_kernel(
					__global uchar* colors,
					__global uchar* gradientXs,
					__global uchar* gradientYs
						)
{
	const int gidx = get_global_id(0);
	const int gidy = get_global_id(1);
    const int gsizex = get_global_size(0);
	const int gsizey = get_global_size(1);
	int idx = mul24(gidy, gsizex)+gidx;
	//uchar4 s22 = vload4(idx, colors);

	if(gidx > 0 && gidx < gsizex-1 && gidy > 0 && gidy < gsizey-1)
	{
		idx --;
		uchar4 s21 = vload4(idx, colors);
		idx += 2;
		uchar4 s23 = vload4(idx, colors);
		idx = idx - gsizey;
		uchar4 s13 = vload4(idx, colors);
		idx --;
		uchar4 s12 = vload4(idx, colors);
		idx --;
		uchar4 s11 = vload4(idx, colors);
		idx = idx + 2*gsizey;
		uchar4 s31 = vload4(idx, colors);
		idx ++;
		uchar4 s32 = vload4(idx, colors);
		idx ++;
		uchar4 s33 = vload4(idx, colors);

		float3 tmp;
		tmp.x = (convert_float(s13.x) - convert_float(s11.x)) + 2*(convert_float(s23.x) - convert_float(s21.x)) + (convert_float(s33.x) - convert_float(s31.x));
		tmp.y = (convert_float(s13.y) - convert_float(s11.y)) + 2*(convert_float(s23.y) - convert_float(s21.y)) + (convert_float(s33.y) - convert_float(s31.y));
		tmp.z = (convert_float(s13.z) - convert_float(s11.z)) + 2*(convert_float(s23.z) - convert_float(s21.z)) + (convert_float(s33.z) - convert_float(s31.z));
		tmp /= 8.0f;
		vstore4((uchar4)(convert_uchar(tmp.x), convert_uchar(tmp.y), convert_uchar(tmp.z), 255), idx, gradientXs);
		
		tmp.x = (convert_float(s31.x) - convert_float(s11.x)) + 2*(convert_float(s32.x) - convert_float(s12.x)) + (convert_float(s33.x) - convert_float(s13.x));
		tmp.y = (convert_float(s31.y) - convert_float(s11.y)) + 2*(convert_float(s32.y) - convert_float(s12.y)) + (convert_float(s33.y) - convert_float(s13.y));
		tmp.z = (convert_float(s31.z) - convert_float(s11.z)) + 2*(convert_float(s32.z) - convert_float(s12.z)) + (convert_float(s33.z) - convert_float(s13.z));
		tmp /= 8.0f;
		vstore4((uchar4)(convert_uchar(tmp.x), convert_uchar(tmp.y), convert_uchar(tmp.z), 255), idx, gradientYs);
	}
	else
	{
		vstore4((uchar4)(0,0,0,0), idx, gradientXs);
		vstore4((uchar4)(0,0,0,0), idx, gradientYs);
	}
}
