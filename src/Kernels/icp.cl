
#include "utils.cl"

__kernel void icp_kernel(
            __global		float*	vertices,
            __global		float*	normals,
			float8					Rcurr1,
			float					Rcurr2,
			float3					tcurr,
			__global		float*	verticesPrev,
            __global		float*	normalsPrev,
			float8					Rprev_inv1,
			float					Rprev_inv2,
			float3					tprev,
			float					intr_fx,
			float					intr_fy,
			float					intr_cx,
			float					intr_cy,
			float					distThres,
			float					angleThres,
			__global volatile float* matrixBuffer
        )
{
	const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
    const uint gsizex = get_global_size(0);
	const uint gsizey = get_global_size(1);

	uint id = mul24(gidy, gsizex)+gidx;
	float3 ncurr = vload3(id, normals);
	if( _isnan3(ncurr) )
		return;
	float3 vcurr = vload3(id, vertices);
    if( _isnan3(vcurr) )
		return;

	float3 projectedVertex = rotate3(vcurr, Rcurr1, Rcurr2) + tcurr;
	float3 projectedPos = rotate3((projectedVertex - tprev), Rprev_inv1, Rprev_inv2);         // prev camera coo space

	int2 projPixel = (int2)(round(projectedPos.x * intr_fx / projectedPos.z + intr_cx), round(projectedPos.y * intr_fy / projectedPos.z + intr_cy));
	if(projPixel.x < 0 || projPixel.x >= gsizex || projPixel.y < 0 || projPixel.y >= gsizey)
		return;

	uint refPixel = mul24(convert_uint(projPixel.y), gsizex)+convert_uint(projPixel.x);
	float3 referenceNormal = vload3(refPixel, normalsPrev);
	if( _isnan3(referenceNormal) )
		return;
	float3 referenceVertex = vload3(refPixel, verticesPrev);
    if( _isnan3(referenceVertex) )
		return;

	float3 diff = referenceVertex - projectedVertex;
	float dist = fast_length (diff);
	if (dist > distThres)
        return;

	float3 projectedNormal = rotate3(ncurr, Rcurr1, Rcurr2);
	float sine = fast_length (cross (projectedNormal, referenceNormal));
	if (sine >= angleThres)
        return;

	float3 row0 = cross (projectedVertex, referenceNormal);
	float coresp[7] = { row0.x, row0.y, row0.z, referenceNormal.x, referenceNormal.y, referenceNormal.z, dot (referenceNormal, diff) };
	
	uint shift = 0;
	for (uint i = 0; i < 6; ++i)        //rows
    {
		for (uint j = i; j < 7; ++j)          // cols + b
        {
			atomicAdd(matrixBuffer + shift, coresp[i] * coresp[j]);
            shift ++;
         }
    }
	atomicAdd(matrixBuffer + shift, coresp[6] * coresp[6]);
	atomicAdd(matrixBuffer + shift + 1, 1.0f);
}


#define CORESPS_NUM 29
#define WORK_GROUP_SIZE 128

static void compute_icp(
			const uint				gid,
			const uint				lid,
			const uint				weight,
			const uint				height,
            __global		float*	vertices,
            __global		float*	normals,
			float8					Rcurr1,
			float					Rcurr2,
			float3					tcurr,
			__global		float*	verticesPrev,
            __global		float*	normalsPrev,
			float8					Rprev_inv1,
			float					Rprev_inv2,
			float3					tprev,
			float					intr_fx,
			float					intr_fy,
			float					intr_cx,
			float					intr_cy,
			float					distThres,
			float					angleThres,
			__local			float*	localBuffer
        )
{
	const uint local_id = mul24(lid, (uint)CORESPS_NUM);
	// Initialize
	for (uint i = 0; i < CORESPS_NUM; ++i)
    {
		vstore(0.0f, local_id + i, localBuffer);
    }

	float3 ncurr = vload3(gid, normals);
	if( _isnan3(ncurr) )
		return;
	float3 vcurr = vload3(gid, vertices);
    if( _isnan3(vcurr) )
		return;

	float3 projectedVertex = rotate3(vcurr, Rcurr1, Rcurr2) + tcurr;
	float3 projectedPos = rotate3((projectedVertex - tprev), Rprev_inv1, Rprev_inv2);         // prev camera coo space

	int2 projPixel = (int2)(round(projectedPos.x * intr_fx / projectedPos.z + intr_cx), round(projectedPos.y * intr_fy / projectedPos.z + intr_cy));
	if(projPixel.x < 0 || projPixel.x >= weight || projPixel.y < 0 || projPixel.y >= height)
		return;

	uint refPixel = mul24(convert_uint(projPixel.y), weight)+convert_uint(projPixel.x);
	float3 referenceNormal = vload3(refPixel, normalsPrev);
	if( _isnan3(referenceNormal) )
		return;
	float3 referenceVertex = vload3(refPixel, verticesPrev);
    if( _isnan3(referenceVertex) )
		return;

	float3 diff = referenceVertex - projectedVertex;
	float dist = fast_length (diff);
	if (dist > distThres)
        return;

	float3 projectedNormal = rotate3(ncurr, Rcurr1, Rcurr2);
	float sine = fast_length (cross (projectedNormal, referenceNormal));
	if (sine >= angleThres)
        return;

	float3 row0 = cross (projectedVertex, referenceNormal);
	float coresp[7] = { row0.x, row0.y, row0.z, referenceNormal.x, referenceNormal.y, referenceNormal.z, dot (referenceNormal, diff) };
	
	uint shift = 0;
	for (uint i = 0; i < 6; ++i)        //rows
    {
		for (uint j = i; j < 7; ++j)          // cols + b
        {
			vstore(coresp[i] * coresp[j],  local_id + shift, localBuffer);
			shift ++;
         }
    }
	vstore(coresp[6] * coresp[6],  local_id + shift, localBuffer);
	shift ++;
	vstore(1.0f,  local_id + shift, localBuffer);
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
			uint a_stride = mul24(lid, (uint)CORESPS_NUM);
			uint b_stride = mul24(lid + d, (uint)CORESPS_NUM);
			for(uint i = 0; i < CORESPS_NUM; i++)
				localBuffer[a_stride+i] += localBuffer[b_stride+i];
		}
	}
	
	barrier(CLK_LOCAL_MEM_FENCE);
	
	if(lid == 0)
	{
		uint group_stride = mul24(get_group_id(0), CORESPS_NUM);
		// the last elements in tempA and tempB contain the sum of the work-group
		for(uint i = 0; i < CORESPS_NUM; i++)
			sumBuffer[group_stride+i] = localBuffer[i];
	}
}

__kernel void icp_block_kernel(
			const			int		div,
			__constant		struct  NuiCLCameraParams* cameraParams,
            __global		float*	vertices,
            __global		float*	normals,
			const			float8	Rcurr1,
			const			float	Rcurr2,
			const			float3	tcurr,
			__global		float*	verticesPrev,
            __global		float*	normalsPrev,
			__global		struct	NuiCLRigidTransform* previousMatrix,
			const			float	distThres,
			const			float	angleThres,
			__global		float*	corespsSumBuffer
        )
{
	__local float localBuffer[ WORK_GROUP_SIZE*CORESPS_NUM ];	// hold the prefix sum for the current work group

	const uint lid = get_local_id(0);
	const uint gid = get_global_id(0);

	const uint local_id = mul24(lid, (uint)CORESPS_NUM);
	// Initialize
	for (uint i = 0; i < CORESPS_NUM; ++i)
    {
		vstore(0.0f, local_id + i, localBuffer);
    }

	float3 ncurr = vload3(gid, normals);
	if( !_isnan3(ncurr) )
	{
		float3 vcurr = vload3(gid, vertices);
		if( !_isnan3(vcurr) )
		{
			float3 projectedVertex = rotate3(vcurr, Rcurr1, Rcurr2) + tcurr;
			float3 projectedPos = transformInverse(projectedVertex, previousMatrix);         // prev camera coo space

			struct NuiCLCameraParams camParams = *cameraParams;
			const float intr_fx = camParams.fx / div;
			const float intr_fy = camParams.fy / div;
			const float intr_cx = camParams.cx / div;
			const float intr_cy = camParams.cy / div;
			const int nImageWidth = camParams.depthImageWidth / div;
			const int nImageHeight = camParams.depthImageHeight / div;
			int2 projPixel = (int2)(round(projectedPos.x * intr_fx / projectedPos.z + intr_cx), round(projectedPos.y * intr_fy / projectedPos.z + intr_cy));
			if(projPixel.x >= 0 && projPixel.x < nImageWidth && projPixel.y >= 0 && projPixel.y < nImageHeight)
			{
				int refPixel = mul24(projPixel.y, nImageWidth)+projPixel.x;
				float3 referenceNormal = vload3(refPixel, normalsPrev);
				if( !_isnan3(referenceNormal) )
				{
					float3 referenceVertex = vload3(refPixel, verticesPrev);
					if( !_isnan3(referenceVertex) )
					{
						float3 diff = referenceVertex - projectedVertex;
						float dist = fast_length (diff);
						if (dist < distThres)
						{
							float3 projectedNormal = rotate3(ncurr, Rcurr1, Rcurr2);
							float sine = fast_length (cross (projectedNormal, referenceNormal));
							if (sine < angleThres)
							{
								float3 referencePos = transformInverse(referenceVertex, previousMatrix);
								diff = referencePos - projectedPos;
								float3 referenceNorm = rotationInverse(referenceNormal, previousMatrix);
								float3 row0 = cross (projectedPos, referenceNorm);
								float coresp[7] = { row0.x, row0.y, row0.z, referenceNorm.x, referenceNorm.y, referenceNorm.z, dot (referenceNorm, diff) };
	
								uint shift = 0;
								for (uint i = 0; i < 6; ++i)        //rows
								{
									for (uint j = i; j < 7; ++j)          // cols + b
									{
										vstore(coresp[i] * coresp[j],  local_id + shift, localBuffer);
										shift ++;
									 }
								}
								vstore(coresp[6] * coresp[6],  local_id + shift, localBuffer);
								shift ++;
								vstore(1.0f,  local_id + shift, localBuffer);
							}
						}
					}
				}
			}
		}
	}

	reduce(lid, gid, localBuffer, corespsSumBuffer, WORK_GROUP_SIZE);
}

__kernel void compute_sums
(
	__global float* corespsBlocks,
	__global float* corespsBuffer,
				uint size
)
{
	__local float localBuffer[ WORK_GROUP_SIZE * CORESPS_NUM ];
	
	const uint lid = get_local_id(0);
	const uint thid = get_global_id(0);
	const uint lid_stride = mul24(lid, (uint)CORESPS_NUM);
	const uint thid_stride = mul24(thid, (uint)CORESPS_NUM);

	if(thid < size)
	{
		for(int i = 0; i < CORESPS_NUM; i++)
		{
			localBuffer[lid_stride+i] = corespsBlocks[thid_stride+i];
		}
	}
	else 
	{
		for(int i = 0; i < CORESPS_NUM; i++)
		{
			localBuffer[lid_stride+i] = 0.0f;
		}
	}

	barrier(CLK_LOCAL_MEM_FENCE);
	
	reduce(lid, thid, localBuffer, corespsBuffer, WORK_GROUP_SIZE);
}

__kernel void intensity_icp_block_kernel(
			const			int		div,
			__constant		struct  NuiCLCameraParams* cameraParams,
            __global		float*	vertices,
            __global		float*	normals,
			__global		float*	intensities,
			const			float8	Rcurr1,
			const			float	Rcurr2,
			const			float3	tcurr,
			__global		float*	verticesPrev,
            __global		float*	normalsPrev,
			__global		float*	intensitiesPrev,
			__global		struct	NuiCLRigidTransform* previousMatrix,
			const			float	distThres,
			const			float	angleThres,
			__global		float*	corespsSumBuffer
        )
{
	__local float localBuffer[ WORK_GROUP_SIZE*CORESPS_NUM ];	// hold the prefix sum for the current work group

	const uint lid = get_local_id(0);
	const uint gid = get_global_id(0);

	const uint local_id = mul24(lid, (uint)CORESPS_NUM);
	// Initialize
	for (uint i = 0; i < CORESPS_NUM; ++i)
    {
		vstore(0.0f, local_id + i, localBuffer);
    }

	float3 ncurr = vload3(gid, normals);
	if( !_isnan3(ncurr) )
	{
		float3 vcurr = vload3(gid, vertices);
		if( !_isnan3(vcurr) )
		{
			float3 projectedVertex = rotate3(vcurr, Rcurr1, Rcurr2) + tcurr;
			float3 projectedPos = transformInverse(projectedVertex, previousMatrix);         // prev camera coo space

			struct NuiCLCameraParams camParams = *cameraParams;
			const float intr_fx = camParams.fx / div;
			const float intr_fy = camParams.fy / div;
			const float intr_cx = camParams.cx / div;
			const float intr_cy = camParams.cy / div;
			const int nImageWidth = camParams.depthImageWidth / div;
			const int nImageHeight = camParams.depthImageHeight / div;
			int2 projPixel = (int2)(round(projectedPos.x * intr_fx / projectedPos.z + intr_cx), round(projectedPos.y * intr_fy / projectedPos.z + intr_cy));
			if(projPixel.x >= 0 && projPixel.x < nImageWidth && projPixel.y >= 0 && projPixel.y < nImageHeight)
			{
				int refPixel = mul24(projPixel.y, nImageWidth)+projPixel.x;
				float3 referenceNormal = vload3(refPixel, normalsPrev);
				if( !_isnan3(referenceNormal) )
				{
					float3 referenceVertex = vload3(refPixel, verticesPrev);
					if( !_isnan3(referenceVertex) )
					{
						float3 diff = referenceVertex - projectedVertex;
						float dist = fast_length (diff);
						if (dist < distThres)
						{
							float3 projectedNormal = rotate3(ncurr, Rcurr1, Rcurr2);
							float sine = fast_length (cross (projectedNormal, referenceNormal));
							if (sine < angleThres)
							{
								float3 row0 = cross (projectedVertex, referenceNormal);
								float coresp[7] = { row0.x, row0.y, row0.z, referenceNormal.x, referenceNormal.y, referenceNormal.z, dot (referenceNormal, diff) };
	
								uint shift = 0;
								for (uint i = 0; i < 6; ++i)        //rows
								{
									for (uint j = i; j < 7; ++j)          // cols + b
									{
										vstore(coresp[i] * coresp[j],  local_id + shift, localBuffer);
										shift ++;
									 }
								}
								if(dist > distThres/100.0f && sine > angleThres/100.0f)
								{
									vstore(coresp[6] * coresp[6],  local_id + shift, localBuffer);
									shift ++;
									vstore(1.0f,  local_id + shift, localBuffer);
								}
							}
						}
					}
				}
			}
		}
	}
	
	reduce(lid, gid, localBuffer, corespsSumBuffer, WORK_GROUP_SIZE);
}