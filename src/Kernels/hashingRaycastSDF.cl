#include "hashingUtils.cl"

inline static float3 frac(float3 val)
{
	return (val - floor(val));
}

inline static bool trilinearInterpolationSimpleFastFast(
	const HashData& hash,
	float3& pos,
	float& dist,
	uchar3& color,
	const float		virtualVoxelSize)
{
	const float oSet = virtualVoxelSize;
	const float3 posDual = pos - (float3)(oSet/2.0f, oSet/2.0f, oSet/2.0f);
	float3 weight = frac(pos / virtualVoxelSize);

	dist = 0.0f;
	float3 colorFloat = (float3)(0.0f, 0.0f, 0.0f);
	NuiVoxel v = hash.getVoxel(posDual+(float3)(0.0f, 0.0f, 0.0f)); if(v.weight == 0) return false; float3 vColor = (float3)(v.color.x, v.color.y, v.color.z); dist+= (1.0f-weight.x)*(1.0f-weight.y)*(1.0f-weight.z)*v.sdf; colorFloat+= (1.0f-weight.x)*(1.0f-weight.y)*(1.0f-weight.z)*vColor;
		    v = hash.getVoxel(posDual+(float3)(oSet, 0.0f, 0.0f)); if(v.weight == 0) return false;		   vColor = (float3)(v.color.x, v.color.y, v.color.z); dist+=	   weight.x *(1.0f-weight.y)*(1.0f-weight.z)*v.sdf; colorFloat+=	   weight.x *(1.0f-weight.y)*(1.0f-weight.z)*vColor;
		    v = hash.getVoxel(posDual+(float3)(0.0f, oSet, 0.0f)); if(v.weight == 0) return false;		   vColor = (float3)(v.color.x, v.color.y, v.color.z); dist+= (1.0f-weight.x)*	   weight.y *(1.0f-weight.z)*v.sdf; colorFloat+= (1.0f-weight.x)*	   weight.y *(1.0f-weight.z)*vColor;
		    v = hash.getVoxel(posDual+(float3)(0.0f, 0.0f, oSet)); if(v.weight == 0) return false;		   vColor = (float3)(v.color.x, v.color.y, v.color.z); dist+= (1.0f-weight.x)*(1.0f-weight.y)*	   weight.z *v.sdf; colorFloat+= (1.0f-weight.x)*(1.0f-weight.y)*	   weight.z *vColor;
		    v = hash.getVoxel(posDual+(float3)(oSet, oSet, 0.0f)); if(v.weight == 0) return false;		   vColor = (float3)(v.color.x, v.color.y, v.color.z); dist+=	   weight.x *	   weight.y *(1.0f-weight.z)*v.sdf; colorFloat+=	   weight.x *	   weight.y *(1.0f-weight.z)*vColor;
		    v = hash.getVoxel(posDual+(float3)(0.0f, oSet, oSet)); if(v.weight == 0) return false;		   vColor = (float3)(v.color.x, v.color.y, v.color.z); dist+= (1.0f-weight.x)*	   weight.y *	   weight.z *v.sdf; colorFloat+= (1.0f-weight.x)*	   weight.y *	   weight.z *vColor;
		    v = hash.getVoxel(posDual+(float3)(oSet, 0.0f, oSet)); if(v.weight == 0) return false;		   vColor = (float3)(v.color.x, v.color.y, v.color.z); dist+=	   weight.x *(1.0f-weight.y)*	   weight.z *v.sdf; colorFloat+=	   weight.x *(1.0f-weight.y)*	   weight.z *vColor;
		    v = hash.getVoxel(posDual+(float3)(oSet, oSet, oSet)); if(v.weight == 0) return false;		   vColor = (float3)(v.color.x, v.color.y, v.color.z); dist+=	   weight.x *	   weight.y *	   weight.z *v.sdf; colorFloat+=	   weight.x *	   weight.y *	   weight.z *vColor;

	color = (uchar3)(colorFloat.x, colorFloat.y, colorFloat.z);//v.color;
		
	return true;
}

__kernel void renderKernel(
            __constant struct CameraParams* cameraParams,
			__global struct RigidTransform* matrix,
			__global float* vmap,
			__global float* nmap,
			__global float* colormap,
			const float		minDepth,
			const float		maxDepth
        )
{
    const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
	const uint gsizex = get_global_size(0);
	const int idx = mul24(gidy, gsizex) + gidx;

	vstore3(NAN, id, vmap);
	vstore3(NAN, id, nmap);
	vstore4(NAN, id, colormap);

	float3 camDir = normalize(cameraData.kinectProjToCamera(x, y, 1.0f, cameraParams));
	float3 worldCamPos = transform( (float3)(0.0f, 0.0f, 0.0f), matrix );
	float4 w = rotate(camDir, matrix);
	float3 worldDir = normalize(w.xyz);

	float minInterval = tex2D(rayMinTextureRef, x, y);
	float maxInterval = tex2D(rayMaxTextureRef, x, y);

	//float minInterval = rayCastParams.m_minDepth;
	//float maxInterval = rayCastParams.m_maxDepth;

	//if (minInterval == 0 || minInterval == MINF) minInterval = rayCastParams.m_minDepth;
	//if (maxInterval == 0 || maxInterval == MINF) maxInterval = rayCastParams.m_maxDepth;
	//TODO MATTHIAS: shouldn't this return in the case no interval is found?
	if (minInterval == 0 || minInterval == NAN) return;
	if (maxInterval == 0 || maxInterval == NAN) return;

	// debugging 
	//if (maxInterval < minInterval) {
	//	printf("ERROR (%d,%d): [ %f, %f ]\n", x, y, minInterval, maxInterval);
	//}

	float lastSampleSdf = 0.0f;
	float lastSampleAlpha = 0.0f;
	float lastSampleWeight = 0;
	const float depthToRayLength = 1.0f/camDir.z; // scale factor to convert from depth to ray length

	float rayCurrent = depthToRayLength * max(minDepth, minInterval);	// Convert depth to raylength
	float rayEnd = depthToRayLength * min(maxDepth, maxInterval);		// Convert depth to raylength
	//float rayCurrent = depthToRayLength * rayCastParams.m_minDepth;	// Convert depth to raylength
	//float rayEnd = depthToRayLength * rayCastParams.m_maxDepth;		// Convert depth to raylength

	while(rayCurrent < rayEnd)
	{
		float3 currentPosWorld = worldCamPos + rayCurrent * worldDir;
		float dist;	uchar3 color;

		if(trilinearInterpolationSimpleFastFast(hash, currentPosWorld, dist, color))
		{
			if(lastSample.weight > 0 && lastSample.sdf > 0.0f && dist < 0.0f) // current sample is always valid here 
			{

				float alpha; // = findIntersectionLinear(lastSample.alpha, rayCurrent, lastSample.sdf, dist);
				uchar3 color2;
				bool b = findIntersectionBisection(hash, worldCamPos, worldDir, lastSample.sdf, lastSample.alpha, dist, rayCurrent, alpha, color2);
					
				float3 currentIso = worldCamPos+alpha*worldDir;
				if(b && abs(lastSample.sdf - dist) < rayCastParams.m_thresSampleDist)
				{
					if(abs(dist) < rayCastParams.m_thresDist)
					{
						float depth = alpha / depthToRayLength; // Convert ray length to depth depthToRayLength

						d_depth[dTid.y*rayCastParams.m_width+dTid.x] = depth;
						d_depth4[dTid.y*rayCastParams.m_width+dTid.x] = make_float4(cameraData.kinectDepthToSkeleton(dTid.x, dTid.y, depth), 1.0f);
						d_colors[dTid.y*rayCastParams.m_width+dTid.x] = make_float4(color2.x/255.f, color2.y/255.f, color2.z/255.f, 1.0f);

						if(rayCastParams.m_useGradients)
						{
							float3 normal = -gradientForPoint(hash, currentIso);
							float4 n = rayCastParams.m_viewMatrix * make_float4(normal, 0.0f);
							d_normals[dTid.y*rayCastParams.m_width+dTid.x] = make_float4(n.x, n.y, n.z, 1.0f);
						}

						return;
					}
				}
			}

			lastSample.sdf = dist;
			lastSample.alpha = rayCurrent;
			// lastSample.color = color;
			lastSample.weight = 1;
			rayCurrent += rayCastParams.m_rayIncrement;
		} else {
			lastSample.weight = 0;
			rayCurrent += rayCastParams.m_rayIncrement;
		}
	}
}
