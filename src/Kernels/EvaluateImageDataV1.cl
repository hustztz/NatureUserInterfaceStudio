
//static void blendLength(
//            global float* CVs,
//            uint    numCVs,
//            float   newLen
//            )
//{
//    // Calculate total length
//    float curLen = 0.0f;
//    for (uint i = 1; i < numCVs; ++i) {
//        curLen += fast_length((float3)(
//            CVs[i * 3] - CVs[i * 3 - 3],
//            CVs[i * 3 + 1] - CVs[i * 3 - 2],
//            CVs[i * 3 + 2] - CVs[i * 3 - 1]));
//    }
//
//    if (islessequal(curLen, 0.0f))
//        return;
//
//    // If there is basically no change just return.
//    if (isless(fabs(curLen - newLen), 0.0001f))
//        return;
//
//    // Now correct it by scaling all control poly segments
//    const float ratio = newLen / curLen;
//
//    //scalePoly(CVs, cvCount, ratio);
//    float3 oldSeg, newSeg, vec;
//    for (uint j = 0; j < numCVs - 1; j++)
//    {
//        oldSeg = (float3)(
//            CVs[j * 3 + 3] - CVs[j * 3],
//            CVs[j * 3 + 4] - CVs[j * 3 + 1],
//            CVs[j * 3 + 5] - CVs[j * 3 + 2]
//            );
//        newSeg = oldSeg * ratio;
//        vec = newSeg - oldSeg;
//        for (uint k = j + 1; k < numCVs; k++)
//        {
//            CVs[k * 3]     += vec.x;
//            CVs[k * 3 + 1] += vec.y;
//            CVs[k * 3 + 2] += vec.z;
//        }
//    }
//}

__kernel void estimate_normals_kernel(
            __global float* positions,
            __global float* normals,
            float            depthThreshold
        )
{
    uint gidx = get_global_id(0);
	uint gidy = get_global_id(1);
    uint gsizex = get_global_size(0);
	uint gsizey = get_global_size(1);

	const uint centerIndex = mul24(gidy, gsizex)+gidx;
	float3 center = vload3(centerIndex, positions);
	if(center.z > 0.0f)
	{
		float3 left = center;
		if(gidx > 0)
		{
			left = vload3(centerIndex-1, positions);
			if(left.z < 0.0f || fabs(center.z - left.z) > depthThreshold)
				left = center;
		}
		float3 right = center;
		if(gidx < gsizex-1)
		{
			right = vload3(centerIndex+1, positions);
			if(right.z < 0.0f || fabs(center.z - right.z) > depthThreshold)
				right = center;
		}
		float3 up = center;
		if(gidy > 0)
		{
			up = vload3(centerIndex-gsizex, positions);
			if(up.z < 0.0f || fabs(center.z - up.z) > depthThreshold)
				up = center;
		}
		float3 down = center;
		if(gidy < gsizey-1)
		{
			down = vload3(centerIndex+gsizex, positions);
			if(down.z < 0.0f || fabs(center.z - down.z) > depthThreshold)
				down = center;
		}
		float3 leftUp = center;
		if(gidx > 0 && gidy > 0)
		{
			leftUp = vload3(centerIndex-gsizex-1, positions);
			if(leftUp.z < 0.0f || fabs(center.z - leftUp.z) > depthThreshold)
				leftUp = center;
		}
		float3 leftDown = center;
		if(gidx > 0 && gidy < gsizey-1)
		{
			leftDown = vload3(centerIndex+gsizex-1, positions);
			if(leftDown.z < 0.0f || fabs(center.z - leftDown.z) > depthThreshold)
				leftDown = center;
		}
		float3 rightUp = center;
		if(gidx < gsizex-1 && gidy > 0)
		{
			rightUp = vload3(centerIndex-gsizex+1, positions);
			if(rightUp.z < 0.0f || fabs(center.z - rightUp.z) > depthThreshold)
				rightUp = center;
		}
		float3 rightDown = center;
		if(gidx < gsizex-1 && gidy < gsizey-1)
		{
			rightDown = vload3(centerIndex+gsizex+1, positions);
			if(rightDown.z < 0.0f || fabs(center.z - rightDown.z) > depthThreshold)
				rightDown = center;
		}

		float3 norm = fast_normalize( cross( (right-left)+(rightDown-leftDown)+(rightUp-leftUp), (down-up)+(rightDown-rightUp)+(leftDown-leftUp) ) );
		vstore3(norm, centerIndex, normals);
	}
	else
	{
		float3 norm = (float3)(0.0f,0.0f,0.0f);
		vstore3(norm, centerIndex, normals);
	}
}

__kernel void smooth_normals_kernel(
            __global float* positions,
            __global float* normals,
            float            depthThreshold
        )
{
    uint gidx = get_global_id(0);
	uint gidy = get_global_id(1);
    uint gsizex = get_global_size(0);
	uint gsizey = get_global_size(1);
	const float dotThreshold = 0.3f;//72C

	const uint centerIndex = mul24(gidy, gsizex)+gidx;
	float3 center = vload3(centerIndex, positions);
	float3 centerNm = vload3(centerIndex, normals);
	if(center.z > 0.0f)
	{
		float3 leftNm = centerNm;
		if(gidx > 0)
		{
			float3 left = vload3(centerIndex-1, positions);
			if(left.z > 0.0f && fabs(center.z - left.z) < depthThreshold)
			{
				leftNm = vload3(centerIndex-1, normals);
				if(dot(leftNm, centerNm) < dotThreshold)
					leftNm = centerNm;
			}
		}
		float3 rightNm = centerNm;
		if(gidx < gsizex-1)
		{
			float3 right = vload3(centerIndex+1, positions);
			if(right.z > 0.0f && fabs(center.z - right.z) < depthThreshold)
			{
				rightNm = vload3(centerIndex+1, normals);
				if(dot(rightNm, centerNm) < dotThreshold)
					rightNm = centerNm;
			}
		}
		float3 upNm = centerNm;
		if(gidy > 0)
		{
			float3 up = vload3(centerIndex-gsizex, positions);
			if(up.z > 0.0f && fabs(center.z - up.z) < depthThreshold)
			{
				upNm = vload3(centerIndex-gsizex, normals);
				if(dot(upNm, centerNm) < dotThreshold)
					upNm = centerNm;
			}
		}
		float3 downNm = centerNm;
		if(gidy < gsizey-1)
		{
			float3 down = vload3(centerIndex+gsizex, positions);
			if(down.z > 0.0f && fabs(center.z - down.z) < depthThreshold)
			{
				downNm = vload3(centerIndex+gsizex, normals);
				if(dot(downNm, centerNm) < dotThreshold)
					downNm = centerNm;
			}
		}
		float3 leftUpNm = centerNm;
		if(gidx > 0 && gidy > 0)
		{
			float3 leftUp = vload3(centerIndex-gsizex-1, positions);
			if(leftUp.z > 0.0f && fabs(center.z - leftUp.z) < depthThreshold)
			{
				leftUpNm = vload3(centerIndex-gsizex-1, normals);
				if(dot(leftUpNm, centerNm) < dotThreshold)
					leftUpNm = centerNm;
			}
		}
		float3 leftDownNm = centerNm;
		if(gidx > 0 && gidy < gsizey-1)
		{
			float3 leftDown = vload3(centerIndex+gsizex-1, positions);
			if(leftDown.z > 0.0f && fabs(center.z - leftDown.z) < depthThreshold)
			{
				leftDownNm = vload3(centerIndex+gsizex-1, normals);
				if(dot(leftDownNm, centerNm) < dotThreshold)
					leftDownNm = centerNm;
			}
		}
		float3 rightUpNm = centerNm;
		if(gidx < gsizex-1 && gidy > 0)
		{
			float3 rightUp = vload3(centerIndex-gsizex+1, positions);
			if(rightUp.z > 0.0f && fabs(center.z - rightUp.z) < depthThreshold)
			{
				rightUpNm = vload3(centerIndex-gsizex+1, normals);
				if(dot(rightUpNm, centerNm) < dotThreshold)
					rightUpNm = centerNm;
			}
		}
		float3 rightDownNm = centerNm;
		if(gidx < gsizex-1 && gidy < gsizey-1)
		{
			float3 rightDown = vload3(centerIndex+gsizex+1, positions);
			if(rightDown.z > 0.0f && fabs(center.z - rightDown.z) < depthThreshold)
			{
				rightDownNm = vload3(centerIndex+gsizex+1, normals);
				if(dot(rightDownNm, centerNm) < dotThreshold)
					rightDownNm = centerNm;
			}
		}

		float3 norm = centerNm + 0.6f*leftNm + 0.6f*rightNm + 0.6f*upNm + 0.6f*downNm + 0.4f*leftUpNm + 0.4f*leftDownNm + 0.4f*rightUpNm + 0.4f*rightDownNm;
		vstore3(norm, centerIndex, normals);
	}
}
