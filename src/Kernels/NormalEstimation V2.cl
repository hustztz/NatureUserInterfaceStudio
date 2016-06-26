#define FLOAT_EPSILON 1.192092896e-07f

static float3 computeRoots2(float b, float c)
{
	float3 roots = (float3)(0.f, 0.f, 0.f);
	float d = b * b - 4.f * c;
	if (d < 0.f) // no real roots!!!! THIS SHOULD NOT HAPPEN!
		d = 0.f;

	float sd = sqrt(d);

	roots.z = 0.5f * (b + sd);
	roots.y = 0.5f * (b - sd);
	return roots;
}


static float3 computeRoots3(float c0, float c1, float c2)
{
	float3 roots = (float3)(0.f, 0.f, 0.f);
    
    if ( fabs(c0) > FLOAT_EPSILON )
    {
        const float s_inv3 = 0.3333333f;
        const float s_sqrt3 = sqrt(3.f);
        // Construct the parameters used in classifying the roots of the equation
        // and in solving the equation for the roots in closed form.
        float c2_over_3 = c2 * s_inv3;
        float a_over_3 = (c1 - c2*c2_over_3)*s_inv3;
        if (a_over_3 > 0.f)
           a_over_3 = 0.f;

        float half_b = 0.5f * (c0 + c2_over_3 * (2.f * c2_over_3 * c2_over_3 - c1));

        float q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;
        if (q > 0.f)
           q = 0.f;

        // Compute the eigenvalues by solving for the roots of the polynomial.
        float rho = sqrt(-a_over_3);
        float theta = atan2 (sqrt (-q), half_b)*s_inv3;
        float cos_theta;
        float sin_theta = sincos (theta, &cos_theta);

		roots.x = c2_over_3 + 2.f * rho * cos_theta;
        roots.y = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
        roots.z = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

		uint2 mask = (uint2)(1, 0);
        // Sort in increasing order.
        if (roots.x > roots.y)
		{
			//swap
			roots.xy = shuffle(roots.xy, mask);
		}

        if (roots.y > roots.z)
        {
			//swap
			roots.yz = shuffle(roots.yz, mask);

			if (roots.x > roots.y)
			{
				//swap
				roots.xy = shuffle(roots.xy, mask);
			}
        }
        if (roots.x < 0) // eigenval for symetric positive semi-definite matrix can not be negative! Set it to 0
			roots = computeRoots2 (c2, c1);
    }
	else// one root is 0 -> quadratic equation
	{
        roots = computeRoots2 (c2, c1);
    }
	return roots;
}

static float3 unitOrthogonal (float3 src)
{
    float3 perp;
    /* Let us compute the crossed product of *this with a vector
    * that is not too close to being colinear to *this.
    */

    /* unless the x and y coords are both close to zero, we can
    * simply take ( -y, x, 0 ) and normalize it.
    */
	const float prec_sqr = FLOAT_EPSILON;
    if((fabs(src.x) > prec_sqr * fabs(src.z)) || (fabs(src.y) > prec_sqr * fabs(src.z)))
    {
        float invnm = rsqrt(src.x*src.x + src.y*src.y);
        perp.x = -src.y * invnm;
        perp.y =  src.x * invnm;
        perp.z = 0.0f;
    }
    /* if both x and y are close to zero, then the vector is close
    * to the z-axis, so it's far from colinear to the x-axis for instance.
    * So we take the crossed product with (1,0,0) and normalize it. 
    */
    else
    {
        float invnm = rsqrt(src.z * src.z + src.y * src.y);
        perp.x = 0.0f;
        perp.y = -src.z * invnm;
        perp.z =  src.y * invnm;
    }
    return perp;
}

__kernel void estimate_normals_covariance_kernel(
            __global float* positions,
            __global float* normals,
            float            depthThreshold
        )
{
    const uint gidx = get_global_id(0);
	const uint gidy = get_global_id(1);
    const uint gsizex = get_global_size(0);
	const uint gsizey = get_global_size(1);

	const uint centerIndex = mul24(gidy, gsizex)+gidx;
	float3 center = vload3(centerIndex, positions);
	if(center.z > 0.0f)
	{
		float3 centroid = center;
		uint validCount = 0;
		// Left
		float3 Left = (float3)(0.0f,0.0f,-1.f);
		float3 LeftLeft = (float3)(0.0f,0.0f,-1.f);
		if(gidx > 0)
		{
			Left = vload3(centerIndex-1, positions);
			if(Left.z > 0.0f && fabs(center.z - Left.z) < depthThreshold)
			{
				centroid += Left;
				validCount ++;
				// LeftLeft
				if(gidx > 1)
				{
					LeftLeft = vload3(centerIndex-2, positions);
					if(LeftLeft.z > 0.0f && fabs(center.z - LeftLeft.z) < depthThreshold)
					{
						centroid += LeftLeft;
						validCount ++;
					}
				}
			}
		}
		// Right
		float3 Right = (float3)(0.0f,0.0f,-1.f);
		float3 RightRight = (float3)(0.0f,0.0f,-1.f);
		if(gidx < gsizex-1)
		{
			Right = vload3(centerIndex+1, positions);
			if(Right.z > 0.0f && fabs(center.z - Right.z) < depthThreshold)
			{
				centroid += Right;
				validCount ++;
				// RightRight
				if(gidx < gsizex-2)
				{
					RightRight = vload3(centerIndex+2, positions);
					if(RightRight.z > 0.0f && fabs(center.z - RightRight.z) < depthThreshold)
					{
						centroid += RightRight;
						validCount ++;
					}
				}
			}
		}
		// Up
		float3 Up = (float3)(0.0f,0.0f,-1.f);
		float3 UpUp = (float3)(0.0f,0.0f,-1.f);
		if(gidy > 0)
		{
			Up = vload3(centerIndex-gsizex, positions);
			if(Up.z > 0.0f && fabs(center.z - Up.z) < depthThreshold)
			{
				centroid += Up;
				validCount ++;
				// UpUp
				if(gidy > 1)
				{
					UpUp = vload3(centerIndex-2*gsizex, positions);
					if(UpUp.z > 0.0f && fabs(center.z - UpUp.z) < depthThreshold)
					{
						centroid += UpUp;
						validCount ++;
					}
				}
			}
		}
		// Down
		float3 Down = (float3)(0.0f,0.0f,-1.f);
		float3 DownDown = (float3)(0.0f,0.0f,-1.f);
		if(gidy < gsizey-1)
		{
			Down = vload3(centerIndex+gsizex, positions);
			if(Down.z > 0.0f && fabs(center.z - Down.z) < depthThreshold)
			{
				centroid += Down;
				validCount ++;
				// DownDown
				if(gidy < gsizey-2)
				{
					DownDown = vload3(centerIndex+2*gsizex, positions);
					if(DownDown.z > 0.0f && fabs(center.z - DownDown.z) < depthThreshold)
					{
						centroid += DownDown;
						validCount ++;
					}
				}
			}
		}
		// LeftUp
		float3 LeftUp = (float3)(0.0f,0.0f,-1.f);
		float3 LeftUpLeft = (float3)(0.0f,0.0f,-1.f);
		float3 LeftUpUp = (float3)(0.0f,0.0f,-1.f);
		float3 LeftUpLeftUp = (float3)(0.0f,0.0f,-1.f);
		if(gidx > 0 && gidy > 0)
		{
			LeftUp = vload3(centerIndex-gsizex-1, positions);
			if(LeftUp.z > 0.0f && fabs(center.z - LeftUp.z) < depthThreshold)
			{
				centroid += LeftUp;
				validCount ++;
				// LeftUpLeft
				if(gidx > 1 && gidy > 0)
				{
					LeftUp = vload3(centerIndex-gsizex-2, positions);
					if(LeftUp.z > 0.0f && fabs(center.z - LeftUp.z) < depthThreshold)
					{
						centroid += LeftUp;
						validCount ++;
					}
				}
				// LeftUpUp
				if(gidx > 0 && gidy > 1)
				{
					LeftUpUp = vload3(centerIndex-2*gsizex-1, positions);
					if(LeftUpUp.z > 0.0f && fabs(center.z - LeftUpUp.z) < depthThreshold)
					{
						centroid += LeftUpUp;
						validCount ++;
					}
				}
				// LeftUpLeftUp
				if(gidx > 1 && gidy > 1)
				{
					LeftUpLeftUp = vload3(centerIndex-2*gsizex-2, positions);
					if(LeftUpLeftUp.z > 0.0f && fabs(center.z - LeftUpLeftUp.z) < depthThreshold)
					{
						centroid += LeftUpLeftUp;
						validCount ++;
					}
				}
			}
		}
		// LeftDown
		float3 LeftDown = (float3)(0.0f,0.0f,-1.f);
		float3 LeftDownLeft = (float3)(0.0f,0.0f,-1.f);
		float3 LeftDownDown = (float3)(0.0f,0.0f,-1.f);
		float3 LeftDownLeftDown = (float3)(0.0f,0.0f,-1.f);
		if(gidx > 0 && gidy < gsizey-1)
		{
			LeftDown = vload3(centerIndex+gsizex-1, positions);
			if(LeftDown.z > 0.0f && fabs(center.z - LeftDown.z) < depthThreshold)
			{
				centroid += LeftDown;
				validCount ++;
				// LeftDownLeft
				if(gidx > 1 && gidy < gsizey-1)
				{
					LeftDownLeft = vload3(centerIndex+gsizex-2, positions);
					if(LeftDownLeft.z > 0.0f && fabs(center.z - LeftDownLeft.z) < depthThreshold)
					{
						centroid += LeftDownLeft;
						validCount ++;
					}
				}
				// LeftDownDown
				if(gidx > 0 && gidy < gsizey-2)
				{
					LeftDownDown = vload3(centerIndex+2*gsizex-1, positions);
					if(LeftDownDown.z > 0.0f && fabs(center.z - LeftDownDown.z) < depthThreshold)
					{
						centroid += LeftDownDown;
						validCount ++;
					}
				}
				// LeftDownLeftDown
				if(gidx > 1 && gidy < gsizey-2)
				{
					LeftDownLeftDown = vload3(centerIndex+2*gsizex-2, positions);
					if(LeftDownLeftDown.z > 0.0f && fabs(center.z - LeftDownLeftDown.z) < depthThreshold)
					{
						centroid += LeftDownLeftDown;
						validCount ++;
					}
				}
			}
		}
		// RightUp
		float3 RightUp = (float3)(0.0f,0.0f,-1.f);
		float3 RightUpRight = (float3)(0.0f,0.0f,-1.f);
		float3 RightUpUp = (float3)(0.0f,0.0f,-1.f);
		float3 RightUpRightUp = (float3)(0.0f,0.0f,-1.f);
		if(gidx < gsizex-1 && gidy > 0)
		{
			RightUp = vload3(centerIndex-gsizex+1, positions);
			if(RightUp.z > 0.0f && fabs(center.z - RightUp.z) < depthThreshold)
			{
				centroid += RightUp;
				validCount ++;
				// RightUpRight
				if(gidx < gsizex-2 && gidy > 0)
				{
					RightUpRight = vload3(centerIndex-gsizex+2, positions);
					if(RightUpRight.z > 0.0f && fabs(center.z - RightUpRight.z) < depthThreshold)
					{
						centroid += RightUpRight;
						validCount ++;
					}
				}
				// RightUpUp
				if(gidx < gsizex-1 && gidy > 1)
				{
					RightUpUp = vload3(centerIndex-2*gsizex+1, positions);
					if(RightUpUp.z > 0.0f && fabs(center.z - RightUpUp.z) < depthThreshold)
					{
						centroid += RightUpUp;
						validCount ++;
					}
				}
				// RightUpRightUp
				if(gidx < gsizex-2 && gidy > 1)
				{
					RightUpRightUp = vload3(centerIndex-2*gsizex+2, positions);
					if(RightUpRightUp.z > 0.0f && fabs(center.z - RightUpRightUp.z) < depthThreshold)
					{
						centroid += RightUpRightUp;
						validCount ++;
					}
				}
			}
		}
		// RightDown
		float3 RightDown = (float3)(0.0f,0.0f,-1.f);
		float3 RightDownRight = (float3)(0.0f,0.0f,-1.f);
		float3 RightDownDown = (float3)(0.0f,0.0f,-1.f);
		float3 RightDownRightDown = (float3)(0.0f,0.0f,-1.f);
		if(gidx < gsizex-1 && gidy < gsizey-1)
		{
			RightDown = vload3(centerIndex+gsizex+1, positions);
			if(RightDown.z > 0.0f && fabs(center.z - RightDown.z) < depthThreshold)
			{
				centroid += RightDown;
				validCount ++;
				// RightDownRight
				if(gidx < gsizex-2 && gidy < gsizey-1)
				{
					RightDownRight = vload3(centerIndex+gsizex+2, positions);
					if(RightDownRight.z > 0.0f && fabs(center.z - RightDownRight.z) < depthThreshold)
					{
						centroid += RightDownRight;
						validCount ++;
					}
				}
				// RightDownDown
				if(gidx < gsizex-1 && gidy < gsizey-2)
				{
					RightDownDown = vload3(centerIndex+2*gsizex+1, positions);
					if(RightDownDown.z > 0.0f && fabs(center.z - RightDownDown.z) < depthThreshold)
					{
						centroid += RightDownDown;
						validCount ++;
					}
				}
				// RightDownRightDown
				if(gidx < gsizex-2 && gidy < gsizey-2)
				{
					RightDownRightDown = vload3(centerIndex+2*gsizex+2, positions);
					if(RightDownRightDown.z > 0.0f && fabs(center.z - RightDownRightDown.z) < depthThreshold)
					{
						centroid += RightDownRightDown;
						validCount ++;
					}
				}
			}
		}

		centroid *= 1.f / (validCount+1);
		float cov[6];
		{
			float3 d = center - centroid;
			cov[0] = d.x * d.x;               //cov (0, 0)
			cov[1] = d.x * d.y;               //cov (0, 1)
			cov[2] = d.x * d.z;               //cov (0, 2)
			cov[3] = d.y * d.y;               //cov (1, 1)
			cov[4] = d.y * d.z;               //cov (1, 2)
			cov[5] = d.z * d.z;               //cov (2, 2)
			// Left
			if(Left.z > 0.0f)
			{
				d = Left - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// LeftLeft
			if(LeftLeft.z > 0.0f)
			{
				d = LeftLeft - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// Right
			if(Right.z > 0.0f)
			{
				d = Right - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// RightRight
			if(RightRight.z > 0.0f)
			{
				d = RightRight - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// Up
			if(Up.z > 0.0f)
			{
				d = Up - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// UpUp
			if(UpUp.z > 0.0f)
			{
				d = UpUp - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// Down
			if(Down.z > 0.0f)
			{
				d = Down - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// DownDown
			if(DownDown.z > 0.0f)
			{
				d = DownDown - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// LeftUp
			if(LeftUp.z > 0.0f)
			{
				d = LeftUp - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// LeftUpLeft
			if(LeftUpLeft.z > 0.0f)
			{
				d = LeftUpLeft - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// LeftUpUp
			if(LeftUpUp.z > 0.0f)
			{
				d = LeftUpUp - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// LeftUpLeftUp
			if(LeftUpLeftUp.z > 0.0f)
			{
				d = LeftUpLeftUp - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// LeftDown
			if(LeftDown.z > 0.0f)
			{
				d = LeftDown - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// LeftDownLeft
			if(LeftDownLeft.z > 0.0f)
			{
				d = LeftDownLeft - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// LeftDownDown
			if(LeftDownDown.z > 0.0f)
			{
				d = LeftDownDown - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// LeftDownLeftDown
			if(LeftDownLeftDown.z > 0.0f)
			{
				d = LeftDownLeftDown - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// RightUp
			if(RightUp.z > 0.0f)
			{
				d = RightUp - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// RightUpRight
			if(RightUpRight.z > 0.0f)
			{
				d = RightUpRight - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// RightUpUp
			if(RightUpRight.z > 0.0f)
			{
				d = RightUpRight - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// RightUpRightUp
			if(RightUpRightUp.z > 0.0f)
			{
				d = RightUpRightUp - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// RightDown
			if(RightDown.z > 0.0f)
			{
				d = RightDown - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// RightDownRight
			if(RightDownRight.z > 0.0f)
			{
				d = RightDownRight - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// RightDownDown
			if(RightDownDown.z > 0.0f)
			{
				d = RightDownDown - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
			// RightDownRightDown
			if(RightDownRightDown.z > 0.0f)
			{
				d = RightDownRightDown - centroid;
				cov[0] += d.x * d.x;               //cov (0, 0)
				cov[1] += d.x * d.y;               //cov (0, 1)
				cov[2] += d.x * d.z;               //cov (0, 2)
				cov[3] += d.y * d.y;               //cov (1, 1)
				cov[4] += d.y * d.z;               //cov (1, 2)
				cov[5] += d.z * d.z;               //cov (2, 2)
			}
		}

		/*float scale = fabs( maxmag(maxmag(maxmag(maxmag(maxmag(cov[0],cov[1]),cov[2]),cov[3]),cov[4]),cov[5]) );
		if (scale <= FLOAT_EPSILON)
           scale = 1.f;
		for(uint i = 0; i < 6; ++i)
			cov[i] /= scale;*/

		// The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0.  The
        // eigenvalues are the roots to this equation, all guaranteed to be
        // real-valued, because the matrix is symmetric.
		float c0 = cov[0]*cov[3]*cov[5]+2.f*cov[1]*cov[2]*cov[4]-cov[0]*cov[4]*cov[4]-cov[3]*cov[2]*cov[2]-cov[5]*cov[1]*cov[1];
		float c1 = cov[0]*cov[3]+cov[0]*cov[5]+cov[5]*cov[3]-cov[1]*cov[1]-cov[2]*cov[2]-cov[4]*cov[4];
		float c2 = cov[0]+cov[3]+cov[5];
		float3 evals = computeRoots3(c0, c1, c2);

		if(evals.z - evals.x <= FLOAT_EPSILON)
        {                                   
			float3 evecs0 = (float3)(0.f, 0.f, -1.f);
			vstore3(evecs0, centerIndex, normals);
        }
		else if (evals.y - evals.x <= FLOAT_EPSILON )
		{
			// first and second equal
			float3 row0 = (float3)(cov[0], cov[1], cov[2]);
			float3 row1 = (float3)(cov[1], cov[3], cov[4]);
			float3 row2 = (float3)(cov[2], cov[4], cov[5]);
			row0.x -= evals.z;
			row1.y -= evals.z;
			row2.z -= evals.z;

			float3 vec_tmp0 = cross(row0, row1);
			float3 vec_tmp1 = cross(row0, row2);
			float3 vec_tmp2 = cross(row1, row2);

			float len1 = dot (vec_tmp0, vec_tmp0);
			float len2 = dot (vec_tmp1, vec_tmp1);
			float len3 = dot (vec_tmp2, vec_tmp2);

			float3 evecs2;
			if (len1 > len2 && len1 > len3)
			{
				evecs2 = vec_tmp0 * rsqrt (len1);
			}
			else if (len2 > len1 && len2 > len3)
			{
				evecs2 = vec_tmp1 * rsqrt (len2);
			}
			else
			{
				evecs2 = vec_tmp2 * rsqrt (len3);
			}
			float3 evecs1 = unitOrthogonal(evecs2);
			float3 evecs0 = fast_normalize( cross(evecs1, evecs2) );
			vstore3(evecs0.z > 0.f ? -evecs0 : evecs0, centerIndex, normals);
		}
		else if (evals.z - evals.y <= FLOAT_EPSILON )
		{
			// second and third equal
			float3 row0 = (float3)(cov[0], cov[1], cov[2]);
			float3 row1 = (float3)(cov[1], cov[3], cov[4]);
			float3 row2 = (float3)(cov[2], cov[4], cov[5]);
			row0.x -= evals.x;
			row1.y -= evals.x;
			row2.z -= evals.x;

			float3 vec_tmp0 = cross(row0, row1);
			float3 vec_tmp1 = cross(row0, row2);
			float3 vec_tmp2 = cross(row1, row2);

			float len1 = dot (vec_tmp0, vec_tmp0);
			float len2 = dot (vec_tmp1, vec_tmp1);
			float len3 = dot (vec_tmp2, vec_tmp2);

			float3 evecs0;
			if (len1 > len2 && len1 > len3)
			{
				evecs0 = vec_tmp0 * rsqrt (len1);
			}
			else if (len2 > len1 && len2 > len3)
			{
				evecs0 = vec_tmp1 * rsqrt (len2);
			}
			else
			{
				evecs0 = vec_tmp2 * rsqrt (len3);
			}
			evecs0 = fast_normalize(evecs0);
			vstore3(evecs0.z > 0.f ? -evecs0 : evecs0, centerIndex, normals);
		}
		else
		{
			float3 row0 = (float3)(cov[0], cov[1], cov[2]);
			float3 row1 = (float3)(cov[1], cov[3], cov[4]);
			float3 row2 = (float3)(cov[2], cov[4], cov[5]);
			row0.x -= evals.z;
			row1.y -= evals.z;
			row2.z -= evals.z;

			float3 vec_tmp0 = cross(row0, row1);
			float3 vec_tmp1 = cross(row0, row2);
			float3 vec_tmp2 = cross(row1, row2);

			float len1 = dot (vec_tmp0, vec_tmp0);
			float len2 = dot (vec_tmp1, vec_tmp1);
			float len3 = dot (vec_tmp2, vec_tmp2);

			float mmax[3];
			uint min_el = 2;
			uint max_el = 2;

			float3 evecs2;
			if (len1 > len2 && len1 > len3)
			{
				mmax[2] = len1;
				evecs2 = vec_tmp0 * rsqrt (len1);
			}
			else if (len2 > len1 && len2 > len3)
			{
				mmax[2] = len2;
				evecs2 = vec_tmp1 * rsqrt (len2);
			}
			else
			{
				mmax[2] = len3;
				evecs2 = vec_tmp2 * rsqrt (len3);
			}

			row0.x = cov[0] - evals.y;
			row1.y = cov[3] - evals.y;
			row2.z = cov[5] - evals.y;

			vec_tmp0 = cross(row0, row1);
			vec_tmp1 = cross(row0, row2);
			vec_tmp2 = cross(row1, row2);

			len1 = dot (vec_tmp0, vec_tmp0);
			len2 = dot (vec_tmp1, vec_tmp1);
			len3 = dot (vec_tmp2, vec_tmp2);

			float3 evecs1;
			if (len1 > len2 && len1 > len3)
			{
				mmax[1] = len1;
				evecs1 = vec_tmp0 * rsqrt (len1);
				min_el = len1 <= mmax[min_el] ? 1 : min_el;
				max_el = len1  > mmax[max_el] ? 1 : max_el;
			}
			else if (len2 > len1 && len2 > len3)
			{
				mmax[1] = len2;
				evecs1 = vec_tmp1 * rsqrt (len2);
				min_el = len2 <= mmax[min_el] ? 1 : min_el;
				max_el = len2  > mmax[max_el] ? 1 : max_el;
			}
			else
			{
				mmax[1] = len3;
				evecs1 = vec_tmp2 * rsqrt (len3);
				min_el = len3 <= mmax[min_el] ? 1 : min_el;
				max_el = len3 >  mmax[max_el] ? 1 : max_el;
			}

			row0.x = cov[0] - evals.x;
			row1.y = cov[3] - evals.x;
			row2.z = cov[5] - evals.x;

			vec_tmp0 = cross(row0, row1);
			vec_tmp1 = cross(row0, row2);
			vec_tmp2 = cross(row1, row2);

			len1 = dot (vec_tmp0, vec_tmp0);
			len2 = dot (vec_tmp1, vec_tmp1);
			len3 = dot (vec_tmp2, vec_tmp2);

			float3 evecs0;
			if (len1 > len2 && len1 > len3)
			{
				mmax[0] = len1;
				evecs0 = vec_tmp0 * rsqrt (len1);
				min_el = len1 <= mmax[min_el] ? 0 : min_el;
				max_el = len1  > mmax[max_el] ? 0 : max_el;
			}
			else if (len2 > len1 && len2 > len3)
			{
				mmax[0] = len2;
				evecs0 = vec_tmp1 * rsqrt (len2);
				min_el = len2 <= mmax[min_el] ? 0 : min_el;
				max_el = len2  > mmax[max_el] ? 0 : max_el;
			}
			else
			{
				mmax[0] = len3;
				evecs0 = vec_tmp2 * rsqrt (len3);
				min_el = len3 <= mmax[min_el] ? 0 : min_el;
				max_el = len3 >  mmax[max_el] ? 0 : max_el;
			}

			if(min_el == 0)
			{
				evecs0 = fast_normalize( cross(evecs1, evecs2) );
			}
			else if(max_el == 0)
			{
				evecs0 = fast_normalize( evecs0 );
			}
			else if(min_el == 1)
			{
				evecs1 = cross(evecs0, evecs2);
				evecs0 = fast_normalize( cross(evecs1, evecs2) );
			}
			else
			{
				evecs2 = cross(evecs0, evecs1);
				evecs0 = fast_normalize( cross(evecs1, evecs2) );
			}
			vstore3(evecs0.z > 0.f ? -evecs0 : evecs0, centerIndex, normals);
		}
	}
	else
	{
		float3 norm = (float3)(0.0f,0.0f,0.0f);
		vstore3(norm, centerIndex, normals);
	}
}

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

#define SECOND_CONFIDENCE_RATIO 0.8f

// -2 -2 -2 -2 -2
// -2 -1 -1 -1 -2
// -2 -1  0 -1 -2
// -2 -1 -1 -1 -2
// -2 -2 -2 -2 -2
__kernel void smooth_normals_kernel(
            __global float* positions,
            __global float* normals,
            float           depthThreshold,
			float			dotThreshold
        )
{
    uint gidx = get_global_id(0);
	uint gidy = get_global_id(1);
    uint gsizex = get_global_size(0);
	uint gsizey = get_global_size(1);

	const uint centerIndex = mul24(gidy, gsizex)+gidx;
	float3 center = vload3(centerIndex, positions);
	float3 centerNm = vload3(centerIndex, normals);
	float3 accNm = centerNm;
	if(center.z > 0.0f && length(centerNm) > 0.0f)
	{
		// Left
		if(gidx > 0)
		{
			const uint leftIndex = centerIndex-1;
			float3 left = vload3(leftIndex, positions);
			if(left.z > 0.0f && fabs(center.z - left.z) < depthThreshold)
			{
				float3 leftNm = vload3(leftIndex, normals);
				float confidence = dot(leftNm, centerNm);
				if(confidence > dotThreshold)
				{
					accNm += confidence * leftNm;
				}
				// LeftLeft
				if(gidx > 1)
				{
					const uint leftLeftIndex = centerIndex-2;
					float3 leftLeft = vload3(leftLeftIndex, positions);
					if(leftLeft.z > 0.0f && fabs(center.z - leftLeft.z) < depthThreshold)
					{
						float3 leftLeftNm = vload3(leftLeftIndex, normals);
						float confidence = dot(leftLeftNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * leftLeftNm;
						}
					}
				}
			}
		}
		// Right
		if(gidx < gsizex-1)
		{
			const uint rightIndex = centerIndex+1;
			float3 right = vload3(rightIndex, positions);
			if(right.z > 0.0f && fabs(center.z - right.z) < depthThreshold)
			{
				float3 rightNm = vload3(rightIndex, normals);
				float confidence = dot(rightNm, centerNm);
				if(confidence > dotThreshold)
				{
					accNm += confidence * rightNm;
				}
				// RightRight
				if(gidx < gsizex-2)
				{
					const uint rightRightIndex = centerIndex+2;
					float3 rightRight = vload3(rightRightIndex, positions);
					if(rightRight.z > 0.0f && fabs(center.z - rightRight.z) < depthThreshold)
					{
						float3 rightRightNm = vload3(rightRightIndex, normals);
						float confidence = dot(rightRightNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * rightRightNm;
						}
					}
				}
			}
		}
		// Up
		if(gidy > 0)
		{
			const uint upIndex = centerIndex-gsizex;
			float3 up = vload3(upIndex, positions);
			if(up.z > 0.0f && fabs(center.z - up.z) < depthThreshold)
			{
				float3 upNm = vload3(upIndex, normals);
				float confidence = dot(upNm, centerNm);
				if(confidence > dotThreshold)
				{
					accNm += confidence * upNm;
				}
				// UpUp
				if(gidy > 1)
				{
					const uint upUpIndex = centerIndex-2*gsizex;
					float3 upUp = vload3(upUpIndex, positions);
					if(upUp.z > 0.0f && fabs(center.z - upUp.z) < depthThreshold)
					{
						float3 upUpNm = vload3(upUpIndex, normals);
						float confidence = dot(upUpNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * upUpNm;
						}
					}
				}
			}
		}
		// Down
		if(gidy < gsizey-1)
		{
			const uint downIndex = centerIndex+gsizex;
			float3 down = vload3(downIndex, positions);
			if(down.z > 0.0f && fabs(center.z - down.z) < depthThreshold)
			{
				float3 downNm = vload3(downIndex, normals);
				float confidence = dot(downNm, centerNm);
				if(confidence > dotThreshold)
				{
					accNm += confidence * downNm;
				}
				// DownDown
				if(gidy < gsizey-2)
				{
					const uint downDownIndex = centerIndex+2*gsizex;
					float3 downDown = vload3(downDownIndex, positions);
					if(downDown.z > 0.0f && fabs(center.z - downDown.z) < depthThreshold)
					{
						float3 downDownNm = vload3(downDownIndex, normals);
						float confidence = dot(downDownNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * downDownNm;
						}
					}
				}
			}
		}
		// LeftUp
		if(gidx > 0 && gidy > 0)
		{
			const uint leftUpIndex = centerIndex-gsizex-1;
			float3 leftUp = vload3(leftUpIndex, positions);
			if(leftUp.z > 0.0f && fabs(center.z - leftUp.z) < depthThreshold)
			{
				float3 leftUpNm = vload3(leftUpIndex, normals);
				float confidence = dot(leftUpNm, centerNm);
				if(confidence > dotThreshold)
				{
					accNm += confidence * leftUpNm;
				}
				// LeftUpLeft
				if(gidx > 1 && gidy > 0)
				{
					const uint leftUpLeftIndex = centerIndex-gsizex-2;
					float3 leftUpLeft = vload3(leftUpLeftIndex, positions);
					if(leftUpLeft.z > 0.0f && fabs(center.z - leftUpLeft.z) < depthThreshold)
					{
						float3 leftUpLeftNm = vload3(leftUpLeftIndex, normals);
						float confidence = dot(leftUpLeftNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * leftUpLeftNm;
						}
					}
				}
				// LeftUpUp
				if(gidx > 0 && gidy > 1)
				{
					const uint leftUpUpIndex = centerIndex-2*gsizex-1;
					float3 leftUpUp = vload3(leftUpUpIndex, positions);
					if(leftUpUp.z > 0.0f && fabs(center.z - leftUpUp.z) < depthThreshold)
					{
						float3 leftUpUpNm = vload3(leftUpUpIndex, normals);
						float confidence = dot(leftUpUpNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * leftUpUpNm;
						}
					}
				}
				// LeftUpLeftUp
				if(gidx > 1 && gidy > 1)
				{
					const uint leftUpLeftUpIndex = centerIndex-2*gsizex-2;
					float3 leftUpLeftUp = vload3(leftUpLeftUpIndex, positions);
					if(leftUpLeftUp.z > 0.0f && fabs(center.z - leftUpLeftUp.z) < depthThreshold)
					{
						float3 leftUpLeftUpNm = vload3(leftUpLeftUpIndex, normals);
						float confidence = dot(leftUpLeftUpNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * leftUpLeftUpNm;
						}
					}
				}
			}
		}
		// LeftDown
		if(gidx > 0 && gidy < gsizey-1)
		{
			const uint leftDownIndex = centerIndex+gsizex-1;
			float3 leftDown = vload3(leftDownIndex, positions);
			if(leftDown.z > 0.0f && fabs(center.z - leftDown.z) < depthThreshold)
			{
				float3 leftDownNm = vload3(leftDownIndex, normals);
				float confidence = dot(leftDownNm, centerNm);
				if(confidence > dotThreshold)
				{
					accNm += confidence * leftDownNm;
				}
				// LeftDownLeft
				if(gidx > 1 && gidy < gsizey-1)
				{
					const uint leftDownLeftIndex = centerIndex+gsizex-2;
					float3 leftDownLeft = vload3(leftDownLeftIndex, positions);
					if(leftDownLeft.z > 0.0f && fabs(center.z - leftDownLeft.z) < depthThreshold)
					{
						float3 leftDownLeftNm = vload3(leftDownLeftIndex, normals);
						float confidence = dot(leftDownLeftNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * leftDownLeftNm;
						}
					}
				}
				// LeftDownDown
				if(gidx > 0 && gidy < gsizey-2)
				{
					const uint leftDownDownIndex = centerIndex+2*gsizex-1;
					float3 leftDownDown = vload3(leftDownDownIndex, positions);
					if(leftDownDown.z > 0.0f && fabs(center.z - leftDownDown.z) < depthThreshold)
					{
						float3 leftDownDownNm = vload3(leftDownDownIndex, normals);
						float confidence = dot(leftDownDownNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * leftDownDownNm;
						}
					}
				}
				// LeftDownLeftDown
				if(gidx > 1 && gidy < gsizey-2)
				{
					const uint leftDownLeftDownIndex = centerIndex+2*gsizex-2;
					float3 leftDownLeftDown = vload3(leftDownLeftDownIndex, positions);
					if(leftDownLeftDown.z > 0.0f && fabs(center.z - leftDownLeftDown.z) < depthThreshold)
					{
						float3 leftDownLeftDownNm = vload3(leftDownLeftDownIndex, normals);
						float confidence = dot(leftDownLeftDownNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * leftDownLeftDownNm;
						}
					}
				}
			}
		}
		// RightUp
		if(gidx < gsizex-1 && gidy > 0)
		{
			const uint rightUpIndex = centerIndex-gsizex+1;
			float3 rightUp = vload3(rightUpIndex, positions);
			if(rightUp.z > 0.0f && fabs(center.z - rightUp.z) < depthThreshold)
			{
				float3 rightUpNm = vload3(rightUpIndex, normals);
				float confidence = dot(rightUpNm, centerNm);
				if(confidence > dotThreshold)
				{
					accNm += confidence * rightUpNm;
				}
				// RightUpRight
				if(gidx < gsizex-2 && gidy > 0)
				{
					const uint rightUpRightIndex = centerIndex-gsizex+2;
					float3 rightUpRight = vload3(rightUpRightIndex, positions);
					if(rightUpRight.z > 0.0f && fabs(center.z - rightUpRight.z) < depthThreshold)
					{
						float3 rightUpRightNm = vload3(rightUpRightIndex, normals);
						float confidence = dot(rightUpRightNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * rightUpRightNm;
						}
					}
				}
				// RightUpUp
				if(gidx < gsizex-1 && gidy > 1)
				{
					const uint rightUpUpIndex = centerIndex-2*gsizex+1;
					float3 rightUpUp = vload3(rightUpUpIndex, positions);
					if(rightUpUp.z > 0.0f && fabs(center.z - rightUpUp.z) < depthThreshold)
					{
						float3 rightUpUpNm = vload3(rightUpUpIndex, normals);
						float confidence = dot(rightUpUpNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * rightUpUpNm;
						}
					}
				}
				// RightUpRightUp
				if(gidx < gsizex-2 && gidy > 1)
				{
					const uint rightUpRightUpIndex = centerIndex-2*gsizex+2;
					float3 rightUpRightUp = vload3(rightUpRightUpIndex, positions);
					if(rightUpRightUp.z > 0.0f && fabs(center.z - rightUpRightUp.z) < depthThreshold)
					{
						float3 rightUpRightUpNm = vload3(rightUpRightUpIndex, normals);
						float confidence = dot(rightUpRightUpNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * rightUpRightUpNm;
						}
					}
				}
			}
		}
		// RightDown
		if(gidx < gsizex-1 && gidy < gsizey-1)
		{
			const uint rightDownIndex = centerIndex+gsizex+1;
			float3 rightDown = vload3(rightDownIndex, positions);
			if(rightDown.z > 0.0f && fabs(center.z - rightDown.z) < depthThreshold)
			{
				float3 rightDownNm = vload3(rightDownIndex, normals);
				float confidence = dot(rightDownNm, centerNm);
				if(confidence > dotThreshold)
				{
					accNm += confidence * rightDownNm;
				}
				// RightDownRight
				if(gidx < gsizex-2 && gidy < gsizey-1)
				{
					const uint rightDownRightIndex = centerIndex+gsizex+2;
					float3 rightDownRight = vload3(rightDownRightIndex, positions);
					if(rightDownRight.z > 0.0f && fabs(center.z - rightDownRight.z) < depthThreshold)
					{
						float3 rightDownRightNm = vload3(rightDownRightIndex, normals);
						float confidence = dot(rightDownRightNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * rightDownRightNm;
						}
					}
				}
				// RightDownDown
				if(gidx < gsizex-1 && gidy < gsizey-2)
				{
					const uint rightDownDownIndex = centerIndex+2*gsizex+1;
					float3 rightDownDown = vload3(rightDownDownIndex, positions);
					if(rightDownDown.z > 0.0f && fabs(center.z - rightDownDown.z) < depthThreshold)
					{
						float3 rightDownDownNm = vload3(rightDownDownIndex, normals);
						float confidence = dot(rightDownDownNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * rightDownDownNm;
						}
					}
				}
				// RightDownRightDown
				if(gidx < gsizex-2 && gidy < gsizey-2)
				{
					const uint rightDownRightDownIndex = centerIndex+2*gsizex+2;
					float3 rightDownRightDown = vload3(rightDownRightDownIndex, positions);
					if(rightDownRightDown.z > 0.0f && fabs(center.z - rightDownRightDown.z) < depthThreshold)
					{
						float3 rightDownRightDownNm = vload3(rightDownRightDownIndex, normals);
						float confidence = dot(rightDownRightDownNm, centerNm);
						if(confidence > dotThreshold)
						{
							accNm += SECOND_CONFIDENCE_RATIO * confidence * rightDownRightDownNm;
						}
					}
				}
			}
		}
	}

	barrier(CLK_GLOBAL_MEM_FENCE);
	vstore3(accNm, centerIndex, normals);
}
