#include "NuiExperimentCPU.h"

#define FLOAT_EPSILON 1.192092896e-07f

inline float maxmag(float a, float b)
{
	return a > b ? a : b;
}

inline float rsqrt(float x)
{
	return 1.0f / sqrt(x);
}

SgVec3f cross(const SgVec3f& v1, const SgVec3f& v2)
{
	return SgVec3f(v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2], v1[0] * v2[1] - v1[1] * v2[0]);
}

float dot(const SgVec3f& v1, const SgVec3f& v2)
{
	return (v1[0] * v1[0] + v1[1] * v2[1] + v1[2] * v2[2]);
}

SgVec3f fast_normalize(const SgVec3f& v)
{
	return v * rsqrt(dot(v, v));
}

SgVec3f computeRoots2(float b, float c)
{
	SgVec3f roots(0.f, 0.f, 0.f);
	float d = b * b - 4.f * c;
	if (d < 0.f) // no real roots!!!! THIS SHOULD NOT HAPPEN!
		d = 0.f;

	float sd = sqrt(d);

	roots[2] = 0.5f * (b + sd);
	roots[1] = 0.5f * (b - sd);
	return roots;
}


SgVec3f computeRoots3(float c0, float c1, float c2)
{
	SgVec3f roots = (SgVec3f)(0.f, 0.f, 0.f);
    
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
        float cos_theta = cos (theta);
        float sin_theta = sin (theta);

		roots[0] = c2_over_3 + 2.f * rho * cos_theta;
        roots[1] = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
        roots[2] = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

		// Sort in increasing order.
        if (roots[0] > roots[1])
		{
			//swap
			float tmp = roots[0];
			roots[0] = roots[1];
			roots[1] = tmp;
		}

        if (roots[1] > roots[2])
        {
			//swap
			float tmp = roots[2];
			roots[2] = roots[1];
			roots[1] = tmp;

			if (roots[0] > roots[1])
			{
				//swap
				float tmp = roots[0];
				roots[0] = roots[1];
				roots[1] = tmp;
			}
        }
        if (roots[0] < 0) // eigenval for symetric positive semi-definite matrix can not be negative! Set it to 0
			roots = computeRoots2 (c2, c1);
    }
	else// one root is 0 -> quadratic equation
	{
        roots = computeRoots2 (c2, c1);
    }
	return roots;
}

SgVec3f unitOrthogonal (SgVec3f src)
{
    SgVec3f perp;
    /* Let us compute the crossed product of *this with a vector
    * that is not too close to being colinear to *this.
    */

    /* unless the x and y coords are both close to zero, we can
    * simply take ( -y, x, 0 ) and normalize it.
    */
	const float prec_sqr = FLOAT_EPSILON;
    if((fabs(src[0]) > prec_sqr * fabs(src[2])) || (fabs(src[1]) > prec_sqr * fabs(src[2])))
    {
        float invnm = 1.f/sqrt(src[0]*src[0] + src[1]*src[1]);
        perp[0] = -src[1] * invnm;
        perp[1] =  src[0] * invnm;
        perp[2] = 0.0f;
    }
    /* if both x and y are close to zero, then the vector is close
    * to the z-axis, so it's far from colinear to the x-axis for instance.
    * So we take the crossed product with (1,0,0) and normalize it. 
    */
    else
    {
        float invnm = 1.f/sqrt(src[2] * src[2] + src[1] * src[1]);
        perp[0] = 0.0f;
        perp[1] = -src[2] * invnm;
        perp[2] =  src[1] * invnm;
    }
    return perp;
}

SgVec3f NuiExperimentCPU::estimateNormalsCovariance(std::vector<SgVec3f>& positions, UINT gidx, UINT gidy, UINT gsizex, UINT gsizey, float  depthThreshold)
{
	const UINT centerIndex = gidy* gsizex+gidx;
	SgVec3f center = positions[centerIndex];
	if(center[2] > 0.0f)
	{
		SgVec3f centroid = center;
		UINT validCount = 0;
		// Left
		SgVec3f Left = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f LeftLeft = (SgVec3f)(0.0f,0.0f,-1.f);
		if(gidx > 0)
		{
			Left = positions[centerIndex-1];
			if(Left[2] > 0.0f && fabs(center[2] - Left[2]) < depthThreshold)
			{
				centroid += Left;
				validCount ++;
				// LeftLeft
				if(gidx > 1)
				{
					LeftLeft = positions[centerIndex-2];
					if(LeftLeft[2] > 0.0f && fabs(center[2] - LeftLeft[2]) < depthThreshold)
					{
						centroid += LeftLeft;
						validCount ++;
					}
				}
			}
		}
		// Right
		SgVec3f Right = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f RightRight = (SgVec3f)(0.0f,0.0f,-1.f);
		if(gidx < gsizex-1)
		{
			Right = positions[centerIndex+1];
			if(Right[2] > 0.0f && fabs(center[2] - Right[2]) < depthThreshold)
			{
				centroid += Right;
				validCount ++;
				// RightRight
				if(gidx < gsizex-2)
				{
					RightRight = positions[centerIndex+2];
					if(RightRight[2] > 0.0f && fabs(center[2] - RightRight[2]) < depthThreshold)
					{
						centroid += RightRight;
						validCount ++;
					}
				}
			}
		}
		// Up
		SgVec3f Up = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f UpUp = (SgVec3f)(0.0f,0.0f,-1.f);
		if(gidy > 0)
		{
			Up = positions[centerIndex-gsizex];
			if(Up[2] > 0.0f && fabs(center[2] - Up[2]) < depthThreshold)
			{
				centroid += Up;
				validCount ++;
				// UpUp
				if(gidy > 1)
				{
					UpUp = positions[centerIndex-2*gsizex];
					if(UpUp[2] > 0.0f && fabs(center[2] - UpUp[2]) < depthThreshold)
					{
						centroid += UpUp;
						validCount ++;
					}
				}
			}
		}
		// Down
		SgVec3f Down = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f DownDown = (SgVec3f)(0.0f,0.0f,-1.f);
		if(gidy < gsizey-1)
		{
			Down = positions[centerIndex+gsizex];
			if(Down[2] > 0.0f && fabs(center[2] - Down[2]) < depthThreshold)
			{
				centroid += Down;
				validCount ++;
				// DownDown
				if(gidy < gsizey-2)
				{
					DownDown = positions[centerIndex+2*gsizex];
					if(DownDown[2] > 0.0f && fabs(center[2] - DownDown[2]) < depthThreshold)
					{
						centroid += DownDown;
						validCount ++;
					}
				}
			}
		}
		// LeftUp
		SgVec3f LeftUp = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f LeftUpLeft = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f LeftUpUp = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f LeftUpLeftUp = (SgVec3f)(0.0f,0.0f,-1.f);
		if(gidx > 0 && gidy > 0)
		{
			LeftUp = positions[centerIndex-gsizex-1];
			if(LeftUp[2] > 0.0f && fabs(center[2] - LeftUp[2]) < depthThreshold)
			{
				centroid += LeftUp;
				validCount ++;
				// LeftUpLeft
				if(gidx > 1 && gidy > 0)
				{
					LeftUp = positions[centerIndex-gsizex-2];
					if(LeftUp[2] > 0.0f && fabs(center[2] - LeftUp[2]) < depthThreshold)
					{
						centroid += LeftUp;
						validCount ++;
					}
				}
				// LeftUpUp
				if(gidx > 0 && gidy > 1)
				{
					LeftUpUp = positions[centerIndex-2*gsizex-1];
					if(LeftUpUp[2] > 0.0f && fabs(center[2] - LeftUpUp[2]) < depthThreshold)
					{
						centroid += LeftUpUp;
						validCount ++;
					}
				}
				// LeftUpLeftUp
				if(gidx > 1 && gidy > 1)
				{
					LeftUpLeftUp = positions[centerIndex-2*gsizex-2];
					if(LeftUpLeftUp[2] > 0.0f && fabs(center[2] - LeftUpLeftUp[2]) < depthThreshold)
					{
						centroid += LeftUpLeftUp;
						validCount ++;
					}
				}
			}
		}
		// LeftDown
		SgVec3f LeftDown = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f LeftDownLeft = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f LeftDownDown = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f LeftDownLeftDown = (SgVec3f)(0.0f,0.0f,-1.f);
		if(gidx > 0 && gidy < gsizey-1)
		{
			LeftDown = positions[centerIndex+gsizex-1];
			if(LeftDown[2] > 0.0f && fabs(center[2] - LeftDown[2]) < depthThreshold)
			{
				centroid += LeftDown;
				validCount ++;
				// LeftDownLeft
				if(gidx > 1 && gidy < gsizey-1)
				{
					LeftDownLeft = positions[centerIndex+gsizex-2];
					if(LeftDownLeft[2] > 0.0f && fabs(center[2] - LeftDownLeft[2]) < depthThreshold)
					{
						centroid += LeftDownLeft;
						validCount ++;
					}
				}
				// LeftDownDown
				if(gidx > 0 && gidy < gsizey-2)
				{
					LeftDownDown = positions[centerIndex+2*gsizex-1];
					if(LeftDownDown[2] > 0.0f && fabs(center[2] - LeftDownDown[2]) < depthThreshold)
					{
						centroid += LeftDownDown;
						validCount ++;
					}
				}
				// LeftDownLeftDown
				if(gidx > 1 && gidy < gsizey-2)
				{
					LeftDownLeftDown = positions[centerIndex+2*gsizex-2];
					if(LeftDownLeftDown[2] > 0.0f && fabs(center[2] - LeftDownLeftDown[2]) < depthThreshold)
					{
						centroid += LeftDownLeftDown;
						validCount ++;
					}
				}
			}
		}
		// RightUp
		SgVec3f RightUp = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f RightUpRight = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f RightUpUp = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f RightUpRightUp = (SgVec3f)(0.0f,0.0f,-1.f);
		if(gidx < gsizex-1 && gidy > 0)
		{
			RightUp = positions[centerIndex-gsizex+1];
			if(RightUp[2] > 0.0f && fabs(center[2] - RightUp[2]) < depthThreshold)
			{
				centroid += RightUp;
				validCount ++;
				// RightUpRight
				if(gidx < gsizex-2 && gidy > 0)
				{
					RightUpRight = positions[centerIndex-gsizex+2];
					if(RightUpRight[2] > 0.0f && fabs(center[2] - RightUpRight[2]) < depthThreshold)
					{
						centroid += RightUpRight;
						validCount ++;
					}
				}
				// RightUpUp
				if(gidx < gsizex-1 && gidy > 1)
				{
					RightUpUp = positions[centerIndex-2*gsizex+1];
					if(RightUpUp[2] > 0.0f && fabs(center[2] - RightUpUp[2]) < depthThreshold)
					{
						centroid += RightUpUp;
						validCount ++;
					}
				}
				// RightUpRightUp
				if(gidx < gsizex-2 && gidy > 1)
				{
					RightUpRightUp = positions[centerIndex-2*gsizex+2];
					if(RightUpRightUp[2] > 0.0f && fabs(center[2] - RightUpRightUp[2]) < depthThreshold)
					{
						centroid += RightUpRightUp;
						validCount ++;
					}
				}
			}
		}
		// RightDown
		SgVec3f RightDown = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f RightDownRight = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f RightDownDown = (SgVec3f)(0.0f,0.0f,-1.f);
		SgVec3f RightDownRightDown = (SgVec3f)(0.0f,0.0f,-1.f);
		if(gidx < gsizex-1 && gidy < gsizey-1)
		{
			RightDown = positions[centerIndex+gsizex+1];
			if(RightDown[2] > 0.0f && fabs(center[2] - RightDown[2]) < depthThreshold)
			{
				centroid += RightDown;
				validCount ++;
				// RightDownRight
				if(gidx < gsizex-2 && gidy < gsizey-1)
				{
					RightDownRight = positions[centerIndex+gsizex+2];
					if(RightDownRight[2] > 0.0f && fabs(center[2] - RightDownRight[2]) < depthThreshold)
					{
						centroid += RightDownRight;
						validCount ++;
					}
				}
				// RightDownDown
				if(gidx < gsizex-1 && gidy < gsizey-2)
				{
					RightDownDown = positions[centerIndex+2*gsizex+1];
					if(RightDownDown[2] > 0.0f && fabs(center[2] - RightDownDown[2]) < depthThreshold)
					{
						centroid += RightDownDown;
						validCount ++;
					}
				}
				// RightDownRightDown
				if(gidx < gsizex-2 && gidy < gsizey-2)
				{
					RightDownRightDown = positions[centerIndex+2*gsizex+2];
					if(RightDownRightDown[2] > 0.0f && fabs(center[2] - RightDownRightDown[2]) < depthThreshold)
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
			SgVec3f d = center - centroid;
			cov[0] = d[0] * d[0];               //cov (0, 0)
			cov[1] = d[0] * d[1];               //cov (0, 1)
			cov[2] = d[0] * d[2];               //cov (0, 2)
			cov[3] = d[1] * d[1];               //cov (1, 1)
			cov[4] = d[1] * d[2];               //cov (1, 2)
			cov[5] = d[2] * d[2];               //cov (2, 2)
			// Left
			if(Left[2] > 0.0f)
			{
				d = Left - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// LeftLeft
			if(LeftLeft[2] > 0.0f)
			{
				d = LeftLeft - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// Right
			if(Right[2] > 0.0f)
			{
				d = Right - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// RightRight
			if(RightRight[2] > 0.0f)
			{
				d = RightRight - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// Up
			if(Up[2] > 0.0f)
			{
				d = Up - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// UpUp
			if(UpUp[2] > 0.0f)
			{
				d = UpUp - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// Down
			if(Down[2] > 0.0f)
			{
				d = Down - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// DownDown
			if(DownDown[2] > 0.0f)
			{
				d = DownDown - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// LeftUp
			if(LeftUp[2] > 0.0f)
			{
				d = LeftUp - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// LeftUpLeft
			if(LeftUpLeft[2] > 0.0f)
			{
				d = LeftUpLeft - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// LeftUpUp
			if(LeftUpUp[2] > 0.0f)
			{
				d = LeftUpUp - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// LeftUpLeftUp
			if(LeftUpLeftUp[2] > 0.0f)
			{
				d = LeftUpLeftUp - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// LeftDown
			if(LeftDown[2] > 0.0f)
			{
				d = LeftDown - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// LeftDownLeft
			if(LeftDownLeft[2] > 0.0f)
			{
				d = LeftDownLeft - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// LeftDownDown
			if(LeftDownDown[2] > 0.0f)
			{
				d = LeftDownDown - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// LeftDownLeftDown
			if(LeftDownLeftDown[2] > 0.0f)
			{
				d = LeftDownLeftDown - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// RightUp
			if(RightUp[2] > 0.0f)
			{
				d = RightUp - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// RightUpRight
			if(RightUpRight[2] > 0.0f)
			{
				d = RightUpRight - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// RightUpUp
			if(RightUpRight[2] > 0.0f)
			{
				d = RightUpRight - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// RightUpRightUp
			if(RightUpRightUp[2] > 0.0f)
			{
				d = RightUpRightUp - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// RightDown
			if(RightDown[2] > 0.0f)
			{
				d = RightDown - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// RightDownRight
			if(RightDownRight[2] > 0.0f)
			{
				d = RightDownRight - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// RightDownDown
			if(RightDownDown[2] > 0.0f)
			{
				d = RightDownDown - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
			// RightDownRightDown
			if(RightDownRightDown[2] > 0.0f)
			{
				d = RightDownRightDown - centroid;
				cov[0] += d[0] * d[0];               //cov (0, 0)
				cov[1] += d[0] * d[1];               //cov (0, 1)
				cov[2] += d[0] * d[2];               //cov (0, 2)
				cov[3] += d[1] * d[1];               //cov (1, 1)
				cov[4] += d[1] * d[2];               //cov (1, 2)
				cov[5] += d[2] * d[2];               //cov (2, 2)
			}
		}

		/*float scale = fabs( maxmag(maxmag(maxmag(maxmag(maxmag(cov[0],cov[1]),cov[2]),cov[3]),cov[4]),cov[5]) );
		if (scale <= FLOAT_EPSILON)
			scale = 1.f;
		for(UINT i = 0; i < 6; ++i)
			cov[i] /= scale;*/

		// The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0.  The
		// eigenvalues are the roots to this equation, all guaranteed to be
		// real-valued, because the matrix is symmetric.
		float c0 = cov[0]*cov[3]*cov[5]+2.f*cov[1]*cov[2]*cov[4]-cov[0]*cov[4]*cov[4]-cov[3]*cov[2]*cov[2]-cov[5]*cov[1]*cov[1];
		float c1 = cov[0]*cov[3]+cov[0]*cov[5]+cov[5]*cov[3]-cov[1]*cov[1]-cov[2]*cov[2]-cov[4]*cov[4];
		float c2 = cov[0]+cov[3]+cov[5];
		SgVec3f evals = computeRoots3(c0, c1, c2);

		if(evals[2] - evals[0] <= FLOAT_EPSILON)
		{                                   
			SgVec3f evecs0 = (SgVec3f)(0.f, 0.f, -1.f);
			return (evecs0);
		}
		else if (evals[1] - evals[0] <= FLOAT_EPSILON )
		{
			// first and second equal
			SgVec3f row0 = (SgVec3f)(cov[0], cov[1], cov[2]);
			SgVec3f row1 = (SgVec3f)(cov[1], cov[3], cov[4]);
			SgVec3f row2 = (SgVec3f)(cov[2], cov[4], cov[5]);
			row0[0] -= evals[2];
			row1[1] -= evals[2];
			row2[2] -= evals[2];

			SgVec3f vec_tmp0 = cross(row0, row1);
			SgVec3f vec_tmp1 = cross(row0, row2);
			SgVec3f vec_tmp2 = cross(row1, row2);

			float len1 = dot (vec_tmp0, vec_tmp0);
			float len2 = dot (vec_tmp1, vec_tmp1);
			float len3 = dot (vec_tmp2, vec_tmp2);

			SgVec3f evecs2;
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
			SgVec3f evecs1 = unitOrthogonal(evecs2);
			SgVec3f evecs0 = fast_normalize( cross(evecs1, evecs2) );
			return (evecs0[2] > 0.f ? -evecs0 : evecs0);
		}
		else if (evals[2] - evals[1] <= FLOAT_EPSILON )
		{
			// second and third equal
			SgVec3f row0 = (SgVec3f)(cov[0], cov[1], cov[2]);
			SgVec3f row1 = (SgVec3f)(cov[1], cov[3], cov[4]);
			SgVec3f row2 = (SgVec3f)(cov[2], cov[4], cov[5]);
			row0[0] -= evals[0];
			row1[1] -= evals[0];
			row2[2] -= evals[0];

			SgVec3f vec_tmp0 = cross(row0, row1);
			SgVec3f vec_tmp1 = cross(row0, row2);
			SgVec3f vec_tmp2 = cross(row1, row2);

			float len1 = dot (vec_tmp0, vec_tmp0);
			float len2 = dot (vec_tmp1, vec_tmp1);
			float len3 = dot (vec_tmp2, vec_tmp2);

			SgVec3f evecs0;
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
			return (evecs0[2] > 0.f ? -evecs0 : evecs0);
		}
		else
		{
			SgVec3f row0(cov[0], cov[1], cov[2]);
			SgVec3f row1(cov[1], cov[3], cov[4]);
			SgVec3f row2(cov[2], cov[4], cov[5]);
			row0[0] -= evals[2];
			row1[1] -= evals[2];
			row2[2] -= evals[2];

			SgVec3f vec_tmp0 = cross(row0, row1);
			SgVec3f vec_tmp1 = cross(row0, row2);
			SgVec3f vec_tmp2 = cross(row1, row2);

			float len1 = dot (vec_tmp0, vec_tmp0);
			float len2 = dot (vec_tmp1, vec_tmp1);
			float len3 = dot (vec_tmp2, vec_tmp2);

			float mmax[3];
			UINT min_el = 2;
			UINT max_el = 2;

			SgVec3f evecs2;
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

			row0[0] = cov[0];
			row1[1] = cov[3];
			row2[2] = cov[5];
			row0[0] -= evals[1];
			row1[1] -= evals[1];
			row2[2] -= evals[1];

			vec_tmp0 = cross(row0, row1);
			vec_tmp1 = cross(row0, row2);
			vec_tmp2 = cross(row1, row2);

			len1 = dot (vec_tmp0, vec_tmp0);
			len2 = dot (vec_tmp1, vec_tmp1);
			len3 = dot (vec_tmp2, vec_tmp2);

			SgVec3f evecs1;
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

			row0[0] = cov[0];
			row1[1] = cov[3];
			row2[2] = cov[5];
			row0[0] -= evals[0];
			row1[1] -= evals[0];
			row2[2] -= evals[0];

			vec_tmp0 = cross(row0, row1);
			vec_tmp1 = cross(row0, row2);
			vec_tmp2 = cross(row1, row2);

			len1 = dot (vec_tmp0, vec_tmp0);
			len2 = dot (vec_tmp1, vec_tmp1);
			len3 = dot (vec_tmp2, vec_tmp2);

			SgVec3f evecs0;
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
			return (evecs0[2] > 0.f ? -evecs0 : evecs0);
		}
	}
	else
	{
		SgVec3f norm = (SgVec3f)(0.0f,0.0f,0.0f);
		return norm;
	}
}
