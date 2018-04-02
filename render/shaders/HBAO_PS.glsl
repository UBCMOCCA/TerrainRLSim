// This is a HBAO-Shader for OpenGL, based upon nvidias directX implementation
// supplied in their SampleSDK available from nvidia.com
// The slides describing the implementation is available at
// http://www.nvidia.co.uk/object/siggraph-2008-HBAO.html

#version 150

const float PI = 3.14159265;

uniform sampler2D gDepthTex;

uniform vec4 FocalLenLinMAD;
uniform vec4 UVToViewAB;
uniform vec2 TexRes;

const float AOStrength = 1.9;
const float R = 0.3;
float TanBias = tan(30.0 * PI / 180.0);
const float MaxRadiusPixels = 100.0;

const int NumDirections = 8;
const int NumSamples = 4;

in vec2 tex_coord;

float ViewSpaceZFromDepth(float d)
{
	// [0,1] -> [-1,1] clip space
	d = d * 2.0 - 1.0;

	// Get view space Z
	return -1.0 / (FocalLenLinMAD.z * d + FocalLenLinMAD.w);
}

vec3 UVToViewSpace(vec2 uv, float z)
{
	uv = UVToViewAB.xy * uv + UVToViewAB.zw;
	return vec3(uv * z, z);
}

vec3 GetViewPos(vec2 uv)
{
	float z = ViewSpaceZFromDepth(texture(gDepthTex, uv).r);
	return UVToViewSpace(uv, z);
}

vec3 GetViewPosPoint(ivec2 uv)
{
	ivec2 coord = ivec2(gl_FragCoord.xy) + uv;
	float z = texelFetch(gDepthTex, coord, 0).r;
	return UVToViewSpace(uv, z);
}

float TanToSin(float x)
{
	return x * inversesqrt(x*x + 1.0);
}

float InvLength(vec2 V)
{
	return inversesqrt(dot(V,V));
}

float Tangent(vec3 V)
{
	return V.z * InvLength(V.xy);
}

float BiasedTangent(vec3 V)
{
	return V.z * InvLength(V.xy) + TanBias;
}

float Tangent(vec3 P, vec3 S)
{
    return -(P.z - S.z) * InvLength(S.xy - P.xy);
}

float Length2(vec3 V)
{
	return dot(V,V);
}

vec3 MinDiff(vec3 P, vec3 Pr, vec3 Pl)
{
    vec3 V1 = Pr - P;
    vec3 V2 = P - Pl;
    return (Length2(V1) < Length2(V2)) ? V1 : V2;
}

vec2 SnapUVOffset(vec2 uv)
{
    return round(uv * TexRes) / TexRes;
}

float Falloff(float d2)
{
	return d2 * (-1.0 / (R * R)) + 1.0f;
}

float HorizonOcclusion(	vec2 deltaUV,
						vec3 P,
						vec3 dPdu,
						vec3 dPdv,
						float randstep,
						float numSamples)
{
	float ao = 0;

	// Offset the first coord with some noise
	vec2 uv = tex_coord + SnapUVOffset(randstep*deltaUV);
	deltaUV = SnapUVOffset( deltaUV );

	// Calculate the tangent vector
	vec3 T = deltaUV.x * dPdu + deltaUV.y * dPdv;

	// Get the angle of the tangent vector from the viewspace axis
	float tanH = BiasedTangent(T);
	float sinH = TanToSin(tanH);

	float tanS;
	float d2;
	vec3 S;

	// Sample to find the maximum angle
	for(float s = 1; s <= numSamples; ++s)
	{
		uv += deltaUV;
		S = GetViewPos(uv);
		tanS = Tangent(P, S);
		d2 = Length2(S - P);

		// Is the sample within the radius and the angle greater?
		if(d2 < R * R && tanS > tanH)
		{
			float sinS = TanToSin(tanS);
			// Apply falloff based on the distance
			ao += Falloff(d2) * (sinS - sinH);

			tanH = tanS;
			sinH = sinS;
		}
	}
	
	return ao;
}

vec2 RotateDirections(vec2 Dir, float theta)
{
	float cos_theta = cos(theta);
	float sin_theta = sin(theta);
    return vec2(Dir.x*cos_theta - Dir.y*sin_theta,
                  Dir.x*sin_theta + Dir.y*cos_theta);
}

void ComputeSteps(inout vec2 stepSizeUv, inout float numSteps, float rayRadiusPix, float rand)
{
    // Avoid oversampling if numSteps is greater than the kernel radius in pixels
    numSteps = min(NumSamples, rayRadiusPix);

    // Divide by Ns+1 so that the farthest samples are not fully attenuated
    float stepSizePix = rayRadiusPix / (numSteps + 1);

    // Clamp numSteps if it is greater than the max kernel footprint
    float maxNumSteps = MaxRadiusPixels / stepSizePix;
    if (maxNumSteps < numSteps)
    {
        // Use dithering to avoid AO discontinuities
        numSteps = floor(maxNumSteps + rand);
        numSteps = max(numSteps, 1);
        stepSizePix = MaxRadiusPixels / numSteps;
    }

    // Step size in uv space
    stepSizeUv = stepSizePix / TexRes;
}

void main(void)
{
	vec4 noise_params = vec4(1, 1, 0.1, 0.2);
	float numDirections = NumDirections;

	vec3 P, Pr, Pl, Pt, Pb;
	P = GetViewPos(tex_coord);
	vec2 seed = P.xy * noise_params.xy + noise_params.zw;
	
	// Sample neighboring pixels
    Pr 	= GetViewPos(tex_coord + vec2( 1 / TexRes.x, 0));
    Pl 	= GetViewPos(tex_coord + vec2(-1 / TexRes.x, 0));
    Pt 	= GetViewPos(tex_coord + vec2( 0, 1 / TexRes.y));
    Pb 	= GetViewPos(tex_coord + vec2( 0,-1 / TexRes.y));

    // Calculate tangent basis vectors using the minimu difference
    vec3 dPdu = MinDiff(P, Pr, Pl);
    vec3 dPdv = MinDiff(P, Pt, Pb) * (TexRes.y * 1 / TexRes.x);

	vec2 random;
	random[0] = fract(sin(dot(seed, vec2(12.9898,78.233))) * 43.5453);
	random[1] = fract(sin(dot(seed, vec2(49.12312,216.1))) * 93.1253);
	//random[2] = fract(sin(dot(seed, vec2(23.9213,35.233))) * 13.321);
	
	// Calculate the projected size of the hemisphere
    vec2 rayRadiusUV = 0.5 * R * FocalLenLinMAD.xy / -P.z;
    float rayRadiusPix = rayRadiusUV.x * TexRes.x;

    float ao = 1.0;

    // Make sure the radius of the evaluated hemisphere is more than a pixel
    if(rayRadiusPix > 1.0)
    {
    	ao = 0.0;
    	float numSteps;
    	vec2 stepSizeUV;

    	// Compute the number of steps
		float step_rand = abs(random.x * 4);
		step_rand = 0;
    	ComputeSteps(stepSizeUV, numSteps, rayRadiusPix, step_rand);

		float alpha = 2.0 * PI / numDirections;

		// Calculate the horizon occlusion of each direction
		for(float d = 0; d < numDirections; ++d)
		{
			float theta = alpha * d;

			// Apply noise to the direction
			float rand_theta = random.y * 2 * PI - PI;
			vec2 dir = RotateDirections(vec2(cos(theta), sin(theta)), rand_theta);
			//vec2 dir = vec2(cos(theta), sin(theta));
			vec2 deltaUV = dir * stepSizeUV;

			// Sample the pixels along the direction
			ao += HorizonOcclusion(	deltaUV,
									P,
									dPdu,
									dPdv,
									random.x,
									numSteps);
		}

		// Average the results and produce the final AO
		ao = max(0, 1.0 - ao / numDirections * AOStrength);
	}

	gl_FragData[0] = vec4(ao, ao, ao, 1);
}