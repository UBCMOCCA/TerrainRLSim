#version 130

in vec2 tex_coord;

uniform mat4 gl_ProjectionMatrix;

uniform sampler2D gDepthTex;
uniform sampler2D gNormalTex;
uniform sampler2D gScreenTex;
uniform sampler2D gDepthBackTex;

uniform vec4 FocalLenLinMAD;
uniform vec4 UVToViewAB;
uniform vec2 TexRes;

const float gSSRadius = 400;
const int gNumRaySteps = 128;

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
	float z = ViewSpaceZFromDepth(texture2D(gDepthTex, uv).r);
	return UVToViewSpace(uv, z);
}

vec3 raytrace(vec3 start, vec3 dir, float start_depth)
{
	vec3 color = vec3(0.0f);
	int num_steps = gNumRaySteps;
	vec3 end = start + dir;

	vec4 a = vec4(tex_coord, start_depth, 0);

	vec4 b = gl_ProjectionMatrix * vec4(end, 1);
	b.xyz /= b.w;
	b.xyz = 0.5f * b.xyz + 0.5;

	vec3 ss_reflect = b.xyz - a.xyz;
	float ss_len = length(ss_reflect.xy);
	float ss_scale = gSSRadius / (TexRes.y * num_steps * ss_len);
	ss_reflect *= ss_scale;

	if (ss_len > 0.001f)
	{
		for (int i = 0; i < num_steps; ++i)
		{
			vec3 curr_coord = a.xyz + (i + 1) * ss_reflect;
			float curr_depth = curr_coord.z;

			vec2 sample_coord = curr_coord.xy;
			float sample_depth = texture2D(gDepthTex, sample_coord).r;

			float delta = abs(sample_depth - curr_depth);
			if (curr_depth > sample_depth && delta < 0.004f
				&& sample_depth < 1)
			{
				if (delta > 0.003f)
				{
					color = texture2D(gScreenTex, sample_coord).rgb;

					float fade = i / (num_steps - 1.f);
					fade = pow(fade, 2);
					fade = 1.f - fade;

					vec2 dist = 2 * (sample_coord.xy - vec2(0.5f, 0.5f));
					float coord_fade = sqrt(dot(dist, dist));
					coord_fade = clamp(coord_fade, 0.f, 1.f);

					coord_fade = 1 - coord_fade;
					color *= coord_fade;

					break;
				}
			}

			if (curr_coord.x > 1.f || curr_coord.x < 0.f ||
				curr_coord.y > 1.f || curr_coord.y < 0.f)
			{
				break;
			}
		}
	}
 
	return color;
}

void main(void)
{
	vec3 reflect_col = vec3(0.f);

	vec3 normal = normalize(texture2D(gNormalTex, tex_coord.xy).xyz);
	vec3 pos = GetViewPos(tex_coord);

	vec3 view_dir = normalize(pos);
	vec3 reflect_dir = reflect(view_dir, normal);
	
	vec4 noise_params = vec4(1, 1, 0.1, 0.2);
	vec2 seed = pos.xy * noise_params.xy + noise_params.zw;
	float jitter = 0.05;
	reflect_dir.x += jitter * fract(sin(dot(seed, vec2(12.9898,78.233))) * 43.5453);
	reflect_dir.y += jitter * fract(sin(dot(seed, vec2(49.12312,216.1))) * 93.1253);
	reflect_dir.z += jitter * fract(sin(dot(seed, vec2(23.9213,35.233))) * 13.321);
	reflect_dir = normalize(reflect_dir);

	float z_threshold = -0.5;

	float depth = texture2D(gDepthTex, tex_coord.xy).r;
	if (depth < 1)
	{
		reflect_col = raytrace(pos, reflect_dir, depth);
		
		float fade = clamp(dot(reflect_dir, normal), 0, 1);
		fade = pow(fade, 2);
		fade = 1 - fade;
		reflect_col *= fade;
	}
	
	vec4 a = gl_ProjectionMatrix * vec4(pos, 1);
	a.xyz /= a.w;
	a.xyz = 0.5f * a.xyz + 0.5;

	reflect_col.r = max(0, reflect_col.r);
	reflect_col.g = max(0, reflect_col.g);
	reflect_col.b = max(0, reflect_col.b);

	gl_FragData[0] = vec4(reflect_col, 1);
}