#version 130

in		vec3	ViewPos;
in		vec3	Normal;
in		vec4	VertColor;
in		vec2	TexCoord;

uniform		sampler2D	gTexture;
uniform		sampler2D	gShadowTex;
uniform		sampler2D	gAOTex;

uniform		mat4		gShadowProj;
uniform		vec3		gLightDir;
uniform		vec3		gLightColour;
uniform		vec3		gAmbientColour[4];
uniform		vec4		gMaterialData;
uniform		vec4		gTexRes;


float CalculateOrenNayarDiffuse( vec3 world_space_normal, vec3 light_direction, 
								vec3 view_direction, float roughness )	// Roughness value [0 ~ 1], 0 is same as Lambertian
{
	// Make sure the interpolated inputs and
	// constant parameters are normalized
	vec3 n = world_space_normal; //If it's an interpolated normal, it may not be a unit vector.
	vec3 l = light_direction;
	vec3 v = view_direction;

	float v_dot_n = dot( v, n);
	float l_dot_n = dot( l, n);

	float gamma = dot ( v - n * v_dot_n, l - n * l_dot_n);
	float rough_sq = roughness * roughness;

	// A & B can be pre-computed in the outside if roughness is a constant
	float A = 1.0f - 0.5f * (rough_sq / (rough_sq + 0.57f));
	float B = 0.45f * (rough_sq / (rough_sq + 0.09));

	float C = sqrt(max(0.f, (1.0 - v_dot_n * v_dot_n) * (1.0 - l_dot_n * l_dot_n))) / max(v_dot_n, l_dot_n); //CorrectSignedZeroValues(max(v_dot_n, l_dot_n));

	float final = max( 0.0f, l_dot_n) * ( A + B * max( 0.0f, gamma) * C);

	return final;
}

vec3 CalculateMicroFacetSpecular(
	const vec3 world_normal,
	const vec3 light_direction,
	const vec3 view_direction, 
	const vec3 Cspec,	// material's parameter (important for metals)
							// Water(0.02), Plastic(0.03~0.05), Glass(0.03~0.08)
							// Iron(0.56,0.57,0.58), Copper(0.95,0.64,0.54), Gold(1.0,0.71,0.29)
							// Aluminum(0.91,0.92,0.92), Silver(0.95,0.93,0.88) in Linear space
	const float roughness )
{

	vec3 n = world_normal;
	vec3 v = view_direction;
	vec3 l = light_direction;
	vec3 h = normalize( view_direction + light_direction );
	
	// 1. Fresnel term
	float schlick_x = ( 1.f - dot( l, h) );
		
	float s2 = schlick_x * schlick_x;
	float s4 = s2 * s2;
	float t11 = s4 * schlick_x;
	vec3 F_term = Cspec + ( 1.f - Cspec) * t11;

	// 2. Distribution term
	float D_term = 1.f;
	
	// b) Beckmann : D(w) = ( 1 / m^2 cos(a)^4 ) exp ( - tan(a)^2 / m^2 )
	//		; m is the root mean square slope of microfacets parameterizing the surface's roughness.
	//		; a is angle between normal & half vector

	float rough_sq = roughness * roughness;
	float cos_alpha = dot( n, h );
	float alpha = acos( cos_alpha );
	float cos_alpha_sqr = cos_alpha * cos_alpha;
	float beckman_denominator = max( 0.00001, cos_alpha_sqr * rough_sq );

	float exponent = ( 1.0f - cos_alpha_sqr ) / beckman_denominator;
	float numerator = exp( -exponent );
	float denominator = max( 0.00001, rough_sq * 3.14159f * cos_alpha_sqr * cos_alpha_sqr );
	D_term = numerator / denominator;

	// 4. Division
	float division = 4.f * dot( n, l) * dot( n, v);
	division = ( division == 0.0 ) ? 0.00001 : division;

	// 3. Geometry term
	float G_term = 1.f;
	// a) Cook-Torrance : G = min{ 2(n'h)(n'v)/(v'h), 2(n'h)(n'l)/(l'h), 1 }
	
	float v_dot_h = dot( v, h);
	v_dot_h = ( v_dot_h == 0.0 ) ? 0.00001 : v_dot_h;
	
	float t31 = 2.f * dot( n, h) * dot( n, v) / v_dot_h;
	float t32 = 2.f * dot( n, h) * dot( n, l) / v_dot_h;
	G_term = min( min( t31, t32), 1.f);		

	// Final result

	vec3 final = max( 0.f, dot( n, l)) * F_term * G_term * D_term / division; //CorrectSignedZeroValues( division);

	return final;
}

vec3 CalculateMicroFacetBRDF( vec3 normal, vec3 light_dir, 
						vec3 light_colour, vec3 view_dir, float roughness,
						vec3 albedo )
{
	float diffuse_coef = CalculateOrenNayarDiffuse( normal, light_dir, 
								view_dir, roughness );

	vec3 c_spec = vec3(0.56,0.57,0.58);
	vec3 spec_coef = CalculateMicroFacetSpecular( normal, light_dir, view_dir,
												c_spec, roughness );

	return diffuse_coef * light_colour * albedo
			+ spec_coef * light_colour;
}

vec2 RotateDirections(vec2 Dir, float theta)
{
	float cos_theta = cos(theta);
	float sin_theta = sin(theta);
    return vec2(Dir.x*cos_theta - Dir.y*sin_theta,
                  Dir.x*sin_theta + Dir.y*cos_theta);
}

float CalculateShadow(vec3 view_pos, vec3 normal)
{
	float bias = 0.04f;
	vec4 position = vec4(view_pos + bias * normal, 1.f);

	vec3 shadow_coord = (gShadowProj * position).xyz;
	float depth = ( shadow_coord.z * 0.5 + 0.5 );// + 0.01f;
	depth = min(depth, 1);

	shadow_coord.xy = shadow_coord.xy * 0.5f + 0.5f;

	float sample_depth = ( texture2D( gShadowTex, shadow_coord.xy).r );

	// if sampling outside the texture, make sure depth is 1
	//vec2 coord_test = abs(shadow_coord.xy - 0.5f);
	//sample_depth = (coord_test.x > 0.5f || coord_test.y > 0.5f) ? 1.f : 0.f;

	sample_depth = (depth <= sample_depth) ? 1.f : 0.f;
	//sample_depth = sample_depth * 2.f - 1.f;


	// pcf taps for anti-aliasing
	vec2 poissonDisk[16] = vec2[]( vec2( -0.6474742f, 0.6028621f ),
									vec2( 0.0939157f, 0.6783564f ),
									vec2( -0.3371512f, 0.04865054f ),
									vec2( -0.4010732f, 0.914994f ),
									vec2( -0.2793565f, 0.4456959f ),
									vec2( -0.6683437f, -0.1396244f ),
									vec2( -0.6369296f, -0.6966243f ),
									vec2( -0.2684143f, -0.3756073f ),
									vec2( 0.1146429f, -0.8692533f ),
									vec2( 0.0697926f, 0.01110036f ),
									vec2( 0.4677842f, 0.5375957f ),
									vec2( 0.377133f, -0.3518053f ),
									vec2( 0.6722369f, 0.03702459f ),
									vec2( 0.6890426f, -0.5889201f ),
									vec2( -0.8208677f, 0.2444565f ),
									vec2( 0.8431721f, 0.3903837f ));
	float shadow_map_size = 2048;
	float total = 17.f;
	float r = 2;

	const vec4 noise_params = vec4(1, 1, 0.1, 0.2);
	vec2 seed = view_pos.xy * noise_params.xy + noise_params.zw;
	float rand_theta = fract(sin(dot(seed, vec2(12.9898,78.233))) * 43.5453);
	rand_theta = 2 * 3.14159f * rand_theta - 3.14159f;

	for ( int i = 0; i < 16; ++i )
	{
		vec2 dir = poissonDisk[i];
		dir = RotateDirections(dir, rand_theta);
		vec2 tap_coord = shadow_coord.xy +  r * dir / shadow_map_size;
		float tap = texture2D( gShadowTex, tap_coord ).r;
		
		//vec2 coord_test = abs(tap_coord.xy - 0.5f);
		//sample_depth = (coord_test.x > 0.5f || coord_test.y > 0.5f) ? 1.f : 0.f;
		
		//float tap_weight = ( poissonDisk[i].x * poissonDisk[i].x + poissonDisk[i].y * poissonDisk[i].y );
		sample_depth += ( depth <= tap ) ? 1.f : 0.f;
		//total += 1.f - tap_weight;
	}

	//sample_depth = sample_depth * 2.f - 1.f;
	float shadow_coef = sample_depth / total;

	return shadow_coef;
}

vec3 CalcAmbient(vec3 normal, vec3 albedo)
{
	float ambient_scale = 0.05;
	vec2 frag_coord = gl_FragCoord.xy / gTexRes.xy;
	//float ao = texture2D(gAOTex, frag_coord).r;
	vec3 ambient = ( gAmbientColour[0] * max(0.f, normal.y ) 
					+ gAmbientColour[1] * abs( normal.x )
					+ gAmbientColour[2] * abs(normal.z ) 
					+ gAmbientColour[3] * max(0.f, -normal.y )) 
					* albedo;
	ambient *= ambient_scale;
	//ambient *= ao;
	return ambient;
}

void main()
{
	float roughness = gMaterialData.x;
	bool enable_albedo_tex = gMaterialData.y != 0;
	vec3 view_dir = -normalize(ViewPos);
	vec3 norm = normalize(Normal);

	vec3 albedo = VertColor.rgb;
	if (enable_albedo_tex)
	{
		vec4 tex_col = texture2D(gTexture, TexCoord);
		albedo.rgb = albedo.rgb * tex_col.rgb;
	}
	
	float shadow_coef = CalculateShadow(ViewPos, norm);
	//vec3 light_colour = 0.002 * vec3(1000, 900, 700) * shadow_coef;
	vec3 light_colour = gLightColour * shadow_coef;
	vec3 light_result = CalculateMicroFacetBRDF(norm, gLightDir, 
										light_colour, view_dir, roughness,
										albedo);

	vec2 frag_coord = gl_FragCoord.xy / gTexRes.xy;
	float ao = texture2D(gAOTex, frag_coord).r;
	vec3 ambient = CalcAmbient(norm, albedo.rgb);
	light_result += ambient;

	gl_FragData[0] = vec4(light_result, VertColor[3]);
}