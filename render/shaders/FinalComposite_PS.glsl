#version 130

uniform		sampler2D	gBufferTex;
uniform		sampler2D	gLumTex;
uniform		sampler2D	gBloomTex;
uniform		sampler2D	gCrepuscularTex;
varying		vec2		tex_coord;
 
float CalcLuminance(vec3 color)
{
    return max( dot(color, vec3(0.299f, 0.587f, 0.114f)), 0.0001f);
}

// Applies the filmic curve from John Hable's presentation
vec3 ToneMapFilmicALU( vec3 color)
{
    color = max( vec3(0.f, 0.f, 0.f ), color - 0.004f);
    color = (color * (6.2f * color + 0.5f)) / (color * (6.2f * color + 1.7f)+ 0.06f);
	color.x = pow(color.x, 2.2f);
	color.y = pow(color.y, 2.2f);
	color.z = pow(color.z, 2.2f);

    return color;
}

// Determines the color based on exposure settings
vec3 CalcExposedColor( vec3 color, float avg_lum, float threshold, float exposure)
{
	// Use geometric mean
	avg_lum = max( avg_lum, 0.001f );
	float key_value = 0.7f;
	float linear_exposure = ( key_value / avg_lum);
	exposure = log2( max( linear_exposure, 0.0001f ));
    exposure -= threshold;
    return exp2(exposure) * color;
}


vec3 ToneMap( vec3 color, float avg_lum, float threshold, float exposure)
{
    float pixelLuminance = CalcLuminance(color);
    color = CalcExposedColor(color, avg_lum, threshold, exposure);
	color = ToneMapFilmicALU(color);
    return color;
}


void main(void) 
{
	vec3 buffer_sample = texture2D( gBufferTex, tex_coord.xy ).xyz;
	vec3 bloom_sample = texture2D( gBloomTex, tex_coord.xy ).xyz;
	vec4 crepuscular_sample = texture2D( gCrepuscularTex, tex_coord.xy );
	float avg_lum = exp( textureLod( gLumTex, tex_coord.xy, 10 ).x );

    float exposure = 0;
	float bloom_scale = 0.02f;
	vec3 composite_color = buffer_sample + bloom_sample + crepuscular_sample.xyz;

	const float occlusion_dimming = 0.3f;
	float crepuscular_occlusion = crepuscular_sample.w; 
	float occlusion_mult = 1.f - occlusion_dimming * crepuscular_occlusion;
	//composite_color *= occlusion_mult;

	vec3 color = ToneMap( composite_color, avg_lum, 0.0, exposure);
	//vec3 color = ( texture2D( gBufferTex, tex_coord.xy ).a == 0.f ) ? composite_color : ToneMap( composite_color, avg_lum, 0.0, exposure);
	//vec3 color = composite_color;

	//bloom_sample = ToneMap( bloom_sample, avg_lum, 0.0, exposure);
	color += bloom_sample * bloom_scale;
	
	//color.rgb += ( color.b > 0.5 ) ? vec3( 0,0.5,0) : vec3( 0,0,0);  
	//color.rgb = ( avg_lum > 1.0 ) ? vec3( 0,1,0) : vec3( avg_lum,avg_lum,avg_lum);  

	gl_FragColor = vec4( color.rgb, 1.f );
}