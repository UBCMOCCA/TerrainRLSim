#version 130

uniform		sampler2D	gBufferTex;
uniform		sampler2D	gLumTex;
varying		vec2		tex_coord;

float CalcLuminance(vec3 color)
{
    return max( dot(color, vec3(0.299f, 0.587f, 0.114f)), 0.0001f);
}

void main(void) {
	vec4 color = texture2D( gBufferTex, tex_coord.xy );
	float avg_lum = exp( textureLod( gLumTex, tex_coord.xy, 10 ).x );

	float threshold = max( 1.f, 1/5 * avg_lum );
	float luminance = CalcLuminance( color.xyz );
	float threshold_scale = smoothstep( 0.f, 1.f, (( luminance - threshold ) * 0.5f / threshold ));
	color.xyz *= threshold_scale;
	//color.xyz = max( vec3( 0.f, 0.f, 0.f ), color.xyz - threshold );

	gl_FragColor = color;
}
