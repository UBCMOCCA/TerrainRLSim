#version 130

uniform		sampler2D	gBloomTex0;
uniform		sampler2D	gBloomTex1;
uniform		sampler2D	gBloomTex2;
uniform		sampler2D	gBloomTex3;
uniform		sampler2D	gBloomTex4;
varying		vec2		tex_coord;

void main(void) 
{
	vec4 buffer_sample = 0.3 * texture2D( gBloomTex0, tex_coord.xy );
	buffer_sample += 0.25 * texture2D( gBloomTex1, tex_coord.xy );
	buffer_sample += 0.2 * texture2D( gBloomTex2, tex_coord.xy );
	buffer_sample += 0.15 * texture2D( gBloomTex3, tex_coord.xy );
	buffer_sample += 0.1 * texture2D( gBloomTex4, tex_coord.xy );

	gl_FragColor = buffer_sample;
}