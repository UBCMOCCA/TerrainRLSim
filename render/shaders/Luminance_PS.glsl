#version 130

uniform		sampler2D	gBufferTex;
varying		vec2		tex_coord;

float CalcLuminance(vec3 color)
{
    return max( dot(color, vec3(0.299f, 0.587f, 0.114f)), 0.0001f);
}

void main(void) {
	vec3 color = texture2D( gBufferTex, tex_coord.xy ).xyz;

	// calculate the luminance using a weighted average
    float luminance = CalcLuminance(color);

	gl_FragColor = vec4( log(luminance), 1.f, 1.f, 1.f );
}