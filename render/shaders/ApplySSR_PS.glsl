#version 130

uniform		sampler2D	gReflectionTex;
varying		vec2		tex_coord;

void main(void) {
	gl_FragColor = vec4(texture2D(gReflectionTex, tex_coord.xy).rgb, 0);
}