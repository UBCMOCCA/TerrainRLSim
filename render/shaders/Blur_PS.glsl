#version 130

uniform		sampler2D	gBufferTex;
uniform		float		gWeights[8];
uniform		vec4		gOffset;
varying		vec2		tex_coord;

void main(void) 
{
	vec4 result = vec4( 0.f, 0.f, 0.f, 0.f );
	for( int i = 0; i < 8; ++i )
	{
		vec2 tap_coord0 = tex_coord.xy + i * gOffset.xy + gOffset.zw; // xy are the incremental offsets
		vec2 tap_coord1 = tex_coord.xy - i * gOffset.xy - gOffset.zw; // zw are the initial offsets, usually by half a texel for larger blurring with bilinear filtering
		result += texture2D( gBufferTex, tap_coord0 ) * gWeights[i];
		result += texture2D( gBufferTex, tap_coord1 ) * gWeights[i];
	}
	gl_FragColor = result;
}