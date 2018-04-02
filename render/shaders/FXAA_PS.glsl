#version 130

uniform		sampler2D	gBufferTex;
varying		vec2		tex_coord;
 
 float FXAALuma( vec3 rgb )
 {
	// luminance optimization, ignore blue channel since blue aliasing is rarely noticeable
	return rgb.y * ( 0.587f / 0.299f ) + rgb.x;
 }

 vec2 PixelOffset( vec2 uv, vec2 offset )
 {
	return uv + ( offset * vec2( abs( dFdx( uv.x )), abs( dFdy( uv.y ))));
 }

void main(void) {
	// Fast Approximate Anti-Aliasing!!!
	float FXAA_EDGE_THRESHOLD_MIN = 1.f / 16.f;
	float FXAA_EDGE_THRESHOLD = 1.f / 8.f;
	float FXAA_SPAN_MAX = 8.f;
	float FXAA_REDUCE_MUL = 1.f / 8.f;
	float FXAA_REDUCE_MIN = 1.f / 128.f;
	vec2 texel_size = vec2( abs( dFdx( tex_coord.x )), abs( dFdy( tex_coord.y )));

	vec3 rgb_NW = texture2D( gBufferTex, PixelOffset( tex_coord.xy, vec2( -1, -1 ))).xyz;
	vec3 rgb_NE = texture2D( gBufferTex, PixelOffset( tex_coord.xy, vec2( 1, -1 ))).xyz;
	vec3 rgb_SW = texture2D( gBufferTex, PixelOffset( tex_coord.xy, vec2( -1, 1 ))).xyz;
	vec3 rgb_SE = texture2D( gBufferTex, PixelOffset( tex_coord.xy, vec2( 1, 1 ))).xyz;
	vec3 rgb_M = texture2D( gBufferTex, tex_coord.xy).xyz;

	float luma_NW = FXAALuma( rgb_NW );
	float luma_NE = FXAALuma( rgb_NE );
	float luma_SW = FXAALuma( rgb_SW );
	float luma_SE = FXAALuma( rgb_SE );
	float luma_M = FXAALuma( rgb_M );

	float luma_min = min( luma_M, min( min( luma_NW, luma_NE ), min( luma_SW, luma_SE )));
	float luma_max = max( luma_M, max( min( luma_NW, luma_NE ), max( luma_SW, luma_SE )));

	//early bailout 
	float range = luma_max - luma_min;

	if ( range < max( FXAA_EDGE_THRESHOLD_MIN, luma_max * FXAA_EDGE_THRESHOLD ))
	{
		gl_FragColor = vec4( rgb_M.xyz, 1.f );
		//gl_FragColor = vec4( 0.f, 0.f, 0.f, 1.f );
		return;
	}

	vec2 dir;
	dir.x = -(( luma_NW + luma_NE ) - ( luma_SW + luma_SE ));
	dir.y = (( luma_NW + luma_SW ) - ( luma_NE + luma_SE ));

	float dir_reduce = max(( luma_NW + luma_NE + luma_SW + luma_SE ) * ( 0.25f * FXAA_REDUCE_MUL),
							FXAA_REDUCE_MIN );

	float rcp_dir_min = 1.f / ( min( abs( dir.x ), abs( dir.y )) + dir_reduce );

	dir = min( vec2( FXAA_SPAN_MAX, FXAA_SPAN_MAX ),
				max( vec2( -FXAA_SPAN_MAX, - FXAA_SPAN_MAX ),
					dir * rcp_dir_min )) * texel_size;

	vec3 rgb_a = 0.5f * 
		( texture2D( gBufferTex, tex_coord.xy + dir * ( 1.f / 3.f - 0.5f )).xyz
		+ texture2D( gBufferTex, tex_coord.xy + dir * ( 2.f / 3.f - 0.5f )).xyz );

	vec3 rgb_b = rgb_a * 0.5f  + 0.25f * 
		( texture2D( gBufferTex, tex_coord.xy + dir * ( -0.5f )).xyz
		+ texture2D( gBufferTex, tex_coord.xy + dir * ( 1.f - 0.5f )).xyz );

	float luma_b = FXAALuma( rgb_b );

	if (( luma_b < luma_min ) || ( luma_b > luma_max ))
	{
		gl_FragColor = vec4( rgb_a.xyz, 1.f );
	}
	else
	{
		gl_FragColor = vec4( rgb_b.xyz, 1.f );
	}
}
