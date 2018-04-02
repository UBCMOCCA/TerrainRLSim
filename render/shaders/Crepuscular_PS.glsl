#version 130

uniform		sampler2D	gBufferTex;
uniform		vec4		gSunParams;
uniform		vec4		gLightColour;
varying		vec2		tex_coord;

vec2 ClampCoord( vec2 dest, vec2 start )
{
	vec2 pos_delta = dest - start;
	vec2 corner = vec2(( pos_delta.x > 0.f ) ? 1.f : 0.f,
						( pos_delta.y > 0.f ) ? 1.f : 0.f );

	vec2 corner_delta = vec2( min( abs( corner.x - tex_coord.x ), abs( pos_delta.x )),
								min( abs( corner.y - tex_coord.y ), abs( pos_delta.y )));
	float min_scale = min( corner_delta.x / abs( pos_delta.x ), corner_delta.y / abs( pos_delta.y ));
	pos_delta *= min_scale;

	return start + pos_delta;
}

float CalcLuminance(vec3 color)
{
    return max( dot(color, vec3(0.299f, 0.587f, 0.114f)), 0.0001f);
}

void main(void) 
{
	// get the sun's screenspace position and clamp it to the edges of the screen
	// to reduce aliasing from low sampling frequency
	vec2 sun_pos = ClampCoord( gSunParams.xy, tex_coord );
	float sun_depth = gSunParams.z + 0.1f;
	float depth_visibility = smoothstep( 0.01f, 0.2f, sun_depth * 2.f );

	//bail out
	if ( depth_visibility == 0.f )
	{
		gl_FragColor = vec4(0.f, 0.f, 0.f, 0.f );
		return;
	}

	vec2 sun_delta = sun_pos - tex_coord;
	float sun_dist_sq = sun_delta.x * sun_delta.x + sun_delta.y * sun_delta.y;
	bool close = sun_dist_sq < 0.05f;


	vec3 illumination_decay = vec3( 1.f, 1.f, 1.f );
	vec3 decay_rate = vec3( 0.89f, 0.88f, 0.87f );

	const int NUM_SAMPLES = 64;
	vec2 delta_coord = ( sun_pos - tex_coord ) / NUM_SAMPLES;
	float occlusion_count = 0.f;

	vec3 colour = vec3( 0.f, 0.f, 0.f );
	vec3 light_color = gLightColour.xyz * gLightColour.w;

	for ( int i = 0; i < NUM_SAMPLES; ++i )
	{
		vec2 cur_coord = tex_coord + i * delta_coord;
		vec4 sample = texture2D( gBufferTex, cur_coord );
     
		float occlusion = (sample.w > 0) ? 1.f : 0.f;

		colour += ( sample.xyz * light_color * illumination_decay ) * ( 1.f - occlusion );

		illumination_decay *= decay_rate;
		occlusion_count += occlusion;
	}

	float occlusion_ratio = occlusion_count / NUM_SAMPLES;

	gl_FragColor = vec4( colour, occlusion_ratio ) * depth_visibility;
}