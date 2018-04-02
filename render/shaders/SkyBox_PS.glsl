#version 130

in			vec3		TexCoord0;
uniform		vec4		gHosekCoefs[7];
uniform		vec4		gHosekRadiance;
uniform		vec4		gSunParam[2];


float PrezeHosekLuminance( float cos_theta, float cos_gamma, 
						  float A, float B, float C, float D, 
						  float E, float F, float G, float H, float I )
{
	float gamma = acos( cos_gamma );
	float expM = exp(E * gamma);
    float rayM = cos(gamma)*cos(gamma);
    float mieM = (1.0 + cos(gamma)*cos(gamma)) 
					/ pow( abs( 1.0 + I * I - 2.0* I * cos( gamma )), 1.5);
    float zenith = sqrt( cos_theta );

    return (1.0 + A * exp( B / ( cos_theta + 0.01 ))) *
            ( C + D * expM + F * rayM + G * mieM + H * zenith);
}

vec3 XYZ_to_RGB( vec3 XYZ )
{

	mat3 XYZ_to_RGB_matrix = mat3( 3.240479f, -0.969256f, 0.055648f,
									-1.537150f,  1.875992f,  -0.204043f,
									-0.498535f, 0.041556f,  1.057311f );

	vec3 rgb = XYZ_to_RGB_matrix * XYZ;
	return rgb;
}

float AngleBetween( vec3 dir0, in vec3 dir1)
{
	return acos(dot(dir0, dir1));
}

vec3 PreethamSky( vec3 dir, vec3 sun_dir )
{
	float cos_theta = max(dir.y, 0); // mode is not defined for cos_theta < 0.f
	float cos_gamma = min( 1.f, dot( dir, sun_dir ));

	float X = gHosekRadiance.x * PrezeHosekLuminance( cos_theta, cos_gamma, 
									gHosekCoefs[0].x, gHosekCoefs[0].y, gHosekCoefs[0].z, gHosekCoefs[0].w, 
									gHosekCoefs[1].x, gHosekCoefs[1].y, gHosekCoefs[1].z, gHosekCoefs[1].w,
									gHosekCoefs[6].x );

	float Y = gHosekRadiance.y * PrezeHosekLuminance( cos_theta, cos_gamma, 
									gHosekCoefs[2].x, gHosekCoefs[2].y, gHosekCoefs[2].z, gHosekCoefs[2].w, 
									gHosekCoefs[3].x, gHosekCoefs[3].y, gHosekCoefs[3].z, gHosekCoefs[3].w,
									gHosekCoefs[6].y );

	float Z = gHosekRadiance.z * PrezeHosekLuminance( cos_theta, cos_gamma, 
									gHosekCoefs[4].x, gHosekCoefs[4].y, gHosekCoefs[4].z, gHosekCoefs[4].w, 
									gHosekCoefs[5].x, gHosekCoefs[5].y, gHosekCoefs[5].z, gHosekCoefs[5].w,
									gHosekCoefs[6].z );

	vec3 XYZ = vec3( X, Y, Z );
	vec3 sky_color = XYZ_to_RGB( XYZ );
	vec3 sun_colour = gSunParam[1].xyz;
	float sun_size = gSunParam[0].w;

	// Draw a circle for the sun
	float sun_gamma = AngleBetween( dir, sun_dir );
	sky_color = mix( sun_colour, sky_color, clamp( abs( sun_gamma ) / sun_size, 0.f, 1.f ));

	return sky_color * 0.07;
}


void main()
{
	vec3 dir = normalize( TexCoord0.xyz);
	vec3 sun_dir = gSunParam[0].xyz;

	gl_FragColor = vec4( PreethamSky( dir, sun_dir ), 0.f );
}
