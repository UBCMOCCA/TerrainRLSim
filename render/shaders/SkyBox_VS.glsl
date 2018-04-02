#version 130

in vec3 Position;
// uniform mat4 gl_ModelViewMatrix;
// uniform mat4 gl_ProjectionMatrix;
varying vec3 TexCoord0;

void main()
{
	vec3 view_pos = mat3( gl_ModelViewMatrix ) * Position.xyz;
    vec4 WVP_Pos = gl_ProjectionMatrix * vec4( view_pos, 1.f );
	WVP_Pos = vec4(WVP_Pos.xyww);
    gl_Position = WVP_Pos;
    TexCoord0 = Position.xyz;
}
