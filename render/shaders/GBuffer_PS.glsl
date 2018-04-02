#version 130

in		vec3	ViewPos;
in		vec3	Normal;
in		vec4	VertColor;

void main()
{
	gl_FragData[0] = vec4(normalize(Normal), 1.f);
}