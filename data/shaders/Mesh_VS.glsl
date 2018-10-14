#version 130

uniform mat4 	uModelViewMatrix;
uniform mat4 	uProjectionMatrix;
uniform	vec4	uColor;

in 		vec3 	inPosition;
in 		vec3 	inNormal;
in 		vec2 	inTexCoord;

out 	vec3 	ViewPos;
out 	vec3 	Normal;
out		vec4 	VertColor;
out 	vec2 	TexCoord;

void main()
{
	vec4 view_pos = uModelViewMatrix * vec4(inPosition.xyz, 1.f);
    vec4 WVP_Pos = uProjectionMatrix * view_pos;

    gl_Position = WVP_Pos;
	ViewPos = view_pos.xyz;
	Normal = normalize(uModelViewMatrix * vec4(inNormal.xyz, 0.f)).xyz;
	VertColor = uColor;
	TexCoord = inTexCoord.xy;
}
