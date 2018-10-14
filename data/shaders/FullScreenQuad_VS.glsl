#version 130

in vec3 inPosition;
in vec3 inNormal;
in vec2 inTexCoord;

out vec3 Normal;
out vec2 TexCoord;
 
void main(void) {
  gl_Position = vec4(inPosition.xy, 0.0, 1.0);
  Normal = inNormal;
  TexCoord = inTexCoord;
}