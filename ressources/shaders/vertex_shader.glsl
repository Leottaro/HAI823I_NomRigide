#version 330 core

layout(location = 0) in vec3 position;

uniform mat4 view, projection;

void main() {
  gl_Position = projection * view * vec4(position.xyz, 1.0);
}