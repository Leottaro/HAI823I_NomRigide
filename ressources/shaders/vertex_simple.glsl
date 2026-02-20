#version 330 core

layout(location = 0) in vec3 v_position;

uniform mat4 projection, view;

void main() {
  gl_Position = projection * view * vec4(v_position, 1.0);
}