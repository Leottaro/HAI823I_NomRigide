#version 330 core

in vec3 f_position;
in vec3 f_position_world_space;
in vec3 f_normal;
in vec2 f_uv;

out vec4 outColor;

void main() {
  // outColor = vec3(1.);
  // outColor = abs(f_normal);
  outColor = (vec4(f_normal, 1.) + vec4(1.)) / vec4(2.);
}
