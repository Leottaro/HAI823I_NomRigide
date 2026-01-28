#version 330 core

in vec3 f_position;
in vec3 f_position_world_space;
in vec3 f_normal;
in vec2 f_uv;

out vec3 outColor;

void main() {
  // outColor = vec3(1.);
  // outColor = abs(f_normal);
  outColor = (f_normal + vec3(1.)) / vec3(2.);
}
