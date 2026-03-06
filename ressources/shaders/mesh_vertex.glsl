#version 330 core

layout(location = 0) in vec3 v_position;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec2 v_uv;

uniform mat4 projection, model_view, normal_mat;

out vec3 f_position;
out vec3 f_position_world_space;
out vec3 f_normal;
out vec2 f_uv;

void main() {
  f_position_world_space = v_position;
  vec4 p = model_view * vec4(v_position, 1.0);
  gl_Position = projection * p;

  vec4 n = normal_mat * vec4(v_normal, 1.0);

  f_position = p.xyz;
  f_normal = normalize(v_normal);
  f_uv = v_uv;
}