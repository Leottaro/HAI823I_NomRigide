#version 330 core

vec2 vertices[4] = vec2[](
    vec2(-1.0, -1.0),
    vec2(1.0, -1.0),
    vec2(-1.0, 1.0),
    vec2(1.0, 1.0));
vec2 uvs[4] = vec2[](
    vec2(0.0, 1.0),
    vec2(1.0, 1.0),
    vec2(0.0, 0.0),
    vec2(1.0, 0.0));

// Per-instance attributes
layout(location = 0) in vec3 in_center;

// Uniforms
uniform mat4 view, projection;
uniform vec3 right, up;

uniform float particle_size;

out vec2 f_uv;

void main() {
  vec4 vertex_world = vec4(in_center + particle_size * vertices[gl_VertexID].x * right + particle_size * vertices[gl_VertexID].y * up, 1.);

  f_uv = uvs[gl_VertexID];
  gl_Position = projection * view * vertex_world;
}