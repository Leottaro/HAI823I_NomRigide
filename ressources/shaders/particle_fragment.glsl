#version 330 core

uniform sampler2D sampler;
uniform int has_texture;

uniform vec4 color;
uniform int has_color;

in vec2 f_uv;

out vec4 outColor;

void main() {
  if (has_texture != 0) {
    outColor = texture(sampler, f_uv);
  } else if (has_color != 0) {
    outColor = color;
  } else {
    outColor = vec4(f_uv, 0., 1.);
  }
}
