#version 330 core

layout(location = 0) in vec3 v_position;
layout(location = 1) in vec3 v_normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 FragPos;
out vec3 Normal;

void main() {

    FragPos = vec3(model * vec4(v_position, 1.0));

    Normal = mat3(transpose(inverse(model))) * v_normal;

    gl_Position = projection * view * vec4(FragPos, 1.0);
}