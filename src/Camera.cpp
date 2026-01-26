// GLFW
#include <GLFW/glfw3.h>

// GLM
#include <glm/glm.hpp>
#include <glm/ext.hpp>
using namespace glm;

// IMGUI
#include <imgui.h>
#include <imgui_impl_glfw_gl3.h>

// USUAL INCLUDES
#include <iostream>
#include "Camera.hpp"

void Camera::updateVP(float _aspect_ratio) {
    m_projection = glm::perspective(m_fovy, _aspect_ratio, .1f, 200.f);
    m_view = glm::lookAt(m_position, m_target, m_up);
}

void Camera::update(float _deltaTime, GLFWwindow *_window, glm::vec3 _target_position) {
    int window_width, window_height;
    glfwGetWindowSize(_window, &window_width, &window_height);
    float aspect_ratio = float(window_width) / window_height;

    float speed = _deltaTime * m_speed;
    vec3 front = m_target - m_position;
    vec3 right = glm::cross(front, m_up);
    vec3 up = glm::cross(right, front);
    if (glfwGetKey(_window, GLFW_KEY_RIGHT) == GLFW_PRESS)
        m_position += speed * right;
    if (glfwGetKey(_window, GLFW_KEY_LEFT) == GLFW_PRESS)
        m_position -= speed * right;
    if (glfwGetKey(_window, GLFW_KEY_UP) == GLFW_PRESS)
        m_position += speed * up;
    if (glfwGetKey(_window, GLFW_KEY_DOWN) == GLFW_PRESS)
        m_position -= speed * up;

    updateVP(aspect_ratio);
}