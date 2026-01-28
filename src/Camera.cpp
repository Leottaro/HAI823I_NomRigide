// GLFW
#include <GLFW/glfw3.h>

// GLM
#include <glm/glm.hpp>
#include <glm/ext.hpp>

// IMGUI
#include <imgui.h>
#include <imgui_impl_glfw_gl3.h>

// USUAL INCLUDES
#include <iostream>
#include "Camera.hpp"
#include "CameraHelper.hpp"

Camera::Camera(glm::vec3 _target, float _distance, glm::vec2 _angles, float _fovy, float _speed)
    : m_target(_target), m_distance(_distance), m_angles(_angles), m_fovy(_fovy), m_speed(_speed) {
    clipAngles();
    updateMatrix(4. / 3.);
}

void Camera::clipAngles() {
    m_angles = glm::vec2(
        glm::clamp(m_angles.x, float(0.001 - M_PI_2), float(M_PI_2 - 0.001)),
        CameraHelper::clipAngle2PI(m_angles.y));
}

void Camera::updateMatrix(float _aspect_ratio) {
    glm::vec3 front = m_target - m_position;
    glm::vec3 right = glm::cross(front, VEC_UP);
    glm::vec3 up = glm::cross(right, front);

    m_projection = glm::perspective(m_fovy, _aspect_ratio, .1f, 200.f);
    m_view = glm::lookAt(m_position, m_target, up);
}

void Camera::update(float _deltaTime, GLFWwindow *_window, glm::vec3 _target_position, glm::vec2 _cursor_vel, glm::vec2 _scroll) {
    int window_width, window_height;
    glfwGetWindowSize(_window, &window_width, &window_height);
    float aspect_ratio = float(window_width) / window_height;

    // input handle
    m_distance = glm::max(m_distance - _scroll.y, 1.f);
    if (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        m_angles.x -= _deltaTime * m_speed * _cursor_vel.y;
        m_angles.y -= _deltaTime * m_speed * _cursor_vel.x;
        Camera::clipAngles();
        glm::vec3 new_front = CameraHelper::EulerToEuclidian(m_angles);
        m_position = _target_position - m_distance * new_front;
        updateMatrix(aspect_ratio);
        return;
    }

    // update target pos
    glm::vec3 front = CameraHelper::EulerToEuclidian(m_angles);
    m_position = _target_position - m_distance * front;

    // re update angle
    glm::vec3 new_front = _target_position - m_position;
    m_angles = CameraHelper::EuclidianToEuler(new_front);

    updateMatrix(aspect_ratio);
}