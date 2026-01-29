// GLFW
#include <GLFW/glfw3.h>

// GLM
#include <glm/glm.hpp>
#include <glm/ext.hpp>

// IMGUI
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

// USUAL INCLUDES
#include <iostream>
#include "Camera.hpp"
#include "CameraHelper.hpp"

#define M_PI_SAFE float(M_PI - 0.001)
#define M_PI_2_SAFE float(M_PI_2 - 0.001)
#define M_PI_4_SAFE float(M_PI_4 - 0.001)

Camera::Camera(glm::vec3 _target, float _distance, glm::vec2 _angles, float _fovy, float _speed, float _scroll_speed)
    : m_desired_distance(_distance), m_angles(_angles), m_fovy(_fovy), m_rotation_speed(_speed), m_scroll_speed(_scroll_speed) {
    clipAngles();
    glm::vec3 front = CameraHelper::EulerToEuclidian(m_angles);
    m_position = _target - _distance * front;
    m_aspect_ratio = 4. / 3.;
    updateMatrix();
}

void Camera::clipAngles() {
    m_angles = glm::vec2(
        glm::clamp(m_angles.x, -M_PI_2_SAFE, M_PI_2_SAFE),
        CameraHelper::clipAnglePI(m_angles.y));
}

void Camera::updateMatrix() {
    glm::vec3 front = CameraHelper::EulerToEuclidian(m_angles);
    glm::vec3 right = glm::cross(front, VEC_UP);
    glm::vec3 up = glm::cross(right, front);

    m_projection = glm::perspective(m_fovy, m_aspect_ratio, .1f, 200.f);
    m_view = glm::lookAt(m_position, m_position + front, up);
}

bool Camera::updateInterface(float _deltaTime) {
    float disable_mouse_actions = false;
    if (ImGui::Begin("Camera Interface")) {
        disable_mouse_actions = ImGui::IsWindowHovered() || ImGui::IsAnyItemHovered() || ImGui::IsAnyItemActive() || ImGui::IsAnyItemFocused();

        glm::vec2 angles_degree = glm::degrees(m_angles);
        bool pitch_changed = ImGui::DragFloat("Pitch", &angles_degree[0], -1., -89.944, 89.944, "%.3f°");
        bool yaw_changed = ImGui::DragFloat("Yaw", &angles_degree[1], -1., -360., 360., "%.3f°");
        if (pitch_changed || yaw_changed) {
            m_angles = glm::radians(angles_degree);
            Camera::clipAngles();
            Camera::updateMatrix();
        }

        float fovy_degree = glm::degrees(m_fovy);
        bool fovy_changed = ImGui::DragFloat("FOV", &fovy_degree, 1., 1., 179., "%.3f°");
        if (fovy_changed) {
            m_fovy = glm::radians(fovy_degree);
        }
        ImGui::DragFloat("Rotation speed", &m_rotation_speed, .001, 0., 1.);
        ImGui::DragFloat("Scroll speed", &m_scroll_speed, .01, 0., 10.);
    }

    ImGui::End();
    return disable_mouse_actions;
}

void Camera::update(GLFWwindow *_window, float _deltaTime, glm::vec3 _target_position, glm::vec2 _cursor_vel, glm::vec2 _scroll) {
    float disable_mouse_actions = updateInterface(_deltaTime);

    int window_width, window_height;
    glfwGetWindowSize(_window, &window_width, &window_height);
    m_aspect_ratio = float(window_width) / window_height;

    // input handle
    if (!disable_mouse_actions) {
        m_desired_distance = glm::max(m_desired_distance - m_scroll_speed * _scroll.y, 1.f);
        if (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
            m_angles.x -= _deltaTime * m_rotation_speed * _cursor_vel.y;
            m_angles.y -= _deltaTime * m_rotation_speed * _cursor_vel.x;
            Camera::clipAngles();
            glm::vec3 new_front = CameraHelper::EulerToEuclidian(m_angles);
            m_position = _target_position - m_desired_distance * new_front;
            updateMatrix();
            return;
        }
    }

    // update target pos
    glm::vec3 front = CameraHelper::EulerToEuclidian(m_angles);
    m_position = _target_position - m_desired_distance * front;

    // re update angle
    glm::vec3 new_front = _target_position - m_position;
    m_angles = CameraHelper::EuclidianToEuler(new_front);

    updateMatrix();
}