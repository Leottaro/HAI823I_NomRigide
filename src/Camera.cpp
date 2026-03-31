// IMGUI
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

// USUAL INCLUDES
#include "Camera.hpp"
#include <iostream>

void Camera::updateData() {
    m_orientation.x = glm::clamp(m_orientation.x, -M_PI_2_SAFE, M_PI_2_SAFE);
    m_orientation.y = Transformation::clipAnglePI(m_orientation.y);

    m_front = Transformation::EulerToEuclidian(m_orientation);
    m_right = glm::cross(m_front, VEC_UP);
    m_real_up = glm::normalize(glm::cross(m_right, m_front));

    m_projection = glm::perspective(m_fovy, m_aspect_ratio, m_near_far.x, m_near_far.y);
    m_view = glm::lookAt(m_position, m_position + m_front, m_real_up);
}

bool Camera::updateInterface() {
    bool disable_mouse_actions = false;
    if (ImGui::Begin("Camera Interface")) {
        disable_mouse_actions = ImGui::IsWindowHovered() || ImGui::IsAnyItemHovered() || ImGui::IsAnyItemActive() || ImGui::IsAnyItemFocused();

        // Camera Type Selection
        int current_type = static_cast<int>(m_type);
        if (ImGui::Combo("Camera Type", &current_type, CAMERA_TYPES)) {
            m_type = CameraType(current_type);
            switch (m_type) {
            case CameraFree:
                break;
            case CameraOrbital:
                m_distance_to_center = glm::distance(m_position, *m_center);
                m_front = glm::normalize(*m_center - m_position);
                m_orientation = Transformation::EuclidianToEuler(m_front);
                break;
            }
            updateData();
        }

        ImGui::Separator();

        if (m_type == CameraFree) { // Free position Controls
            bool position_changed = ImGui::DragFloat3("Position", &m_position[0], 0.1f);
            if (position_changed) {
                updateData();
            }
            ImGui::Separator();
        }

        // Orientation Controls
        glm::vec2 angles_degree = glm::degrees(m_orientation);
        bool pitch_changed = ImGui::DragFloat("Pitch", &angles_degree[0], 1.f, -89.943f, 89.943f, "%.3f°");
        bool yaw_changed = ImGui::DragFloat("Yaw", &angles_degree[1], -1.f, -180.f, 180.f, "%.3f°");
        if (pitch_changed || yaw_changed) {
            m_orientation = glm::radians(angles_degree);
            updateData();
        }

        ImGui::Separator();

        // FOV Control
        float fovy_degree = glm::degrees(m_fovy);
        bool fovy_changed = ImGui::DragFloat("FOV", &fovy_degree, 0.1f, 1.f, 179.f, "%.3f°");
        if (fovy_changed) {
            m_fovy = glm::radians(fovy_degree);
        }

        ImGui::Separator();

        // Speed Controls
        if (m_type == CameraFree) {
            ImGui::DragFloat("Translation Speed", &m_translation_speed, 1.e-2f, 0.f, 1.e2f);
        } else {
            bool distance_changed = ImGui::DragFloat("Distance to Center", &m_distance_to_center, 0.1f, 1.e-4f, 1.e4f);
            if (distance_changed) {
                updateData();
            }
            ImGui::DragFloat("Zoom Rate", &m_zoom_rate, 1.e-4f, 0.f, 1.f);
        }
        ImGui::DragFloat("Rotation Speed", &m_rotation_speed, 1.e-4f, 0.f, 1.e2f);
    }

    ImGui::End();
    return disable_mouse_actions;
}

void Camera::updateKeyboardInput(GLFWwindow *_window, float _deltaTime) {
    static bool c_was_pressed = false;
    bool c_is_pressed = glfwGetKey(_window, GLFW_KEY_C) == GLFW_PRESS;
    if (c_is_pressed && !c_was_pressed) {
        m_type = CameraType((int(m_type) + 1) % CAMERA_TYPES_N);
        switch (m_type) {
        case CameraFree:
            break;
        case CameraOrbital:
            m_distance_to_center = glm::distance(m_position, *m_center);
            m_front = glm::normalize(*m_center - m_position);
            m_orientation = Transformation::EuclidianToEuler(m_front);
            break;
        }
    }

    if (m_type == CameraFree) {
        float translation_speed = _deltaTime * m_translation_speed;
        if (glfwGetKey(_window, GLFW_KEY_W) == GLFW_PRESS) {
            m_position += m_front * translation_speed;
        } else if (glfwGetKey(_window, GLFW_KEY_S) == GLFW_PRESS) {
            m_position -= m_front * translation_speed;
        } else if (glfwGetKey(_window, GLFW_KEY_A) == GLFW_PRESS) {
            m_position -= m_right * translation_speed;
        } else if (glfwGetKey(_window, GLFW_KEY_D) == GLFW_PRESS) {
            m_position += m_right * translation_speed;
        }
    }

    c_was_pressed = c_is_pressed;
}

void Camera::updateMouseInput(GLFWwindow *_window, float _deltaTime, const glm::vec2 &_cursor_vel, const glm::vec2 &_scroll, bool _disable_actions) {
    bool is_moving = glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && glfwGetKey(_window, GLFW_KEY_LEFT_SHIFT) == GLFW_RELEASE;

    float rotation_speed = _deltaTime * m_rotation_speed;
    switch (m_type) {
    case CameraFree:
        if (!_disable_actions && is_moving) {
            m_orientation.x -= rotation_speed * _cursor_vel.y;
            m_orientation.y -= rotation_speed * _cursor_vel.x;
        }
        break;
    case CameraOrbital:
        if (!_disable_actions) {
            m_distance_to_center = glm::max(m_distance_to_center * (1.f - _scroll.y * m_zoom_rate), 1.e-4f);
            if (is_moving) {
                m_orientation.x -= rotation_speed * _cursor_vel.y;
                m_orientation.y -= rotation_speed * _cursor_vel.x;
                updateData();
                m_position = *m_center - m_distance_to_center * m_front;
                break;
            }
        }

        // update target pos
        m_position = *m_center - m_distance_to_center * m_front;

        // re update angle
        m_front = *m_center - m_position;
        m_orientation = Transformation::EuclidianToEuler(m_front);
        break;
    }
}

void Camera::update(GLFWwindow *_window, float _deltaTime, const glm::vec2 &_cursor_vel, const glm::vec2 &_scroll, bool _disable_mouse_actions) {
    int window_width, window_height;
    glfwGetWindowSize(_window, &window_width, &window_height);
    m_aspect_ratio = float(window_width) / window_height;

    updateKeyboardInput(_window, _deltaTime);
    updateMouseInput(_window, _deltaTime, _cursor_vel, _scroll, _disable_mouse_actions);

    updateData();
}
