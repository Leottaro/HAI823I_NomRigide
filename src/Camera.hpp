#pragma once

// GLM
#include <glm/ext.hpp>
#include <glm/glm.hpp>

// USUAL INCLUDES
#include <math.h>
#include "Transformation.hpp"

#define CAMERA_TYPES_N 2
#define IMGUI_CAMERA_TYPES "Free\0Orbital\0"
enum CameraType {
    CameraFree,
    CameraOrbital,
};

class Camera {
public:
    CameraType m_type = CameraOrbital;
    glm::vec3 m_position = glm::vec3(1.0f, 1.0f, 1.0f);
    float m_translation_speed = 2.5f;

    glm::vec2 m_orientation = glm::vec2(-M_PI_4 * 0.5, 0.); // (pitch, yaw)
    float m_rotation_speed = 1.0f;

    float m_aspect_ratio = 1.f;
    float m_fovy = glm::pi<float>() / 2.f;
    glm::vec2 m_near_far = glm::vec2(1.e-1f, 1.e4f);

    const glm::vec3 *m_center = &VEC_ZERO; // Only in oribtal type
    float m_distance_to_center = 5.f;      // Only in oribtal type
    float m_zoom_rate = 0.05f;             // Only in oribtal type

private:
    glm::vec3 m_front;
    glm::vec3 m_right;
    glm::vec3 m_real_up;

    glm::mat4 m_view;
    glm::mat4 m_projection;

    void updateData();

    void updateKeyboardInput(GLFWwindow *_window, float _deltaTime);
    void updateMouseInput(GLFWwindow *_window, float _deltaTime, const glm::vec2 &_cursor_vel, const glm::vec2 &_scroll, bool _disable_actions);

public:
    Camera() { updateData(); };

    bool updateInterface();
    void update(GLFWwindow *_window, float _deltaTime, const glm::vec2 &_cursor_vel, const glm::vec2 &_scroll, bool _disable_mouse_actions);

    glm::vec3 getFront() const { return m_front; }
    glm::vec3 getRight() const { return m_right; }
    glm::vec3 getUp() const { return m_real_up; }

    glm::mat4 getViewMatrix() const { return m_view; }
    glm::mat4 getProjectionMatrix() const { return m_projection; }
};
