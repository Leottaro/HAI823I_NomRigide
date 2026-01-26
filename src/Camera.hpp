// GLFW
#include <GLFW/glfw3.h>

// GLM
#include <glm/glm.hpp>
#include <glm/ext.hpp>

// IMGUI
#include <imgui.h>
#include <imgui_impl_glfw_gl3.h>

// USUAL INCLUDES

class Camera {
private:
    // Camera parameters
    glm::vec3 m_position;
    glm::vec3 m_target;
    glm::vec3 m_up;
    float m_fovy;
    float m_speed;

    glm::mat4 m_view;
    glm::mat4 m_projection;

    void updateVP(float _aspect_ratio);

public:
    Camera(glm::vec3 _position, glm::vec3 _target, glm::vec3 _up, float _fovy = M_PI / 4., float _speed = 2.5) : m_position(_position), m_target(_target), m_up(_up), m_fovy(_fovy), m_speed(_speed) {}
    Camera(float distance, glm::vec3 _target, float _fov = M_PI / 4., float _speed = 2.5) : m_position(_target + glm::vec3(0., 0., distance)), m_target(_target), m_up(glm::vec3(0., 1., 0.)), m_fovy(_fov), m_speed(_speed) {}

    void update(float _deltaTime, GLFWwindow *_window, glm::vec3 _target_position);

    glm::mat4 getViewMatrix() const { return m_view; }
    glm::mat4 getProjectionMatrix() const { return m_projection; }
};