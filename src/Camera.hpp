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
    glm::vec2 m_angles; // (pitch, yaw)
    float m_distance;

    float m_fovy = M_PI_4;
    float m_speed = 0.04;

    glm::mat4 m_view;
    glm::mat4 m_projection;

    void clipAngles();
    void updateMatrix(float _aspect_ratio);

public:
    Camera(glm::vec3 _target, float _distance, glm::vec2 _angles = glm::vec2(0.), float _fovy = M_PI_4, float _speed = 0.4);

    void update(float _deltaTime, GLFWwindow *_window, glm::vec3 _target_position, glm::vec2 _cursor_vel, glm::vec2 _scroll);

    glm::mat4 getViewMatrix() const { return m_view; }
    glm::mat4 getProjectionMatrix() const { return m_projection; }
};