// GLFW
#include <GLFW/glfw3.h>

// GLM
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include "Transformation.hpp"

// USUAL INCLUDES

class Camera {
private:
    // Camera parameters
    Transformation m_transformation;
    float m_desired_distance;

    float m_fovy;
    float m_rotation_speed;
    float m_scroll_speed;

    glm::mat4 m_view;
    glm::mat4 m_projection;

    float m_aspect_ratio;

    void updateMatrix();

public:
    Camera(glm::vec3 _target, float _distance, glm::vec2 _angles = glm::vec2(0.), float _fovy = M_PI_4, float _speed = 0.4, float _scroll_speed = 0.4);

    bool updateInterface(float _deltaTime);
    void update(GLFWwindow *_window, float _deltaTime, glm::vec3 _target_position, glm::vec2 _cursor_vel, glm::vec2 _scroll);

    glm::mat4 getViewMatrix() const { return m_view; }
    glm::mat4 getProjectionMatrix() const { return m_projection; }
};