#pragma once

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <math.h>

#define M_PI_SAFE float(M_PI - 0.001)
#define M_PI_2_SAFE float(M_PI_2 - 0.001)
#define M_PI_4_SAFE float(M_PI_4 - 0.001)

const glm::vec3 VEC_ZERO = glm::vec3(0.f, 0.f, 0.f);
const glm::vec3 VEC_UP = glm::vec3(0.f, 1.f, 0.f);
const glm::vec3 VEC_FRONT = glm::vec3(0.f, 0.f, 1.f);
const glm::vec3 VEC_RIGHT = glm::vec3(1.f, 0.f, 0.f);

class Transformation {
    glm::vec3 m_translation;
    glm::quat m_rotation;
    glm::vec3 m_scale;

    glm::vec3 m_euler_angles;

    // HELPERS
    static float clipAnglePI(float _angle) {
        while (_angle < -M_PI)
            _angle += 2. * M_PI;
        while (_angle > M_PI)
            _angle -= 2. * M_PI;
        return _angle;
    }

    static glm::vec3 EulerToEuclidian(const glm::vec2 &_angles) {
        float sinPhi = cosf(_angles.x);
        float x = sinPhi * sinf(_angles.y);
        float y = sinf(_angles.x);
        float z = sinPhi * cosf(_angles.y);

        return glm::vec3(x, y, z);
    }

    static glm::vec2 EuclidianToEuler(const glm::vec3 &xyz) {
        float angles_x = asin(xyz[1] / glm::length(xyz)); // polar angle from +y axis, 0..π

        float angles_y = atan2(xyz[0], xyz[2]); // azimuth around y-axis, 0..2π
        if (angles_y < 0.0f)
            angles_y += 2.0f * M_PI;

        return glm::vec2(angles_x, angles_y);
    }

public:
    Transformation(glm::vec3 _translation = glm::vec3(0.f), glm::vec3 _scale = glm::vec3(1.f), glm::vec3 _euler_angles = glm::vec3(0.f)) : m_translation(_translation), m_scale(_scale), m_euler_angles(_euler_angles) { updateRotation(); }

    // GETTERS
    inline const glm::vec3 getTranslation() const { return m_translation; }
    inline const glm::vec3 getEulerAngles() const { return m_euler_angles; }
    inline const glm::quat getRotation() const { return m_rotation; }
    inline glm::vec3 getScale() const { return m_scale; }
    inline glm::vec3 getFrontVector() const { return Transformation::EulerToEuclidian(m_euler_angles); }

    // SETTERS
    inline void setTranslation(const glm::vec3 &t) { m_translation = t; }
    inline void setEulerAngles(const glm::vec3 &r) { m_euler_angles = r; }
    inline void addEulerAngles(const glm::vec3 &r) { m_euler_angles += r; }
    inline void setEulerAnglesFromFront(const glm::vec3 &_front) { m_euler_angles = glm::vec3(Transformation::EuclidianToEuler(_front), 0.f); }
    inline void setRotation(const glm::quat &q) { m_rotation = q; }
    inline void setScale(glm::vec3 s) { m_scale = s; }
    inline void setScale(float s) { m_scale = glm::vec3(s); }
    inline void setScaleX(float sx) { m_scale.x = sx; }
    inline void setScaleY(float sy) { m_scale.y = sy; }
    inline void setScaleZ(float sz) { m_scale.z = sz; }
    inline void setScaleXY(float s) { m_scale.x = m_scale.y = s; }
    inline void setScaleXZ(float s) { m_scale.x = m_scale.z = s; }
    inline void setScaleYZ(float s) { m_scale.y = m_scale.z = s; }

    // UPDATES
    inline void updateRotation() {
        m_euler_angles = glm::vec3(
            glm::clamp(m_euler_angles.x, -M_PI_2_SAFE, M_PI_2_SAFE), // Pitch clamp
            Transformation::clipAnglePI(m_euler_angles.y),           // Yaw clip
            m_euler_angles.z);

        m_rotation = glm::quat(m_euler_angles);
    }
    inline void updateEulerAngles() {
        m_euler_angles = glm::eulerAngles(m_rotation);
    }

    inline glm::mat4 computeTransformationMatrix() const {
        glm::mat4 scale_matrix = glm::scale(glm::mat4(1.), m_scale);
        glm::mat4 rotation_scale_matrix = glm::mat4_cast(m_rotation) * scale_matrix;
        glm::mat4 transformation_matrix = glm::translate(rotation_scale_matrix, m_translation);
        return transformation_matrix;
    }
};