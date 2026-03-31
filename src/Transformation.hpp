#pragma once

// GLEW
#include <GL/glew.h>

// GLM
#include <glm/glm.hpp>
#include <glm/ext.hpp>

// GLFW
#include <GLFW/glfw3.h>

// USUAL INCLUDES
#include <math.h>

// helpers
#define M_PI_SAFE float(M_PI - 0.001)
#define M_PI_2_SAFE float(M_PI_2 - 0.001)
#define M_PI_4_SAFE float(M_PI_4 - 0.001)

constexpr glm::vec3 VEC_ZERO(0.f, 0.f, 0.f);
constexpr glm::vec3 VEC_RIGHT(1.f, 0.f, 0.f);
constexpr glm::vec3 VEC_UP(0.f, 1.f, 0.f);
constexpr glm::vec3 VEC_FRONT(0.f, 0.f, 1.f);

inline glm::vec3 applyTransformation(const glm::vec3 &vec, float w, const glm::mat4 &transfo) {
    glm::vec4 temp = transfo * glm::vec4(vec.x, vec.y, vec.z, w);
    return temp.w == 0. ? glm::vec3(temp.x, temp.y, temp.z) : glm::vec3(temp.x, temp.y, temp.z) / temp.w;
}

template <typename T>
inline glm::vec<3, T, glm::packed_highp> projectPointOnLine(const glm::vec<3, T, glm::packed_highp> &_origin, const glm::vec<3, T, glm::packed_highp> &_direction, const glm::vec<3, T, glm::packed_highp> &_point) {
    return _origin + _direction * glm::dot(_direction, _point - _origin);
}

class Transformation {
    glm::vec3 m_translation;
    glm::vec3 m_scale;
    glm::vec3 m_euler_angles;

public:
    Transformation(glm::vec3 _translation = glm::vec3(0.f), glm::vec3 _scale = glm::vec3(1.f), glm::vec3 _euler_angles = glm::vec3(0.f)) : m_translation(_translation), m_scale(_scale), m_euler_angles(_euler_angles) { updateRotation(); }

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

    // GETTERS
    inline const glm::vec3 &getTranslation() const { return m_translation; }
    inline const glm::vec3 &getEulerAngles() const { return m_euler_angles; }
    inline const glm::vec3 &getScale() const { return m_scale; }
    inline glm::vec3 getFrontVector() { return Transformation::EulerToEuclidian(m_euler_angles); }
    inline glm::vec3 &getTranslation() { return m_translation; }
    inline glm::vec3 &getEulerAngles() { return m_euler_angles; }
    inline glm::vec3 &getScale() { return m_scale; }

    // SETTERS
    inline void setTranslation(const glm::vec3 &t) { m_translation = t; }
    inline void setTranslationX(float tx) { m_translation.x = tx; }
    inline void setTranslationY(float ty) { m_translation.y = ty; }
    inline void setTranslationZ(float tz) { m_translation.z = tz; }

    inline void setEulerAngles(const glm::vec3 &r) { m_euler_angles = r; }
    inline void setEulerAnglesFromFront(const glm::vec3 &_front) { m_euler_angles = glm::vec3(Transformation::EuclidianToEuler(_front), 0.f); }
    inline void setPitch(float p) { m_euler_angles.x = p; }
    inline void setYaw(float y) { m_euler_angles.y = y; }
    inline void setRoll(float r) { m_euler_angles.z = r; }
    inline void addEulerAngles(const glm::vec3 &r) { m_euler_angles += r; }
    inline void addPitch(float p) { m_euler_angles.x += p; }
    inline void addYaw(float y) { m_euler_angles.y += y; }
    inline void addRoll(float r) { m_euler_angles.z += r; }

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
            m_euler_angles.z);                                       // Roll
    }

    inline glm::mat4 computeTransformationMatrix() const {
        glm::mat4 translation_matrix = glm::translate(glm::mat4(1.), m_translation);
        glm::mat4 rotation_matrix = glm::rotate(glm::mat4(1.), m_euler_angles.x, VEC_RIGHT);
        rotation_matrix = glm::rotate(rotation_matrix, m_euler_angles.y, VEC_UP);
        rotation_matrix = glm::rotate(rotation_matrix, m_euler_angles.z, VEC_FRONT);
        glm::mat4 scale_matrix = glm::scale(glm::mat4(1.), m_scale);
        return translation_matrix * rotation_matrix * scale_matrix;
    }
};