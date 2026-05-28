#pragma once

// GLEW
#include <GL/glew.h>

// GLM
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/norm.hpp>

// GLFW
#include <GLFW/glfw3.h>

// USUAL INCLUDES
#include <math.h>
#include <algorithm>

// helpers
#define M_PI_SAFE float(M_PI - 0.001)
#define M_PI_2_SAFE float(M_PI_2 - 0.001)
#define M_PI_4_SAFE float(M_PI_4 - 0.001)

constexpr glm::vec3 VEC_ZERO(0.f, 0.f, 0.f);
constexpr glm::vec3 VEC_RIGHT(1.f, 0.f, 0.f);
constexpr glm::vec3 VEC_UP(0.f, 1.f, 0.f);
constexpr glm::vec3 VEC_FRONT(0.f, 0.f, 1.f);

template <typename T>
inline glm::vec<3, T, glm::defaultp> applyTransformation(const glm::vec<3, T, glm::defaultp>& vec, T w, const glm::mat<4, 4, T, glm::defaultp>& transfo) {
    glm::vec<4, T, glm::defaultp> temp = transfo * glm::vec4(vec.x, vec.y, vec.z, w);
    return temp.w == 0. ? glm::vec<3, T, glm::defaultp>(temp.x, temp.y, temp.z) : glm::vec<3, T, glm::defaultp>(temp.x, temp.y, temp.z) / temp.w;
}

template <typename T>
inline glm::vec<3, T, glm::defaultp> projectVectorOnPlane(const glm::vec<3, T, glm::defaultp>& _vec, const glm::vec<3, T, glm::defaultp>& _normal) {
    return glm::cross(glm::normalize(_normal), glm::cross(_vec, glm::normalize(_normal)));
}
template <typename T>
inline glm::vec<3, T, glm::defaultp> projectPointOnPlane(const glm::vec<3, T, glm::defaultp>& _point, const glm::vec<3, T, glm::defaultp>& _origin, const glm::vec<3, T, glm::defaultp>& _normal) {
    return _origin + projectVectorOnPlane(_point - _origin, _normal);
}
template <typename T>
inline glm::vec<3, T, glm::defaultp> projectVectorOnLine(const glm::vec<3, T, glm::defaultp>& _vec, const glm::vec<3, T, glm::defaultp>& _direction) {
    return glm::dot(_vec, _direction) * _direction;
}
template <typename T>
inline glm::vec<3, T, glm::defaultp> projectPointOnLine(const glm::vec<3, T, glm::defaultp>& _point, const glm::vec<3, T, glm::defaultp>& _origin, const glm::vec<3, T, glm::defaultp>& _direction) {
    return _origin + projectVectorOnLine(_point - _origin, _direction);
}

template <typename T>
bool computeBarycentrics(const glm::vec<3, T, glm::defaultp>& v0, const glm::vec<3, T, glm::defaultp>& v1, const glm::vec<3, T, glm::defaultp>& v2, const glm::vec<3, T, glm::defaultp>& normal, const glm::vec<3, T, glm::defaultp>& p, glm::vec<3, T, glm::defaultp>& barycentrics) {
    T total_area_sq = glm::length2(normal);
    if (total_area_sq < T(1.e-8))
        return false;

    // Signed barycentric coordinates
    barycentrics.x = glm::dot(glm::cross(v1 - p, v2 - p), normal) / total_area_sq;
    barycentrics.y = glm::dot(glm::cross(v2 - p, v0 - p), normal) / total_area_sq;
    barycentrics.z = glm::dot(glm::cross(v0 - p, v1 - p), normal) / total_area_sq;

    if (barycentrics.x < T(-1e-5) || T(1. + 1e-5) < barycentrics.x ||
        barycentrics.y < T(-1e-5) || T(1. + 1e-5) < barycentrics.y ||
        barycentrics.z < T(-1e-5) || T(1. + 1e-5) < barycentrics.z) {
        return false;
    }

    return true;
}

template <typename T>
bool rayTriangleIntersection(const glm::vec<3, T, glm::defaultp>& origin, const glm::vec<3, T, glm::defaultp>& direction,
                             const glm::vec<3, T, glm::defaultp>& v0, const glm::vec<3, T, glm::defaultp>& v1, const glm::vec<3, T, glm::defaultp>& v2, const glm::vec<3, T, glm::defaultp>& normal,
                             T& t, glm::vec<3, T, glm::defaultp>& intersection, glm::vec<3, T, glm::defaultp>& barycentrics) {
    // Check if ray is parallel
    T dot = glm::dot(direction, normal);
    if (std::abs(dot) <= T(1.e-8)) {
        return false;
    }

    // determine intersection
    t = -(glm::dot(normal, origin - v0)) / dot;
    intersection = origin + t * direction;

    // barycentric coordinates
    return computeBarycentrics(v0, v1, v2, normal, intersection, barycentrics);
}

// https://www.desmos.com/calculator/eeqkstj2ck
template <typename T>
glm::vec<3, T, glm::defaultp> fallbackInTriangle(const glm::vec<3, T, glm::defaultp>& p1, const glm::vec<3, T, glm::defaultp>& p2, glm::vec<3, T, glm::defaultp>& project_on_plane) {
    glm::vec<3, T, glm::defaultp> direction = p2 - p1;
    T n_squared = std::pow(glm::distance(p2, p1), 2);
    T dot = glm::dot(direction, project_on_plane - p1);
    T dot_over_one = dot / n_squared;
    T r = std::clamp(dot_over_one, T(0), T(1));
    return p1 + direction * r;
};
template <typename T>
inline T closestPointInTriangle(const glm::vec<3, T, glm::defaultp>& point,
                                const glm::vec<3, T, glm::defaultp>& v0, const glm::vec<3, T, glm::defaultp>& v1, const glm::vec<3, T, glm::defaultp>& v2, const glm::vec<3, T, glm::defaultp>& normal,
                                glm::vec<3, T, glm::defaultp>& surface, glm::vec<3, T, glm::defaultp>& barycentrics) {
    glm::vec<3, T, glm::defaultp> project_on_plane = v0 + glm::cross(normal, glm::cross(point - v0, normal));
    computeBarycentrics(v0, v1, v2, normal, project_on_plane, barycentrics);
    surface = barycentrics[0] < T(0)   ? fallbackInTriangle(v1, v2, project_on_plane)
              : barycentrics[1] < T(0) ? fallbackInTriangle(v2, v0, project_on_plane)
              : barycentrics[2] < T(0) ? fallbackInTriangle(v0, v1, project_on_plane)
                                       : project_on_plane;

    return glm::distance(point, surface);
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

    static glm::vec3 EulerToEuclidian(const glm::vec2& _angles) {
        float sinPhi = cosf(_angles.x);
        float x = sinPhi * sinf(_angles.y);
        float y = sinf(_angles.x);
        float z = sinPhi * cosf(_angles.y);

        return glm::vec3(x, y, z);
    }

    static glm::vec2 EuclidianToEuler(const glm::vec3& xyz) {
        float angles_x = asin(xyz[1] / glm::length(xyz)); // polar angle from +y axis, 0..π

        float angles_y = atan2(xyz[0], xyz[2]); // azimuth around y-axis, 0..2π
        if (angles_y < 0.0f)
            angles_y += 2.0f * M_PI;

        return glm::vec2(angles_x, angles_y);
    }

    // GETTERS
    inline const glm::vec3& getTranslation() const { return m_translation; }
    inline const glm::vec3& getEulerAngles() const { return m_euler_angles; }
    inline const glm::vec3& getScale() const { return m_scale; }
    inline glm::vec3 getFrontVector() { return Transformation::EulerToEuclidian(m_euler_angles); }
    inline glm::vec3& getTranslation() { return m_translation; }
    inline glm::vec3& getEulerAngles() { return m_euler_angles; }
    inline glm::vec3& getScale() { return m_scale; }

    // SETTERS
    inline void setTranslation(const glm::vec3& t) { m_translation = t; }
    inline void setTranslationX(float tx) { m_translation.x = tx; }
    inline void setTranslationY(float ty) { m_translation.y = ty; }
    inline void setTranslationZ(float tz) { m_translation.z = tz; }

    inline void setEulerAngles(const glm::vec3& r) { m_euler_angles = r; }
    inline void setEulerAnglesFromFront(const glm::vec3& _front) { m_euler_angles = glm::vec3(Transformation::EuclidianToEuler(_front), 0.f); }
    inline void setPitch(float p) { m_euler_angles.x = p; }
    inline void setYaw(float y) { m_euler_angles.y = y; }
    inline void setRoll(float r) { m_euler_angles.z = r; }
    inline void addEulerAngles(const glm::vec3& r) { m_euler_angles += r; }
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