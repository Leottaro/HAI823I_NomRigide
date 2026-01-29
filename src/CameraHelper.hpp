#pragma once

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <math.h>

const glm::vec3 VEC_ZERO = glm::vec3(0.f, 0.f, 0.f);
const glm::vec3 VEC_UP = glm::vec3(0.f, 1.f, 0.f);
const glm::vec3 VEC_FRONT = glm::vec3(0.f, 0.f, 1.f);
const glm::vec3 VEC_RIGHT = glm::vec3(1.f, 0.f, 0.f);

class CameraHelper {
public:
    static float clipAnglePI(float _angle);
    static glm::vec3 EulerToEuclidian(const glm::vec2 &ThetaPhiR);
    static glm::vec2 EuclidianToEuler(const glm::vec3 &xyz);
};

float CameraHelper::clipAnglePI(float _angle) {
    while (_angle < -M_PI)
        _angle += 2. * M_PI;
    while (_angle > M_PI)
        _angle -= 2. * M_PI;
    return _angle;
}

glm::vec3 CameraHelper::EulerToEuclidian(const glm::vec2 &_angles) {
    float sinPhi = cosf(_angles.x);
    float x = sinPhi * sinf(_angles.y);
    float y = sinf(_angles.x);
    float z = sinPhi * cosf(_angles.y);

    return glm::vec3(x, y, z);
}

glm::vec2 CameraHelper::EuclidianToEuler(const glm::vec3 &xyz) {
    float angles_x = asin(xyz[1] / glm::length(xyz)); // polar angle from +y axis, 0..π

    float angles_y = atan2(xyz[0], xyz[2]); // azimuth around y-axis, 0..2π
    if (angles_y < 0.0f)
        angles_y += 2.0f * M_PI;

    return glm::vec2(angles_x, angles_y);
}