#pragma once

// Include GLEW
#include <GL/glew.h>

// GLM
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

// USUAL INCLUDESs
#include <iostream>

template <typename T>
struct AABB {
    using vec3 = glm::vec<3, T, glm::defaultp>;
    static constexpr T POSITIVE_EPSILON = std::numeric_limits<T>::min();
    static constexpr T NEGATIVE_EPSILON = -std::numeric_limits<T>::min();
    static constexpr T POSITIVE_MAX = std::numeric_limits<T>::max();
    static constexpr T NEGATIVE_MAX = -std::numeric_limits<T>::max();

    vec3 min;
    vec3 max;

    AABB() : min(POSITIVE_MAX), max(NEGATIVE_MAX) {}
    AABB(const vec3& _min, const vec3& _max) : min(_min), max(_max) {}

    friend std::ostream& operator<<(std::ostream& os, const AABB& aabb) {
        os << "AABB{min: " << aabb.min.x << ", " << aabb.min.y << ", " << aabb.min.z
           << " | max: " << aabb.max.x << ", " << aabb.max.y << ", " << aabb.max.z << "}";
        return os;
    }
    friend AABB operator+(const AABB& _a, const vec3& _offset) {
        return AABB(_a.min + _offset, _a.max + _offset);
    }
    friend AABB operator+(const vec3& _offset, const AABB& _a) {
        return AABB(_offset + _a.min, _offset + _a.max);
    }
    inline void expand(double amount) {
        min -= amount;
        max += amount;
    }

    inline void addPosition(const vec3& v) {
        min.x = std::min(min.x, v.x);
        min.y = std::min(min.y, v.y);
        min.z = std::min(min.z, v.z);
        max.x = std::max(max.x, v.x);
        max.y = std::max(max.y, v.y);
        max.z = std::max(max.z, v.z);
    }

    inline vec3 getCorner(int idx) const {
        return vec3(
            (idx & 1) ? max.x : min.x,
            (idx & 2) ? max.y : min.y,
            (idx & 4) ? max.z : min.z);
    }

    template <typename Func>
    inline void forAllCorners(Func&& _func) const {
        for (int i = 0; i < 8; ++i) {
            _func(getCorner(i));
        }
    }

    inline bool isInside(const vec3& v) const {
        return !(v.x < min.x || v.y < min.y || v.z < min.z ||
                 v.x > max.x || v.y > max.y || v.z > max.z);
    }

    // Face indices: -Z=0, -X=1, -Y=2, +Z=3, +X=4, +Y=5
    inline bool intersectRay(const vec3& origin, const vec3& direction, T& tmin, T& tmax) const {
        vec3 delta_min = min - origin;
        vec3 delta_max = max - origin;

        T t0, t1;
        int f0, f1;

        // X slab: -X=1, +X=4
        t0 = delta_min.x / direction.x;
        t1 = delta_max.x / direction.x;
        f0 = 1;
        f1 = 4; // -X, +X
        if (t0 > t1) {
            std::swap(t0, t1);
            std::swap(f0, f1);
        }
        tmin = t0;
        tmax = t1;

        // Y slab: -Y=2, +Y=5
        T tmin_tmp = delta_min.y / direction.y;
        T tmax_tmp = delta_max.y / direction.y;
        int fmin_tmp = 2, fmax_tmp = 5; // -Y, +Y
        if (tmin_tmp > tmax_tmp) {
            std::swap(tmin_tmp, tmax_tmp);
            std::swap(fmin_tmp, fmax_tmp);
        }

        if (tmax_tmp < tmin || tmin_tmp > tmax)
            return false;
        if (tmin_tmp > tmin) {
            tmin = tmin_tmp;
        }
        if (tmax_tmp < tmax) {
            tmax = tmax_tmp;
        }

        // Z slab: -Z=0, +Z=3
        tmin_tmp = delta_min.z / direction.z;
        tmax_tmp = delta_max.z / direction.z;
        fmin_tmp = 0;
        fmax_tmp = 3; // -Z, +Z
        if (tmin_tmp > tmax_tmp) {
            std::swap(tmin_tmp, tmax_tmp);
            std::swap(fmin_tmp, fmax_tmp);
        }

        if (tmax_tmp < tmin || tmin_tmp > tmax)
            return false;
        if (tmin_tmp > tmin) {
            tmin = tmin_tmp;
        }
        if (tmax_tmp < tmax) {
            tmax = tmax_tmp;
        }

        return true;
    }

    inline bool intersectAABB(const AABB& _other) const {
        const T overlapX = std::min(max.x, _other.max.x) - std::max(min.x, _other.min.x);
        const T overlapY = std::min(max.y, _other.max.y) - std::max(min.y, _other.min.y);
        const T overlapZ = std::min(max.z, _other.max.z) - std::max(min.z, _other.min.z);
        return !(overlapX <= T(0) || overlapY <= T(0) || overlapZ <= T(0));
    }
};