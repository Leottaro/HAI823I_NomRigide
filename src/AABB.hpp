#pragma once

// GLM
#include <glm/ext.hpp>
#include <glm/glm.hpp>

// USUAL INCLUDES
#include <limits>

template <typename T>
struct AABB {
    using vec3 = glm::vec<3, T, glm::packed_highp>;

    vec3 min;
    vec3 max;

    AABB() : min(std::numeric_limits<T>::max()), max(-std::numeric_limits<T>::max()) {}
    AABB(vec3 const& _min, vec3 const& _max) : min(_min), max(_max) {}
    AABB(const std::vector<vec3>& _points) : min(std::numeric_limits<T>::max()), max(std::numeric_limits<T>::max()) {
        for (const vec3& point : _points)
            addPosition(point);
    }

    inline void addPosition(vec3 const& v) {
        min.x = std::min(min.x, v.x);
        min.y = std::min(min.y, v.y);
        min.z = std::min(min.z, v.z);
        max.x = std::max(max.x, v.x);
        max.y = std::max(max.y, v.y);
        max.z = std::max(max.z, v.z);
    }

    inline bool isInside(vec3 const& v) const {
        return !(v.x < min.x || v.y < min.y || v.z < min.z ||
                 v.x > max.x || v.y > max.y || v.z > max.z);
    }

    inline void expand(double amount) {
        min -= amount;
        max += amount;
    }

    bool intersect(const vec3& origin, const vec3& direction, T& tmin, T& tmax) const {
        // https://www.rose-hulman.edu/class/cs/csse451/AABB/#:~:text=Axis%2DAligned%20Bounding%20Boxes%20(AABBs,bound%20and%20a%20maximum%20bound.
        vec3 delta_min = min - origin;
        vec3 delta_max = max - origin;

        tmin = delta_min.x / direction.x;
        tmax = delta_max.x / direction.x;
        if (tmin > tmax)
            std::swap(tmin, tmax);

        T tmin_tmp = delta_min.y / direction.y;
        T tmax_tmp = delta_max.y / direction.y;
        if (tmin_tmp > tmax_tmp)
            std::swap(tmin_tmp, tmax_tmp);

        if (tmax_tmp < tmin || tmin_tmp > tmax)
            return false;
        tmin = std::max(tmin, tmin_tmp);
        tmax = std::min(tmax, tmax_tmp);

        tmin_tmp = delta_min.z / direction.z;
        tmax_tmp = delta_max.z / direction.z;
        if (tmin_tmp > tmax_tmp)
            std::swap(tmin_tmp, tmax_tmp);

        if (tmax_tmp < tmin || tmin_tmp > tmax)
            return false;
        tmin = std::max(tmin, tmin_tmp);
        tmax = std::min(tmax, tmax_tmp);

        return true;
    }

    inline bool intersect(const vec3& origin, const vec3& direction) const {
        T tmin, tmax;
        return intersect(origin, direction, tmin, tmax);
    }
};