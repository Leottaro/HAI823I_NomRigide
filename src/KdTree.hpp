#pragma once

#include "AABB.hpp"
#include <vector>
#include <memory>
#include <algorithm>
#include "Transformation.hpp"

struct KdTriangle {
    glm::vec3 v0, v1, v2;
    size_t triangle_index;
    glm::vec3 normal;

    KdTriangle() {}
    KdTriangle(const glm::vec3& _v0, const glm::vec3& _v1, const glm::vec3& _v2, size_t _index) : v0(_v0), v1(_v1), v2(_v2), triangle_index(_index), normal(glm::normalize(glm::cross(_v1 - _v0, _v2 - _v0))) {}

    bool isBefore(size_t axis, float pos) const {
        return v0[axis] - 1.e-4f <= pos ||
               v1[axis] - 1.e-4f <= pos ||
               v2[axis] - 1.e-4f <= pos;
    }

    bool isAfter(size_t axis, float pos) const {
        return v0[axis] + 1.e-4f >= pos ||
               v1[axis] + 1.e-4f >= pos ||
               v2[axis] + 1.e-4f >= pos;
    }
};

class KdTree {
    AABB<float> m_aabb{};
    size_t m_leaf_size{};
    size_t m_split_axis{};
    bool m_leaf{};
    std::unique_ptr<KdTree> m_left{};
    std::unique_ptr<KdTree> m_right{};
    std::vector<KdTriangle> m_triangles{};

public:
    KdTree() {}

    KdTree(KdTree&& other) noexcept
        : m_aabb(std::move(other.m_aabb)), m_leaf_size(other.m_leaf_size), m_split_axis(other.m_split_axis),
          m_leaf(other.m_leaf), m_left(std::move(other.m_left)), m_right(std::move(other.m_right)),
          m_triangles(std::move(other.m_triangles)) {}
    KdTree& operator=(KdTree&& other) noexcept {
        if (this != &other) {
            m_aabb = std::move(other.m_aabb);
            m_leaf_size = other.m_leaf_size;
            m_split_axis = other.m_split_axis;
            m_leaf = other.m_leaf;
            m_left = std::move(other.m_left);
            m_right = std::move(other.m_right);
            m_triangles = std::move(other.m_triangles);
        }
        return *this;
    }
    KdTree(const KdTree& other)
        : m_aabb(other.m_aabb), m_leaf_size(other.m_leaf_size), m_split_axis(other.m_split_axis),
          m_leaf(other.m_leaf), m_triangles(other.m_triangles) {
        if (other.m_left) {
            m_left = std::make_unique<KdTree>(*other.m_left);
        }
        if (other.m_right) {
            m_right = std::make_unique<KdTree>(*other.m_right);
        }
    }
    KdTree& operator=(const KdTree& other) {
        if (this != &other) {
            m_aabb = other.m_aabb;
            m_leaf_size = other.m_leaf_size;
            m_split_axis = other.m_split_axis;
            m_leaf = other.m_leaf;
            m_triangles = other.m_triangles;
            m_left = other.m_left ? std::make_unique<KdTree>(*other.m_left) : nullptr;
            m_right = other.m_right ? std::make_unique<KdTree>(*other.m_right) : nullptr;
        }
        return *this;
    }
    ~KdTree() = default;

    KdTree(std::vector<KdTriangle>& _kd_triangles, const AABB<float>& _aabb, size_t _leaf_size, uint8_t _split_axis) : m_aabb(_aabb), m_leaf_size(_leaf_size), m_split_axis(_split_axis), m_leaf(_kd_triangles.size() <= _leaf_size) {
        if (m_leaf) {
            m_triangles = _kd_triangles;
            return;
        }

        // sort the triangles and split by the median // TODO: already sorted in input
        uint8_t sort_split_axis = m_split_axis;
        std::sort(_kd_triangles.begin(), _kd_triangles.end(), [sort_split_axis](const KdTriangle& a, const KdTriangle& b) { return a.v0[sort_split_axis] + a.v1[sort_split_axis] + a.v2[sort_split_axis] < b.v0[sort_split_axis] + b.v1[sort_split_axis] + b.v2[sort_split_axis]; });
        size_t median_i = _kd_triangles.size() / 2;
        float split_pos = (_kd_triangles[median_i].v0[m_split_axis] + _kd_triangles[median_i].v1[m_split_axis] + _kd_triangles[median_i].v2[m_split_axis]) / 3.f;

        // split the Triangles
        std::vector<KdTriangle> left_triangles;
        std::vector<KdTriangle> right_triangles;
        for (const KdTriangle& triangle : _kd_triangles) {
            if (triangle.isBefore(m_split_axis, split_pos)) { // TODO: push directly sorted
                left_triangles.push_back(triangle);
            }
            if (triangle.isAfter(m_split_axis, split_pos)) {
                right_triangles.push_back(triangle);
            }
        }

        // If triangles are unsplittable don't split the node
        if (_kd_triangles.size() == left_triangles.size() || _kd_triangles.size() == right_triangles.size()) {
            m_leaf = true;
            m_triangles = _kd_triangles;
            return;
        }

        // Generate the bounding boxes
        glm::vec3 left_max = glm::vec3(m_aabb.max);
        left_max[m_split_axis] = split_pos;
        glm::vec3 right_min = glm::vec3(m_aabb.min);
        right_min[m_split_axis] = split_pos;
        AABB<float> left_aabb = AABB<float>(m_aabb.min, left_max);
        AABB<float> right_aabb = AABB<float>(right_min, m_aabb.max);

        size_t new_split_axis = (m_split_axis + 1) % 3;
        if (!left_triangles.empty()) {
            m_left = std::make_unique<KdTree>(left_triangles, left_aabb, m_leaf_size, new_split_axis);
        }
        if (!right_triangles.empty()) {
            m_right = std::make_unique<KdTree>(right_triangles, right_aabb, m_leaf_size, new_split_axis);
        }
    }

    inline bool intersect(const glm::vec3& _origin, const glm::vec3& _direction, float& t, size_t& triangle_index, glm::vec3& intersection, glm::vec3& barycentrics) const {
        float tmin = 0., tmax = 0.;
        if (!m_aabb.intersectRay(_origin, _direction, tmin, tmax)) {
            return false;
        }

        return m_leaf ? intersect_leaf(_origin, _direction, t, triangle_index, intersection, barycentrics)
                      : intersect_nonleaf(_origin, _direction, t, triangle_index, intersection, barycentrics);
    }

    inline bool intersect_leaf(const glm::vec3& _origin, const glm::vec3& _direction, float& t, size_t& triangle_index, glm::vec3& intersection, glm::vec3& barycentrics) const {
        t = FLT_MAX;
        float tmp_t;
        glm::vec3 tmp_inter, tmp_bary;
        for (const KdTriangle& kd_triangle : m_triangles) {
            if (rayTriangleIntersection<float>(_origin, _direction, kd_triangle.v0, kd_triangle.v1, kd_triangle.v2, kd_triangle.normal, tmp_t, tmp_inter, tmp_bary) && tmp_t < t) {
                t = tmp_t;
                intersection = tmp_inter;
                barycentrics = tmp_bary;
                triangle_index = kd_triangle.triangle_index;
            }
        }
        return t == FLT_MAX;
    }

    inline bool intersect_nonleaf(const glm::vec3& _origin, const glm::vec3& _direction, float& t, size_t& triangle_index, glm::vec3& intersection, glm::vec3& barycentrics) const {
        KdTree *first = nullptr, *second = nullptr;
        float t_first_min = FLT_MAX, t_first_max = FLT_MAX;
        float t_second_min = FLT_MAX, t_second_max = FLT_MAX;

        // Check intersection with child bounding boxes
        if (m_left && m_left->m_aabb.intersectRay(_origin, _direction, t_first_min, t_first_max)) {
            first = m_left.get();
        }
        if (m_right && m_right->m_aabb.intersectRay(_origin, _direction, t_second_min, t_second_max)) {
            second = m_right.get();
        }

        // Determine traversal order
        if (t_first_min > t_second_min) {
            std::swap(first, second);
        }

        // Traverse children in order
        return (first != nullptr && first->intersect(_origin, _direction, t, triangle_index, intersection, barycentrics)) ||
               (second != nullptr && second->intersect(_origin, _direction, t, triangle_index, intersection, barycentrics));
    }
};