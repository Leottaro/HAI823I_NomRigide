#pragma once

#include "AABB.hpp"
#include <set>

// template <typename T>
// class PositionHasher {
//     using vec3 = glm::vec<3, T, glm::packed_highp>;

//     static constexpr size_t P1{73856093};
//     static constexpr size_t P2{19349663};
//     static constexpr size_t P3{83492791};

// public:
//     size_t m_hashmod;
//     T m_grid_size;
//     std::vector<std::set<size_t>> m_table;

//     PositionHasher(size_t _nb_vertices, T _grid_size) : m_grid_size(_grid_size) {
//         size_t table_size = 1;
//         while (table_size < _nb_vertices + _nb_vertices / 4)
//             table_size *= 2;
//         m_table = std::vector<std::set<size_t>>(table_size, std::set<size_t>());
//         m_hashmod = table_size - 1;
//     }

//     ~PositionHasher() = default;

//     inline glm::u64vec3 getGridCell(const vec3& _vertex) const { return glm::floor(_vertex / m_grid_size); }

//     inline void insertIndex(const vec3& _vertex, size_t _vertex_i) {
//         size_t bucket = hashVertex(_vertex);
//         m_table.at(bucket).insert(_vertex_i);
//     }

//     inline void insertLine(const vec3& _start, const vec3& _end, size_t _vertex_i) {
//         glm::u64vec3 start_grid_cell = getGridCell(_start);
//         glm::u64vec3 end_grid_cell = getGridCell(_end);
//         glm::u64vec3 key;
//         for (key.z = start_grid_cell.z; key.z <= end_grid_cell.z; key.z++)
//             for (key.y = start_grid_cell.y; key.y <= end_grid_cell.y; key.y++)
//                 for (key.x = start_grid_cell.x; key.x <= end_grid_cell.x; key.x++)
//                     m_table.at(hashKey(key)).insert(_vertex_i);
//     }

//     inline const std::set<size_t>& lookupVertex(const vec3& _vertex) const {
//         size_t bucket = hashVertex(_vertex);
//         return m_table.at(bucket);
//     };

//     inline void forAllGridCells(const AABB<T>& _aabb, const std::function<void(const std::set<size_t>&)>& _f) const {
//         glm::u64vec3 min_grid_cell = getGridCell(_aabb.min);
//         glm::u64vec3 max_grid_cell = getGridCell(_aabb.max);
//         glm::u64vec3 key;
//         for (key.z = min_grid_cell.z; key.z <= max_grid_cell.z; key.z++)
//             for (key.y = min_grid_cell.y; key.y <= max_grid_cell.y; key.y++)
//                 for (key.x = min_grid_cell.x; key.x <= max_grid_cell.x; key.x++)
//                     _f(lookupKey(key));
//     }

// private:
//     inline size_t hashVertex(const vec3& _vertex) const {
//         glm::u64vec3 key = getGridCell(_vertex);
//         return hashKey(key);
//     };
//     inline size_t hashKey(const glm::u64vec3& key) const {
//         return ((key.x * P1) ^ (key.y * P2) ^ (key.z * P3)) & m_hashmod;
//     };
//     inline const std::set<size_t>& lookupKey(const glm::u64vec3& _key) const {
//         size_t bucket = hashKey(_key);
//         return m_table.at(bucket);
//     };
// };

template <typename T>
class PositionHasher {
    using vec3 = glm::vec<3, T, glm::defaultp>;

    static constexpr size_t P1{73856093};
    static constexpr size_t P2{19349663};
    static constexpr size_t P3{83492791};

public:
    size_t m_hashmod;
    T m_grid_size;
    std::vector<std::set<size_t>> m_table;

    PositionHasher(size_t _nb_vertices, T _grid_size) : m_grid_size(_grid_size) {
        size_t table_size = 1;
        while (table_size < _nb_vertices + _nb_vertices / 4)
            table_size *= 2;
        m_table = std::vector<std::set<size_t>>(table_size, std::set<size_t>());
        m_hashmod = table_size - 1;
    }

    inline glm::u64vec3 getGridCell(const vec3& _vertex) const { return glm::u64vec3(std::numeric_limits<u_int64_t>::max() / 2) + glm::u64vec3(glm::floor(_vertex / m_grid_size)); }

    inline void insertIndexVertex(const vec3& _vertex, size_t _vertex_i) {
        size_t bucket = hashVertex(_vertex);
        m_table.at(bucket).insert(_vertex_i);
    }
    inline void insertIndexKey(const glm::u64vec3& _key, size_t _vertex_i) {
        size_t bucket = hashKey(_key);
        m_table.at(bucket).insert(_vertex_i);
    }
    inline void insertRange(const vec3& _start, const vec3& _end, size_t _vertex_i) {
        forAllGridCells(_start, _end, [&](const glm::u64vec3& key) { insertIndexKey(key, _vertex_i); });
    }
    inline void insertRange(const AABB<T>& _aabb, size_t _vertex_i) {
        forAllGridCells(_aabb, [&](const glm::u64vec3& key) { insertIndexKey(key, _vertex_i); });
    }

    inline const std::set<size_t>& lookupVertex(const vec3& _vertex) const {
        size_t bucket = hashVertex(_vertex);
        return m_table.at(bucket);
    };
    inline const std::set<size_t>& lookupKey(const glm::u64vec3& _key) const {
        size_t bucket = hashKey(_key);
        return m_table.at(bucket);
    };

    inline void forAllGridCells(const vec3& _min, const vec3& _max, const std::function<void(const glm::u64vec3&)>& _f) const {
        glm::u64vec3 min_grid_cell = getGridCell(_min);
        glm::u64vec3 max_grid_cell = getGridCell(_max);
        glm::u64vec3 key;
        for (key.z = min_grid_cell.z; key.z <= max_grid_cell.z; key.z++)
            for (key.y = min_grid_cell.y; key.y <= max_grid_cell.y; key.y++)
                for (key.x = min_grid_cell.x; key.x <= max_grid_cell.x; key.x++)
                    _f(key);
    }
    inline void forAllGridCells(const AABB<T>& _aabb, const std::function<void(const glm::u64vec3&)>& _f) const {
        forAllGridCells(_aabb.min, _aabb.max, _f);
    }

private:
    inline size_t hashVertex(const vec3& _vertex) const {
        glm::u64vec3 key = getGridCell(_vertex);
        return hashKey(key);
    };
    inline size_t hashKey(const glm::u64vec3& key) const {
        return ((key.x * P1) ^ (key.y * P2) ^ (key.z * P3)) & m_hashmod;
    };
};