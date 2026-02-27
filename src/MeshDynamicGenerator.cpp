#include "Mesh.hpp"
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <map>

struct Vec3Less {
    bool operator()(const glm::vec3 &a, const glm::vec3 &b) const {
        return a.x != b.x   ? a.x < b.x
               : a.y != b.y ? a.y < b.y
                            : a.z < b.z;
    }
};

void addEdgeIfNeeded(DynamicObject &object, std::unordered_set<uint64_t> &seen_edges, uint a, uint b) {
    const uint v0 = (a < b) ? a : b;
    const uint v1 = (a < b) ? b : a;
    const uint64_t key = (static_cast<uint64_t>(v0) << 32) | static_cast<uint64_t>(v1);

    if (seen_edges.find(key) == seen_edges.end()) {
        seen_edges.insert(key);
        object.addDistanceConstraint(v0, v1, 1.f);
    }
}

DynamicObject Mesh::intoRigidBody() const {
    DynamicObject object;

    std::map<glm::vec3, uint, Vec3Less> seen_positions;
    std::unordered_map<uint, uint> positions_map;
    uint pj = 0;
    for (uint i = 0; i < m_positions.size(); i++) {
        const glm::vec3 &pos = m_positions[i];
        if (seen_positions.find(pos) == seen_positions.end()) {
            object.addVertex(pos, glm::vec3(0.f), 1.f, false);
            seen_positions.insert(std::make_pair(pos, pj));
            positions_map.insert(std::make_pair(i, pj));
            pj++;
        } else {
            uint j = seen_positions.at(pos);
            positions_map.insert(std::make_pair(i, j));
        }
    }

    std::unordered_set<uint64_t> seen_edges;
    for (uint i = 0; i < m_triangles.size(); i++) {
        const uint a = positions_map.at(m_triangles[i][0]);
        const uint b = positions_map.at(m_triangles[i][1]);
        const uint c = positions_map.at(m_triangles[i][2]);

        addEdgeIfNeeded(object, seen_edges, a, b);
        addEdgeIfNeeded(object, seen_edges, a, c);
        addEdgeIfNeeded(object, seen_edges, b, c);
    }

    return object;
}
