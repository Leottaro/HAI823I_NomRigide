#include "Mesh.hpp"
#include <unordered_set>
#include <iostream>

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

    for (uint i = 0; i < m_positions.size(); i++) {
        object.addVertex(m_positions[i], glm::vec3(0.f), 1.f, false);
    }

    std::unordered_set<uint64_t> seen_edges;
    for (uint i = 0; i < m_triangles.size(); i++) {
        const uint a = m_triangles[i][0];
        const uint b = m_triangles[i][1];
        const uint c = m_triangles[i][2];

        addEdgeIfNeeded(object, seen_edges, a, b);
        addEdgeIfNeeded(object, seen_edges, a, c);
        addEdgeIfNeeded(object, seen_edges, b, c);
    }

    return object;
}
