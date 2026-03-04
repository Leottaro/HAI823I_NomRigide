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

DynamicObject Mesh::intoRigidBody(const Transformation &_t) const {
    const glm::mat4 tranformation = _t.computeTransformationMatrix();
    DynamicObject object;

    std::map<glm::vec3, uint, Vec3Less> seen_positions;
    std::unordered_map<uint, uint> positions_map;
    uint pj = 0;
    for (uint i = 0; i < m_positions.size(); i++) {
        const glm::vec3 &pos = m_positions[i];
        if (seen_positions.find(pos) == seen_positions.end()) {
            glm::vec4 transformed_pos = tranformation * glm::vec4(pos, 1.f);
            glm::vec3 added_pos = glm::vec3(transformed_pos) / transformed_pos.w;
            object.addVertex(added_pos, glm::vec3(0.f), 1.f, false);
            seen_positions.insert(std::make_pair(pos, pj));
            positions_map.insert(std::make_pair(i, pj));
            pj++;
        } else {
            uint j = seen_positions.at(pos);
            positions_map.insert(std::make_pair(i, j));
        }
    }

    // DISTANCES CONSTRAINTS

    std::unordered_set<uint64_t> seen_edges;
    const auto addEdgeIfNeeded = [&object, &seen_edges](uint a, uint b) {
        const uint v0 = (a < b) ? a : b;
        const uint v1 = (a < b) ? b : a;
        const uint64_t key = (static_cast<uint64_t>(v0) << 32) | static_cast<uint64_t>(v1);

        if (seen_edges.find(key) == seen_edges.end()) {
            seen_edges.insert(key);
            object.addDistanceConstraint(v0, v1, 1.f);
            // object.addDrawLine(v0, v1);
        }
    };

    for (uint i = 0; i < m_triangles.size(); i++) {
        const uint a = positions_map.at(m_triangles[i][0]);
        const uint b = positions_map.at(m_triangles[i][1]);
        const uint c = positions_map.at(m_triangles[i][2]);

        addEdgeIfNeeded(a, b);
        addEdgeIfNeeded(a, c);
        addEdgeIfNeeded(b, c);
    }

    // BENDING CONSTRAINTS

    std::map<std::pair<uint, uint>, std::vector<uint>> edgeToOpposite;
    for (uint i = 0; i < m_triangles.size(); i++) {
        uint a = positions_map.at(m_triangles[i][0]);
        uint b = positions_map.at(m_triangles[i][1]);
        uint c = positions_map.at(m_triangles[i][2]);

        edgeToOpposite[{std::min(a, b), std::max(a, b)}].push_back(c);
        edgeToOpposite[{std::min(a, c), std::max(a, c)}].push_back(b);
        edgeToOpposite[{std::min(b, c), std::max(b, c)}].push_back(a);
    }

    for (auto const &pair : edgeToOpposite) {
        const auto &edge = pair.first;
        const auto &opposites = pair.second;
        if (opposites.size() == 2) {
            uint p0 = edge.first;
            uint p1 = edge.second;
            uint p2 = opposites[0];
            uint p3 = opposites[1];

            object.addBendingConstraint(p0, p1, p2, p3, 1.f);
            object.addDrawLine(p0, p1);
            object.addDrawLine(p1, p2);
            object.addDrawLine(p0, p2);
            object.addDrawLine(p0, p3);
            object.addDrawLine(p1, p3);
        }
    }

    return object;
}
