#include "DynamicObject.hpp"
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

DynamicObject DynamicObject::bodyFromMesh(const StaticBody &_static_body, float _distance_stiffness, float _angle_stiffness) {
    const glm::mat4 tranformation = _static_body.m_transformation->computeTransformationMatrix();
    DynamicObject object;

    std::map<glm::vec3, uint, Vec3Less> seen_positions;
    std::unordered_map<uint, uint> positions_map;
    uint pj = 0;

    const std::vector<glm::vec3> &mesh_positions = _static_body.m_mesh->vertexPositions();
    for (uint i = 0; i < mesh_positions.size(); i++) {
        const glm::vec3 &pos = mesh_positions[i];
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
    const auto addEdgeIfNeeded = [&object, &seen_edges, _distance_stiffness](uint a, uint b) {
        const uint v0 = (a < b) ? a : b;
        const uint v1 = (a < b) ? b : a;
        const uint64_t key = (static_cast<uint64_t>(v0) << 32) | static_cast<uint64_t>(v1);

        if (seen_edges.find(key) == seen_edges.end()) {
            seen_edges.insert(key);
            object.addDistanceConstraint(v0, v1, _distance_stiffness);
            object.addDrawLine(v0, v1);
        }
    };

    const std::vector<glm::uvec3> &mesh_triangles = _static_body.m_mesh->triangleIndices();
    for (uint i = 0; i < mesh_triangles.size(); i++) {
        const uint a = positions_map.at(mesh_triangles[i][0]);
        const uint b = positions_map.at(mesh_triangles[i][1]);
        const uint c = positions_map.at(mesh_triangles[i][2]);

        addEdgeIfNeeded(a, b);
        addEdgeIfNeeded(a, c);
        addEdgeIfNeeded(b, c);
    }

    // BENDING CONSTRAINTS

    std::map<std::pair<uint, uint>, std::vector<uint>> edgeToOpposite;
    for (uint i = 0; i < mesh_triangles.size(); i++) {
        uint a = positions_map.at(mesh_triangles[i][0]);
        uint b = positions_map.at(mesh_triangles[i][1]);
        uint c = positions_map.at(mesh_triangles[i][2]);

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

            object.addBendingConstraint(p0, p1, p2, p3, _angle_stiffness);
            object.addDrawLine(p0, p1);
            object.addDrawLine(p1, p2);
            object.addDrawLine(p0, p2);
            object.addDrawLine(p0, p3);
            object.addDrawLine(p1, p3);
        }
    }

    // VOLUME CONSTRAINT
    std::vector<glm::uvec3> remapped_triangles;
    remapped_triangles.reserve(mesh_triangles.size());

    for (const auto& tri : mesh_triangles) {
        remapped_triangles.push_back(glm::uvec3(
            positions_map.at(tri[0]),
            positions_map.at(tri[1]),
            positions_map.at(tri[2])
        ));
    }

    object.addVolumeConstraint(remapped_triangles, 1., 1.);

    return object;
}
