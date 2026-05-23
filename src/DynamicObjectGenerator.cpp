#include "DynamicObject.hpp"
#include <iostream>
#include <set>
#include <map>
#include <map>

struct Vec3Less {
    bool operator()(const glm::vec3& a, const glm::vec3& b) const {
        return a.x != b.x   ? a.x < b.x
               : a.y != b.y ? a.y < b.y
                            : a.z < b.z;
    }
};

void DynamicObject::addObject(const DynamicObject& _object) {
    uint pj_offset = N;
    N += _object.N;
    M += _object.M;
    Mcoll += _object.Mcoll;

    for (uint pj = 0; pj < _object.N; pj++) {
        m_positions.push_back(_object.m_positions[pj]);
        m_velocities.push_back(_object.m_velocities[pj]);
        m_masses.push_back(_object.m_masses[pj]);
        m_weights.push_back(_object.m_weights[pj]);
        m_fixed.push_back(_object.m_fixed[pj]);
    }

    for (uint ci = 0; ci < _object.M + _object.Mcoll; ci++) {
        m_cardinalities.push_back(_object.m_cardinalities[ci]);
        m_functions.push_back(_object.m_functions[ci]);
        m_gradients.push_back(_object.m_gradients[ci]);
        std::vector<uint> indices{_object.m_indices[ci]};
        for (uint& pj : indices) {
            pj += pj_offset;
        }
        m_indices.push_back(indices);
        m_stiffnesses.push_back(_object.m_stiffnesses[ci]);
        m_types.push_back(_object.m_types[ci]);
        m_debug_types.push_back(_object.m_debug_types[ci]);
    }

    m_lines.reserve(m_lines.size() + _object.m_lines.size());
    for (const glm::uvec2& line : _object.m_lines) {
        m_lines.push_back(line + glm::uvec2(pj_offset));
    }

    m_triangles.reserve(m_triangles.size() + _object.m_triangles.size());
    for (const glm::uvec3& triangle : _object.m_triangles) {
        m_triangles.push_back(triangle + glm::uvec3(pj_offset));
    }
}

DynamicObject DynamicObject::bodyFromMesh(const StaticBody& _static_body, float _distance_stiffness, float _angle_stiffness, float _volume_stiffness, float _volume_pressure, float _vertex_mass) {
    const std::vector<glm::uvec3>& mesh_triangles = _static_body.m_mesh->triangleIndices();
    const glm::mat4 transformation = _static_body.m_transformation->computeTransformationMatrix();

    DynamicObject object;

    std::map<glm::vec3, uint, Vec3Less> seen_positions;
    std::map<uint, uint> positions_map;
    uint pj = 0;

    const std::vector<glm::vec3>& mesh_positions = _static_body.m_mesh->vertexPositions();
    for (uint i = 0; i < mesh_positions.size(); i++) {
        const glm::vec3& pos = mesh_positions[i];
        if (seen_positions.find(pos) == seen_positions.end()) {
            object.addVertex(applyTransformation(pos, 1.f, transformation), glm::vec3(0.f), _vertex_mass, false);
            seen_positions.insert(std::make_pair(pos, pj));
            positions_map.insert(std::make_pair(i, pj));
            pj++;
        } else {
            uint j = seen_positions.at(pos);
            positions_map.insert(std::make_pair(i, j));
        }
    }

    // DISTANCES CONSTRAINTS

    std::set<uint64_t> seen_edges;
    const auto addEdgeIfNeeded = [&object, &seen_edges, _distance_stiffness](uint a, uint b) {
        const uint v0 = (a < b) ? a : b;
        const uint v1 = (a < b) ? b : a;
        const uint64_t key = (static_cast<uint64_t>(v0) << 32) | static_cast<uint64_t>(v1);

        if (seen_edges.find(key) == seen_edges.end()) {
            seen_edges.insert(key);
            object.addDistanceConstraint(v0, v1, _distance_stiffness);
            object.m_lines.push_back(glm::vec2(v0, v1));
        }
    };

    for (uint i = 0; i < mesh_triangles.size(); i++) {
        const uint a = positions_map.at(mesh_triangles[i][0]);
        const uint b = positions_map.at(mesh_triangles[i][1]);
        const uint c = positions_map.at(mesh_triangles[i][2]);
        object.m_triangles.push_back(glm::uvec3(a, b, c));

        if (_distance_stiffness > 0.) {
            addEdgeIfNeeded(a, b);
            addEdgeIfNeeded(a, c);
            addEdgeIfNeeded(b, c);
        }
    }

    // BENDING CONSTRAINTS
    if (_angle_stiffness > 0.) {
        std::map<std::pair<uint, uint>, std::vector<uint>> edgeToOpposite;
        for (uint i = 0; i < mesh_triangles.size(); i++) {
            uint a = positions_map.at(mesh_triangles[i][0]);
            uint b = positions_map.at(mesh_triangles[i][1]);
            uint c = positions_map.at(mesh_triangles[i][2]);

            edgeToOpposite[{std::min(a, b), std::max(a, b)}].push_back(c);
            edgeToOpposite[{std::min(a, c), std::max(a, c)}].push_back(b);
            edgeToOpposite[{std::min(b, c), std::max(b, c)}].push_back(a);
        }

        for (auto const& pair : edgeToOpposite) {
            const auto& edge = pair.first;
            const auto& opposites = pair.second;
            if (opposites.size() == 2) {
                uint p0 = edge.first;
                uint p1 = edge.second;
                uint p2 = opposites[0];
                uint p3 = opposites[1];

                object.addBendingConstraint(p0, p1, p2, p3, _angle_stiffness);
            }
        }
    }

    // VOLUME CONSTRAINT
    if (_volume_stiffness > 0.f) {
        std::vector<glm::uvec3> remapped_triangles;
        remapped_triangles.reserve(mesh_triangles.size());

        for (const auto& tri : mesh_triangles) {
            remapped_triangles.push_back(glm::uvec3(
                positions_map.at(tri[0]),
                positions_map.at(tri[1]),
                positions_map.at(tri[2])));
        }

        object.addVolumeConstraint(remapped_triangles, _volume_stiffness, _volume_pressure);
    }

    return object;
}