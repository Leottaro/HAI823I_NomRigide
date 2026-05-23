#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <glm/matrix.hpp>

#include "DynamicObject.hpp"
#include <iostream>
#include <map>

/*
READ "3.5. Damping" of ./articles/Position_Based_Dynamics.pdf
(1) xcm = (∑i xi*mi )/( ∑i mi )
(2) vcm = (∑i vi*mi )/( ∑i mi )
(3) L = ∑i ri × (mi*vi)
(4) I = ∑i r̃i r̃Ti mi
(5) ω = I−1L
(6) forall vertices i
(7) ∆vi = vcm + ω × ri − vi
(8) vi ← vi + kdamping ∆vi
(9) endfor
*/
void DynamicObject::dampVelocities(uint _start, uint _end) {
    double total_mass = 0.;
    glm::dvec3 xcm = glm::dvec3(0.); // (1) : global linear velocity
    glm::dvec3 vcm = glm::dvec3(0.); // (2)
    for (uint i = _start; i < _end; i++) {
        if (m_fixed[i])
            continue;
        xcm += m_positions[i] * m_masses[i];  // (1)
        vcm += m_velocities[i] * m_masses[i]; // (2)
        total_mass += m_masses[i];
    }

    if (total_mass == 0.) {
        return; // Nothing to damp
    }

    xcm /= total_mass; // (1)
    vcm /= total_mass; // (2)

    glm::dvec3 L = glm::dvec3(0.); // (3)
    glm::dmat3 I = glm::dmat3(0.); // (4)
    for (uint i = _start; i < _end; i++) {
        if (m_fixed[i])
            continue;
        glm::dvec3 ri = m_positions[i] - xcm;
        // (3)
        L += glm::cross(ri, m_masses[i] * m_velocities[i]);
        // (4)
        // glm::dmat3 r_tilde_i = glm::dmat3(0, -ri.z, ri.y, ri.z, 0, -ri.x, -ri.y, ri.x, 0);
        // I += r_tilde_i * glm::transpose(r_tilde_i) * m_masses[i];
        I += m_masses[i] * glm::dmat3(ri.y * ri.y + ri.z * ri.z, -ri.x * ri.y, -ri.x * ri.z, -ri.x * ri.y, ri.x * ri.x + ri.z * ri.z, -ri.y * ri.z, -ri.x * ri.z, -ri.y * ri.z, ri.x * ri.x + ri.y * ri.y);
    }

    // check invertibility
    if (std::abs(glm::determinant(I)) < 1e-8) {
        return;
    }
    glm::dvec3 omega = glm::inverse(I) * L; // (5): angular velocity

    // (6)-(9)
    for (uint i = _start; i < _end; i++) {
        if (m_fixed[i])
            continue;
        glm::dvec3 ri = m_positions[i] - xcm;
        glm::dvec3 dvi = vcm + glm::cross(omega, ri) - m_velocities[i]; // (7)
        m_velocities[i] += m_damping_coefficient * dvi;                 // (8)
    }
}

/*
READ "3.1. Algorithm Overview" of ./articles/Position_Based_Dynamics.pdf
 (1)  forall vertices i
 (2)      initialize xi = x0i , vi = v0i , wi = 1/mi
 (3)  endfor
 (4)  loop
 (5)      forall vertices i do vi ← vi + ∆twifext (xi)
 (6)      dampVelocities(v1 ,..., vN )
 (7)      forall vertices i do pi ← xi + ∆tvi
 (8)      forall vertices i do generateCollisionConstraints(xi → pi)
 (9)      loop solverIterations times
(10)          projectConstraints(C1 ,..., CM+Mcoll , p1,..., pN )
(11)      endloop
(12)      forall vertices i
(13)          v i ← (pi − xi )/∆t
(14)          x i ← pi
(15)      endfor
(16)      velocityUpdate(v1 ,..., vN )
(17)  endloop
*/
bool DynamicObject::projectConstraints(uint _solver_iterations, std::vector<glm::dvec3>& new_positions, std::map<uint, glm::dvec3>& _collisions_responses) {
    for (uint ci = 0; ci < M + Mcoll; ci++) {
        // gather function input (and total weight)
        std::vector<glm::dvec3> affected_points(m_cardinalities[ci]);
        double total_weigths = 0.;
        for (uint i = 0; i < m_cardinalities[ci]; i++) {
            uint pj = m_indices[ci][i];
            affected_points[i] = new_positions[pj];
            total_weigths += m_weights[pj];
        }
        if (total_weigths == 0.) {
            continue;
        }

        if (total_weigths != total_weigths) {
            std::cerr << total_weigths << " total weights... Is there a vertex with a zero mass ?" << std::endl;
            return false;
        }

        double function_value = m_functions[ci](affected_points);
        if (m_types[ci] == INEQUALITY_CONSTRAINT) {
            // The constraint is already satisfied so we don't project it
            if (function_value >= 0) {
                continue;
            }
            // accumulate collision normal
            else {
                bool is_collision_constraint = (m_debug_types[ci] == VERTEX_COLLISION_CONSTRAINT ||
                                                m_debug_types[ci] == EDGE_COLLISION_CONSTRAINT ||
                                                m_debug_types[ci] == TRAINGLE_COLLISION_CONSTRAINT);
                if (is_collision_constraint) {
                    std::vector<glm::dvec3> grads = m_gradients[ci](affected_points);
                    for (uint idx = 0; idx < m_cardinalities[ci]; idx++) {
                        uint global_pj = m_indices[ci][idx];
                        if (glm::length2(grads[idx]) > 1e-12) {
                            glm::dvec3 normal = glm::normalize(grads[idx]);
                            accumulateCollisionsResponse(global_pj, normal, _collisions_responses);
                        }
                    }
                }
            }
        }

        // Determine S
        std::vector<glm::dvec3> gradients = m_gradients[ci](affected_points);
        double denominator = 0.;
        for (uint i = 0; i < m_cardinalities[ci]; i++) {
            if (gradients[i] != gradients[i]) {
                std::cerr << glm::to_string(gradients[i]) << " gradients[i] for a " << CONSTAINT_DEBUG_NAMES[m_debug_types[ci]] << " constraint " << ci << " of cardinality " << m_cardinalities[ci] << "." << std::endl;
                return false;
            }
            denominator += glm::length2(gradients[i]);
        }
        if (denominator != denominator || denominator == 0.) {
            std::cerr << "invalid denominator=" << denominator << " for a " << CONSTAINT_DEBUG_NAMES[m_debug_types[ci]] << " constraint " << ci << " of cardinality " << m_cardinalities[ci] << ". You may need to lower the deltaTime!" << std::endl;
            return false;
        }
        double s = function_value / denominator;

        // add the deltas
        for (uint i = 0; i < m_cardinalities[ci]; i++) {
            uint pj = m_indices[ci][i];
            glm::dvec3 delta_pj = -s * (double(m_cardinalities[ci]) * m_weights[pj] / total_weigths) * gradients[i];
            if (delta_pj != delta_pj) {
                std::cerr << delta_pj << " delta_pj for a " << CONSTAINT_DEBUG_NAMES[m_debug_types[ci]] << " constraint " << ci << " of cardinality " << m_cardinalities[ci] << "." << std::endl;
                return false;
            }
            double k_prime = 1. - std::pow(1. - m_stiffnesses[ci], 1. / _solver_iterations);
            new_positions[pj] += k_prime * delta_pj;
            if (new_positions[pj] != new_positions[pj]) {
                std::cerr << new_positions[pj] << " new_positions[pj] for a " << CONSTAINT_DEBUG_NAMES[m_debug_types[ci]] << " constraint " << ci << " of cardinality " << m_cardinalities[ci] << "." << std::endl;
                return false;
            }
        }
    }
    return true;
}

bool DynamicObject::applyNewPositions(double _delta_time, const std::vector<glm::dvec3>& new_positions) {
    // (12)-(15)
    for (uint pj = 0; pj < N; pj++) {
        if (new_positions[pj] != new_positions[pj]) {
            std::cerr << m_positions[pj] << " m_positions[pj]." << std::endl;
            return false;
        }
        m_velocities[pj] = (new_positions[pj] - m_positions[pj]) / _delta_time; // (13)
        m_positions[pj] = new_positions[pj];                                    // (14)
    }
    return true;
}

bool DynamicObject::applyCollisions(const std::map<uint, glm::dvec3>& collisions_responses, uint _start, uint _end) {
    for (auto [pj, collision_normal] : collisions_responses) {
        if (pj < _start)
            continue;
        if (pj >= _end)
            break;

        collision_normal = glm::normalize(collision_normal);

        // Decompose velocity into normal and tangential components
        double v_dot_n = glm::dot(m_velocities[pj], collision_normal);
        glm::dvec3 v_normal = v_dot_n * collision_normal;
        glm::dvec3 v_tangent = m_velocities[pj] - v_normal;

        // Apply restitution (reflection in the direction of collision normal)
        // Negative sign because we reflect away from surface
        v_normal *= -m_restitution_coefficient;

        // Apply friction (dampen velocity perpendicular to collision normal)
        v_tangent *= (1. - m_friction_coefficient);

        // Reconstruct velocity
        m_velocities[pj] = v_normal + v_tangent;

        if (m_velocities[pj] != m_velocities[pj]) {
            std::cerr << m_velocities[pj] << " m_velocities[pj] after friction/restitution." << std::endl;
            return false;
        }
    }
    return true;
}

void DynamicObject::removeCollisionsConstraints() {
    // cancel collision constraitns
    Mcoll = 0;
    m_cardinalities.resize(M);
    m_indices.resize(M);
    m_stiffnesses.resize(M);
    m_types.resize(M);
    m_debug_types.resize(M);
    m_functions.resize(M);
    m_gradients.resize(M);
}

bool DynamicObject::update(const std::vector<StaticBody>& static_bodies, double _sub_delta_time, double _full_delta_time, uint _solver_iterations, bool _is_first_step) {
    std::vector<glm::dvec3> new_positions(N); // p_i
    std::vector<glm::dvec3> full_frame_velocities(N);
    std::vector<glm::dvec3> full_frame_positions(N);
    std::map<uint, glm::dvec3> collisions_responses;

    // (5) external forces (gravity, etc...) (for now, just gravity)
    for (uint pj = 0; pj < N; pj++) {
        full_frame_velocities[pj] = m_velocities[pj];
        m_velocities[pj] = m_fixed[pj] ? m_velocities[pj] : m_velocities[pj] + _sub_delta_time * glm::dvec3(0., -9.807, 0.);
    }

    // (6)
    dampVelocities(0, N);

    // (7)
    for (uint pj = 0; pj < N; pj++)
        new_positions[pj] = m_fixed[pj] ? m_positions[pj] : m_positions[pj] + _sub_delta_time * m_velocities[pj];

    // (8)
    if (_is_first_step) {
        removeCollisionsConstraints();

        // full frame velocity and position
        for (uint pj = 0; pj < N; pj++) {
            full_frame_velocities[pj] = m_fixed[pj] ? full_frame_velocities[pj] : full_frame_velocities[pj] + _full_delta_time * glm::dvec3(0., -9.807, 0.);
            full_frame_positions[pj] = m_positions[pj] + _full_delta_time * full_frame_velocities[pj];
        }

        double hash_grid_size = 0.;
        for (const glm::uvec2& edge : m_lines)
            hash_grid_size += glm::distance(m_positions[edge[0]], m_positions[edge[1]]) + glm::distance(new_positions[edge[0]], new_positions[edge[1]]);
        hash_grid_size /= m_lines.size(); // On fait pas *2 parce que je veux que la tailel sopit autour de 2 fois la moyenne des liens
        PositionHasher hasher(N, hash_grid_size);
        for (uint pj = 0; pj < N; pj++) {
            AABB<double> aabb;
            aabb.addPosition(m_positions[pj]);
            aabb.addPosition(new_positions[pj]);
            hasher.insertRange(aabb, pj);
        }
        detectPointTriangleCollision(full_frame_positions, static_bodies, collisions_responses, 0, N);
        detectEdgeEdgeCollision(full_frame_positions, static_bodies, collisions_responses, 0, m_lines.size());
        detectTrianglePointCollision(full_frame_positions, static_bodies, collisions_responses, 0, m_triangles.size());
        detectSelfPointTriangleCollision(hasher, full_frame_positions, collisions_responses, 0, m_triangles.size());
    }

    // (9)-(11)
    for (uint _ = 0; _ < _solver_iterations; _++)
        if (!projectConstraints(_solver_iterations, new_positions, collisions_responses))
            return false;

    if (!applyNewPositions(_sub_delta_time, new_positions)) // (12)-(15)
        return false;
    if (!applyCollisions(collisions_responses, 0, N)) // (16)
        return false;

    return true;
}

bool DynamicObject::update(std::vector<DynamicObject>& dynamic_objects, const std::vector<StaticBody>& static_bodies, double _sub_delta_time, double _full_delta_time, uint _solver_iterations, bool _is_first_step) {
    uint nb_objects = dynamic_objects.size();
    DynamicObject all_objects;
    std::vector<uint> vertices_offsets(nb_objects + 1, 0);
    std::vector<uint> lines_offsets(nb_objects + 1, 0);
    std::vector<uint> triangles_offsets(nb_objects + 1, 0);
    std::vector<uint> constraints_offsets(2 * (nb_objects + 1), 0); // pairs: M, impairs: Mcoll
    for (uint i = 0; i < nb_objects; i++) {
        all_objects.addObject(dynamic_objects[i]);
        vertices_offsets[i + 1] = all_objects.N;
        lines_offsets[i + 1] = all_objects.m_lines.size();
        triangles_offsets[i + 1] = all_objects.m_triangles.size();
        constraints_offsets[i * 2] = all_objects.M;
        constraints_offsets[i * 2 + 1] = all_objects.M + all_objects.Mcoll;
    }

    for (uint pj = 0; pj < all_objects.N; pj++)
        all_objects.m_velocities[pj] = all_objects.m_fixed[pj] ? all_objects.m_velocities[pj] : all_objects.m_velocities[pj] + _sub_delta_time * glm::dvec3(0., -9.807, 0.);
    for (uint i = 0; i < nb_objects; i++) {
        all_objects.m_damping_coefficient = dynamic_objects[i].m_damping_coefficient;
        all_objects.dampVelocities(vertices_offsets[i], vertices_offsets[i + 1]);
    }
    std::vector<glm::dvec3> new_positions(all_objects.N);
    std::vector<glm::dvec3> full_frame_velocities(all_objects.N);
    std::vector<glm::dvec3> full_frame_positions(all_objects.N);
    for (uint pj = 0; pj < all_objects.N; pj++) {
        full_frame_velocities[pj] = all_objects.m_velocities[pj];
        new_positions[pj] = all_objects.m_fixed[pj] ? all_objects.m_positions[pj] : all_objects.m_positions[pj] + _sub_delta_time * all_objects.m_velocities[pj];
    }

    std::map<uint, glm::dvec3> collisions_responses;
    if (_is_first_step) {
        all_objects.removeCollisionsConstraints();

        // full frame velocity and position
        for (uint pj = 0; pj < all_objects.N; pj++) {
            full_frame_velocities[pj] = all_objects.m_fixed[pj] ? full_frame_velocities[pj] : full_frame_velocities[pj] + _full_delta_time * glm::dvec3(0., -9.807, 0.);
            full_frame_positions[pj] = all_objects.m_positions[pj] + _full_delta_time * full_frame_velocities[pj];
        }

        double hash_grid_size = 0.;
        for (const glm::uvec2& edge : all_objects.m_lines)
            hash_grid_size += glm::distance(all_objects.m_positions[edge[0]], all_objects.m_positions[edge[1]]) + glm::distance(new_positions[edge[0]], new_positions[edge[1]]);
        hash_grid_size /= all_objects.m_lines.size(); // On fait pas / 2 parce que je veux que la tailel sopit autour de 2 fois la moyenne des liens
        PositionHasher hasher(all_objects.N, hash_grid_size);
        for (uint i = 0; i < nb_objects; i++) {
            for (uint pj = vertices_offsets[i]; pj < vertices_offsets[i + 1]; pj++) {
                AABB<double> aabb;
                aabb.addPosition(all_objects.m_positions[pj]);
                aabb.addPosition(new_positions[pj]);
                aabb.expand(dynamic_objects[i].m_surface_thickness);
                hasher.insertRange(aabb, pj);
            }
        }
        all_objects.detectPointTriangleCollision(new_positions, static_bodies, collisions_responses, 0, all_objects.N);
        for (uint i = 0; i < nb_objects; i++) {
            all_objects.m_surface_thickness = dynamic_objects[i].m_surface_thickness;
            all_objects.detectEdgeEdgeCollision(new_positions, static_bodies, collisions_responses, lines_offsets[i], lines_offsets[i + 1]);
            all_objects.detectTrianglePointCollision(new_positions, static_bodies, collisions_responses, triangles_offsets[i], triangles_offsets[i + 1]);
            all_objects.detectSelfPointTriangleCollision(hasher, new_positions, collisions_responses, triangles_offsets[i], triangles_offsets[i + 1]);
        }
    }

    for (uint _ = 0; _ < _solver_iterations; _++)
        all_objects.projectConstraints(_solver_iterations, new_positions, collisions_responses);

    all_objects.applyNewPositions(_sub_delta_time, new_positions);
    for (uint i = 0; i < nb_objects; i++) {
        all_objects.m_friction_coefficient = dynamic_objects[i].m_friction_coefficient;
        all_objects.m_restitution_coefficient = dynamic_objects[i].m_restitution_coefficient;
        all_objects.applyCollisions(collisions_responses, vertices_offsets[i], vertices_offsets[i + 1]);
    }

    for (uint i = 0; i < nb_objects; i++) {
        for (uint pj = 0; pj < dynamic_objects[i].N; pj++) {
            dynamic_objects[i].m_positions[pj] = all_objects.m_positions[vertices_offsets[i] + pj];
            dynamic_objects[i].m_velocities[pj] = all_objects.m_velocities[vertices_offsets[i] + pj];
        }
    }

    return true; // TODO:
}

void DynamicObject::addVertex(const glm::dvec3& _position, const glm::dvec3& _velocity, double _mass, bool _fixed) {
    N++;
    m_positions.push_back(_position);
    m_velocities.push_back(_velocity);
    m_masses.push_back(_mass);
    m_weights.push_back(_fixed ? 0. : 1. / _mass);
    m_fixed.push_back(_fixed);
}

// Object interaction

void DynamicObject::findNearestPointToLine(const glm::dvec3& _position, const glm::dvec3& _direction, uint& point, double& distance, glm::dvec3& projection) const {
    point = 0;
    distance = DBL_MAX;

    for (uint pj = 0; pj < N; pj++) {
        glm::dvec3 proj = projectPointOnLine(m_positions[pj], _position, _direction);
        double dist = glm::distance(m_positions[pj], proj);

        if (dist < distance) {
            point = pj;
            distance = dist;
            projection = proj;
        }
    }

    distance = distance;
}

bool DynamicObject::updateInteractions(GLFWwindow* _window, const glm::dvec3& _camera_pos, const glm::dvec3& _cursor_worldpos) {
    if (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE || glfwGetKey(_window, GLFW_KEY_LEFT_SHIFT) == GLFW_RELEASE) {
        if (grabbed_point != UINT32_MAX) {
            // std::cout << "finished grabbing point " << grabbed_point << "..." << std::endl;
            setVertexFixed(grabbed_point, grabbed_fixed);
            grabbed_point = UINT32_MAX;
            return true;
        }
        return false;
    }

    glm::dvec3 cursor_direction = glm::normalize(_cursor_worldpos - _camera_pos);
    if (grabbed_point != UINT32_MAX) {
        // std::cout << "grabbing point " << grabbed_point << "..." << std::endl;
        m_positions[grabbed_point] = projectPointOnLine(m_positions[grabbed_point], _camera_pos, cursor_direction);
        return true;
    }

    uint point;
    double distance;
    glm::dvec3 projection;
    findNearestPointToLine(_camera_pos, cursor_direction, point, distance, projection);
    if (distance < 0.3) {
        // std::cout << "grabbing point " << point << " with distance " << distance << std::endl;
        grabbed_point = point;
        m_positions[grabbed_point] = projection;
        grabbed_fixed = m_fixed[grabbed_point];
        setVertexFixed(grabbed_point, true);
        return true;
    }

    return false;
}

// OpenGL uinterface

void DynamicObject::computeNormals() {

    m_normals.clear();
    m_normals.resize(m_positions.size(), glm::dvec3(0.f));

    for (const auto& tri : m_triangles) {

        uint32_t i0 = tri.x;
        uint32_t i1 = tri.y;
        uint32_t i2 = tri.z;

        glm::dvec3 p0 = m_positions[i0];
        glm::dvec3 p1 = m_positions[i1];
        glm::dvec3 p2 = m_positions[i2];

        glm::dvec3 e1 = p1 - p0;
        glm::dvec3 e2 = p2 - p0;

        glm::dvec3 n = glm::normalize(glm::cross(e1, e2));

        m_normals[i0] += n;
        m_normals[i1] += n;
        m_normals[i2] += n;
    }

    for (glm::dvec3& n : m_normals) {
        n = glm::normalize(n);
    }
}

void DynamicObject::initRendering() {
    glGenVertexArrays(1, &m_VAO);
    glBindVertexArray(m_VAO);

    // 1. Configuration des Positions
    glGenBuffers(1, &m_positions_VBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_positions_VBO); // <-- Crucial !
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    // 2. Configuration des Normales
    glGenBuffers(1, &m_normals_VBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_normals_VBO); // <-- Crucial !
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

    // 3. Génération des EBOs (pense à les lier/remplir dans updateRenderedConstraints)
    glGenBuffers(1, &m_lines_EBO);
    glGenBuffers(1, &m_triangles_EBO);

    // On nettoie l'état d'OpenGL en déliant le VAO et le ARRAY_BUFFER
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    updateRenderedPositions();
    updateRenderedConstraints();
}

void DynamicObject::updateRenderedPositions() {
    glBindVertexArray(m_VAO);
    std::vector<glm::vec3> positions_float(m_positions.begin(), m_positions.end());
    glBindBuffer(GL_ARRAY_BUFFER, m_positions_VBO);
    glBufferData(GL_ARRAY_BUFFER, positions_float.size() * sizeof(glm::vec3), positions_float.data(), GL_DYNAMIC_DRAW);

    computeNormals();
    std::vector<glm::vec3> normals_float(m_normals.begin(), m_normals.end());
    glBindBuffer(GL_ARRAY_BUFFER, m_normals_VBO);

    glBufferData(GL_ARRAY_BUFFER, normals_float.size() * sizeof(glm::vec3), normals_float.data(), GL_DYNAMIC_DRAW);
}

void DynamicObject::updateRenderedConstraints() {
    glBindVertexArray(m_VAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_lines_EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_lines.size() * sizeof(glm::uvec2), m_lines.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangles_EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_triangles.size() * sizeof(glm::uvec3), m_triangles.data(), GL_STATIC_DRAW);
}

void DynamicObject::render(DynamicRenderType _type) const {
    glBindVertexArray(m_VAO); // Activate the VAO storing geometry data
    switch (_type) {
    case PointRender:
        glDrawArrays(GL_POINTS, 0, m_positions.size());
        break;
    case LineRender:
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_lines_EBO);
        glDrawElements(GL_LINES, m_lines.size() * 2, GL_UNSIGNED_INT, 0);
        break;
    case TriangleRender:
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangles_EBO);
        glDrawElements(GL_TRIANGLES, m_triangles.size() * 3, GL_UNSIGNED_INT, 0);
        break;
    case Auto:
        if (!m_triangles.empty()) {
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangles_EBO);
            glDrawElements(GL_TRIANGLES, m_triangles.size() * 3, GL_UNSIGNED_INT, 0);
        } else if (!m_lines.empty()) {
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_lines_EBO);
            glDrawElements(GL_LINES, m_lines.size() * 2, GL_UNSIGNED_INT, 0);
        } else {
            glDrawArrays(GL_POINTS, 0, m_positions.size());
        }
        break;
    }
}

void DynamicObject::clear() {
    N = 0;
    m_positions.clear();
    m_velocities.clear();
    m_masses.clear();
    m_weights.clear();
    m_fixed.clear();

    M = Mcoll = 0;
    m_cardinalities.clear();
    m_functions.clear();
    m_gradients.clear();
    m_indices.clear();
    m_stiffnesses.clear();
    m_types.clear();
    m_lines.clear();
    m_triangles.clear();

    if (m_VAO) {
        glDeleteVertexArrays(1, &m_VAO);
        m_VAO = 0;
    }
    if (m_positions_VBO) {
        glDeleteBuffers(1, &m_positions_VBO);
        m_positions_VBO = 0;
    }
    if (m_lines_EBO) {
        glDeleteBuffers(1, &m_lines_EBO);
        m_lines_EBO = 0;
    }
    if (m_triangles_EBO) {
        glDeleteBuffers(1, &m_triangles_EBO);
        m_triangles_EBO = 0;
    }
}