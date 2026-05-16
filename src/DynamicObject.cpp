#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <glm/matrix.hpp>

#include "DynamicObject.hpp"
#include <iostream>
#include <unordered_map>

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
void DynamicObject::dampVelocities() {
    double total_mass = 0.;
    glm::dvec3 xcm = glm::dvec3(0.); // (1) : global linear velocity
    glm::dvec3 vcm = glm::dvec3(0.); // (2)
    for (uint i = 0; i < N; i++) {
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
    for (uint i = 0; i < N; i++) {
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
    for (uint i = 0; i < N; i++) {
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
void DynamicObject::generateNewPositions(double _delta_time, std::vector<glm::dvec3>& new_positions) {
    // (5) external forces (gravity, etc...) (for now, just gravity)
    for (uint pj = 0; pj < N; pj++)
        m_velocities[pj] = m_fixed[pj] ? m_velocities[pj] : m_velocities[pj] + _delta_time * glm::dvec3(0., -9.807, 0.);

    // (6)
    dampVelocities();

    // (7)
    for (uint pj = 0; pj < N; pj++)
        new_positions[pj] = m_fixed[pj] ? m_positions[pj] : m_positions[pj] + _delta_time * m_velocities[pj];
}

void DynamicObject::generateCollisions(double _delta_time, std::vector<glm::dvec3>& new_positions, const std::vector<StaticBody>& static_bodies, const std::vector<DynamicObject*> dynamic_objects, std::vector<std::unordered_map<uint, glm::dvec3>>& collisions_responses) {
    for (uint i = 0; i < dynamic_objects.size(); i++) {
        if (dynamic_objects[i] == this) {
            detectPointTriangleCollision(new_positions, static_bodies, collisions_responses[i]);
            detectEdgeEdgeCollision(new_positions, static_bodies, collisions_responses[i]);
            detectTrianglePointCollision(new_positions, static_bodies, collisions_responses[i]);
        }
    }
    detectSelfPointTriangleCollision(new_positions, dynamic_objects, collisions_responses);
}

bool DynamicObject::projectConstraints(uint _solver_iterations, std::vector<glm::dvec3>& new_positions) {
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
        if (m_types[ci] == INEQUALITY_CONSTRAINT && function_value >= 0.) {
            // The constraint is already satisfied so we don't project it
            continue;
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
        if (m_positions[pj] != new_positions[pj]) {
            std::cerr << m_positions[pj] << " m_positions[pj]." << std::endl;
            return false;
        }
        m_velocities[pj] = (new_positions[pj] - m_positions[pj]) / _delta_time; // (13)
        m_positions[pj] = new_positions[pj];                                    // (14)
    }
    return true;
}

bool DynamicObject::applyCollisions(std::unordered_map<uint, glm::dvec3> collisions_responses) {
    for (auto [pj, collision_normal] : collisions_responses) {
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

bool DynamicObject::update(double _delta_time, uint _solver_iterations, const std::vector<StaticBody>& static_bodies) {
    std::vector<glm::dvec3> new_positions(N);
    std::vector<std::unordered_map<uint, glm::dvec3>> collisions_responses;

    generateNewPositions(_delta_time, new_positions);                                            // (5)-(7)
    generateCollisions(_delta_time, new_positions, static_bodies, {this}, collisions_responses); // (8)

    // (9)-(11)
    for (uint _ = 0; _ < _solver_iterations; _++)
        if (!projectConstraints(_solver_iterations, new_positions))
            return false;

    if (!applyNewPositions(_delta_time, new_positions)) // (12)-(15)
        return false;
    if (!applyCollisions(collisions_responses[0])) // (16)
        return false;

    removeCollisionsConstraints();

    return true;
}

bool DynamicObject::update(const std::vector<DynamicObject*>& dynamic_objects, double _delta_time, uint _solver_iterations, const std::vector<StaticBody>& static_bodies) {
    uint nb_objects = dynamic_objects.size();

    std::vector<bool> stop_update(nb_objects);
    std::vector<std::vector<glm::dvec3>> new_positions(nb_objects);
    for (uint i = 0; i < nb_objects; i++)
        dynamic_objects[i]->generateNewPositions(_delta_time, new_positions[i]);

    std::vector<std::unordered_map<uint, glm::dvec3>> collisions_responses;
    for (uint i = 0; i < nb_objects; i++)
        dynamic_objects[i]->generateCollisions(_delta_time, new_positions[i], static_bodies, dynamic_objects, collisions_responses); // (8)

    for (uint _ = 0; _ < _solver_iterations; _++)
        for (uint i = 0; i < nb_objects; i++)
            stop_update[i] = stop_update[i] || !dynamic_objects[i]->projectConstraints(_solver_iterations, new_positions[i]);

    for (uint i = 0; i < nb_objects; i++) {
        stop_update[i] = stop_update[i] || !dynamic_objects[i]->applyNewPositions(_delta_time, new_positions[i]); // (12)-(15)
        stop_update[i] = stop_update[i] || !dynamic_objects[i]->applyCollisions(collisions_responses[i]);         // (16)
        dynamic_objects[i]->removeCollisionsConstraints();
    }
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

void DynamicObject::initRendering() {
    glGenVertexArrays(1, &m_VAO);
    glBindVertexArray(m_VAO);

    glGenBuffers(1, &m_positions_VBO);
    updateRenderedPositions();
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glGenBuffers(1, &m_lines_EBO);
    glGenBuffers(1, &m_triangles_EBO);
    updateRenderedConstraints();
}

void DynamicObject::updateRenderedPositions() {
    glBindVertexArray(m_VAO);
    std::vector<glm::vec3> positions_float(m_positions.begin(), m_positions.end());
    glBindBuffer(GL_ARRAY_BUFFER, m_positions_VBO);
    glBufferData(GL_ARRAY_BUFFER, positions_float.size() * sizeof(glm::vec3), positions_float.data(), GL_DYNAMIC_DRAW);
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