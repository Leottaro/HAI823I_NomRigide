#include "DynamicObject.hpp"
#include <glm/matrix.hpp>
#include <iostream>

float length2(const glm::vec3 &vec) {
    return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
}

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
void DynamicObject::dampVelocities(float k_damping) {
    float total_mass = 0.f;
    glm::vec3 xcm = glm::vec3(0.); // (1) : global linear velocity
    glm::vec3 vcm = glm::vec3(0.); // (2)
    for (uint i = 0; i < N; i++) {
        if (m_fixed[i])
            continue;
        xcm += m_positions[i] * m_masses[i];  // (1)
        vcm += m_velocities[i] * m_masses[i]; // (2)
        total_mass += m_masses[i];
    }

    if (total_mass == 0.f) {
        return; // Nothing to damp
    }

    xcm /= total_mass; // (1)
    vcm /= total_mass; // (2)

    glm::vec3 L = glm::vec3(0.); // (3)
    glm::mat3 I = glm::mat3(0.); // (4)
    for (uint i = 0; i < N; i++) {
        if (m_fixed[i])
            continue;
        glm::vec3 ri = m_positions[i] - xcm;
        // (3)
        L += glm::cross(ri, m_masses[i] * m_velocities[i]);
        // (4)
        // glm::mat3 r_tilde_i = glm::mat3(0, -ri.z, ri.y, ri.z, 0, -ri.x, -ri.y, ri.x, 0);
        // I += r_tilde_i * glm::transpose(r_tilde_i) * m_masses[i];
        I += m_masses[i] * glm::mat3(ri.y * ri.y + ri.z * ri.z, -ri.x * ri.y, -ri.x * ri.z, -ri.x * ri.y, ri.x * ri.x + ri.z * ri.z, -ri.y * ri.z, -ri.x * ri.z, -ri.y * ri.z, ri.x * ri.x + ri.y * ri.y);
    }

    // check invertibility
    if (fabs(glm::determinant(I)) < 1e-8f) {
        return;
    }
    glm::vec3 omega = glm::inverse(I) * L; // (5): angular velocity

    // (6)-(9)
    for (uint i = 0; i < N; i++) {
        if (m_fixed[i])
            continue;
        glm::vec3 ri = m_positions[i] - xcm;
        glm::vec3 dvi = vcm + glm::cross(omega, ri) - m_velocities[i]; // (7)
        m_velocities[i] += k_damping * dvi;                            // (8)
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
void DynamicObject::update(float _delta_time) {
    std::vector<glm::vec3> new_positions(N); // p_i

    // (5) external forces (gravity, etc...) (for now, just gravity)
    for (uint i = 0; i < N; i++)
        m_velocities[i] = m_fixed[i] ? m_velocities[i] : m_velocities[i] + _delta_time * glm::vec3(0., -9.807f, 0.f);

    // (6)
    dampVelocities(1.f);

    // (7)
    for (uint i = 0; i < N; i++)
        new_positions[i] = m_fixed[i] ? m_positions[i] : m_positions[i] + _delta_time * m_velocities[i];

    // TODO: (8) Generate collision constraints

    // (9)-(11)
    std::vector<glm::vec3> affected_points;
    std::vector<glm::vec3> gradients;
    float old_evolution, evolution, constraint_evolution;
    old_evolution = evolution = constraint_evolution = 0.f;
    do { // TODO: while pas convergé
        old_evolution = evolution;
        evolution = 0.f;
        for (uint ci = 0; ci < M; ci++) {
            constraint_evolution = 0.f;

            // gather function input (and total weight)
            affected_points.resize(m_cardinalities[ci]);
            float total_weigths = 0.f;
            for (uint i = 0; i < m_cardinalities[ci]; i++) {
                uint pj = m_indices[ci][i];
                affected_points[i] = new_positions[pj];
                total_weigths += m_weights[pj];
            }

            float function_value = m_functions[ci](affected_points);
            if (m_types[ci] == INEQUALITY_CONSTRAINT && function_value >= 0.f) {
                // The constraint is already satisfied so we don't project it
                continue;
            }

            // Determine S
            gradients.resize(m_cardinalities[ci]);
            float denominator = 0.f;
            for (uint i = 0; i < m_cardinalities[ci]; i++) {
                // uint pj = m_indices[ci][i];
                gradients[i] = m_gradients[ci](affected_points, i);
                denominator += length2(gradients[i]);
            }
            float s = function_value / denominator;

            // add the deltas
            for (uint i = 0; i < m_cardinalities[ci]; i++) {
                uint pj = m_indices[ci][i];
                glm::vec3 delta_pj = -s * (float(m_cardinalities[ci]) * m_weights[pj] / total_weigths) * gradients[i];
                new_positions[pj] += delta_pj; // TODO: stiffness factor
                constraint_evolution += glm::length(delta_pj);
            }

            evolution += constraint_evolution / m_cardinalities[ci];
        }
        evolution /= float(M);
    } while (abs(old_evolution - evolution) > 1e-7f);

    // (12)-(15)
    for (uint i = 0; i < N; i++) {
        m_velocities[i] = (new_positions[i] - m_positions[i]) / _delta_time; // (13)
        m_positions[i] = new_positions[i];                                   // (14)
    }

    // TODO: (16) Velocity update
    // std::cout << std::endl;
    // for (uint i = 0; i < N; i++) {
    //     std::cout << "v" << i << ":" << std::endl
    //               << "    (" << m_positions[i].x << "," << m_positions[i].y << "," << m_positions[i].z << ")" << std::endl
    //               << "    (" << m_velocities[i].x << "," << m_velocities[i].y << "," << m_velocities[i].z << ")" << std::endl
    //               << "    " << m_masses[i] << "\t" << m_weights[i] << std::endl;
    // }
}

void DynamicObject::addVertex(const glm::vec3 &_position, const glm::vec3 &_velocity, float _mass, bool _fixed) {
    N++;
    m_positions.push_back(_position);
    m_velocities.push_back(_velocity);
    m_masses.push_back(_mass);
    m_weights.push_back(_fixed ? 0.f : 1.f / _mass);
    m_fixed.push_back(_fixed);
}

void DynamicObject::setVertexFixed(uint _pj, bool _fixed) {
    m_fixed[_pj] = _fixed;
    m_weights[_pj] = _fixed ? 0.f : 1.f / m_masses[_pj];
}

void DynamicObject::addConstraint(
    uint _cardinality,
    const constraint_function &_function,
    const gradient_function &_gradient,
    const std::vector<uint> &_indices,
    float _stiffness,
    const ConstraintType &_type) {
    M++;
    m_cardinalities.push_back(_cardinality);
    m_functions.push_back(_function);
    m_gradients.push_back(_gradient);
    m_indices.push_back(_indices);
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(_type);
}

void DynamicObject::addDistanceConstraint(float _targeted_distance, const std::vector<uint> &_indices, float _stiffness, const ConstraintType &_type) {
    M++;
    m_cardinalities.push_back(2);
    m_functions.push_back([_targeted_distance](const std::vector<glm::vec3> &_p) {
        return glm::distance(_p[0], _p[1]) - _targeted_distance;
    });
    m_gradients.push_back([_targeted_distance](const std::vector<glm::vec3> &_p, uint _pj) {
        glm::vec3 n = (_p[0] - _p[1]) / glm::distance(_p[0], _p[1]);
        return _pj == 0 ? n : -n;
    });
    m_indices.push_back(_indices);
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(_type);
}

// OpenGL uinterface

void DynamicObject::init() {
    glGenVertexArrays(1, &m_VAO);
    glBindVertexArray(m_VAO);

    glGenBuffers(1, &m_positions_VBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_positions_VBO);
    glBufferData(GL_ARRAY_BUFFER, m_positions.size() * sizeof(glm::vec3), m_positions.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, m_positions_VBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindVertexArray(0);
}

void DynamicObject::render() {
    glBindVertexArray(m_VAO); // Activate the VAO storing geometry data
    // glDrawArrays(GL_TRIANGLE_STRIP, 0, m_positions.size());
    // glDrawArrays(GL_LINE_STRIP, 0, m_positions.size());
    glDrawArrays(GL_POINTS, 0, m_positions.size());
}

void DynamicObject::clear() {
    m_positions.clear();
    if (m_VAO) {
        glDeleteVertexArrays(1, &m_VAO);
        m_VAO = 0;
    }
    if (m_positions_VBO) {
        glDeleteBuffers(1, &m_positions_VBO);
        m_positions_VBO = 0;
    }
}
