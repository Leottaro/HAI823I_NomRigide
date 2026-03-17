#include "DynamicObject.hpp"
#include <glm/matrix.hpp>
#include <iostream>

double length2(const glm::dvec3 &vec) {
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
void DynamicObject::dampVelocities(double k_damping) {
    bool is_fixed = false;
    for (uint i = 0; i < N; i++) {
        if (m_fixed[i]) {
            is_fixed = true;
            break;
        }
    }

    if (is_fixed) {
        float air_friction = 0.001f;
        for (uint i = 0; i < N; i++) {
            if (!m_fixed[i]) {
                m_velocities[i] *= (1.0f - air_friction);
            }
        }
        return;
    }

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
    if (fabsf64(glm::determinant(I)) < 1e-8) {
        return;
    }
    glm::dvec3 omega = glm::inverse(I) * L; // (5): angular velocity

    // (6)-(9)
    for (uint i = 0; i < N; i++) {
        if (m_fixed[i])
            continue;
        glm::dvec3 ri = m_positions[i] - xcm;
        glm::dvec3 dvi = vcm + glm::cross(omega, ri) - m_velocities[i]; // (7)
        m_velocities[i] += k_damping * dvi;                             // (8)
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

// TODO: imgui solver settings
#define SOLVER_CONVERGENCE_WINDOW 50
#define MAXIMUM_RESIDUAL_THRESHOLD 1.e-6
#define RESIDUAL_RANGE_THRESHOLD 1.e-6
#define MAXIMUM_DELTA_THRESHOLD 1.e-8

void DynamicObject::update(double _delta_time, const std::vector<StaticBody> &static_bodies) {
    std::vector<glm::dvec3> new_positions(N); // p_i

    // (5) external forces (gravity, etc...) (for now, just gravity)
    for (uint pj = 0; pj < N; pj++)
        m_velocities[pj] = m_fixed[pj] ? m_velocities[pj] : m_velocities[pj] + _delta_time * glm::dvec3(0., -9.807, 0.);

    // (6)
    dampVelocities(1.);

    // (7)
    for (uint pj = 0; pj < N; pj++)
        new_positions[pj] = m_fixed[pj] ? m_positions[pj] : m_positions[pj] + _delta_time * m_velocities[pj];

    // TODO: (8) Generate collision constraints with static_bodies

    // (9)-(11)
    std::vector<double> residuals_buffer(SOLVER_CONVERGENCE_WINDOW, DBL_MAX);
    std::vector<double> maxdelta_buffer(SOLVER_CONVERGENCE_WINDOW, -DBL_MAX);

    double rmin, rmax, dmax;
    uint nbloop = 0;
    do {
        residuals_buffer[nbloop % SOLVER_CONVERGENCE_WINDOW] = 0.;
        maxdelta_buffer[nbloop % SOLVER_CONVERGENCE_WINDOW] = -DBL_MAX;

        for (uint ci = 0; ci < M; ci++) {
            // gather function input (and total weight)
            std::vector<glm::dvec3> affected_points(m_cardinalities[ci]);
            double total_weigths = 0.;
            for (uint i = 0; i < m_cardinalities[ci]; i++) {
                uint pj = m_indices[ci][i];
                affected_points[i] = new_positions[pj];
                total_weigths += m_weights[pj];
            }

            double function_value = m_functions[ci](affected_points);
            if (m_types[ci] == INEQUALITY_CONSTRAINT && function_value >= 0.) {
                // The constraint is already satisfied so we don't project it
                continue;
            }
            residuals_buffer[nbloop % SOLVER_CONVERGENCE_WINDOW] += function_value * function_value;

            // Determine S
            std::vector<glm::dvec3> gradients = m_gradients[ci](affected_points);
            double denominator = 0.;
            for (uint i = 0; i < m_cardinalities[ci]; i++) {
                denominator += length2(gradients[i]);
            }
            double s = function_value / denominator;

            // add the deltas
            for (uint i = 0; i < m_cardinalities[ci]; i++) {
                uint pj = m_indices[ci][i];
                glm::dvec3 delta_pj = -s * (double(m_cardinalities[ci]) * m_weights[pj] / total_weigths) * gradients[i];
                new_positions[pj] += delta_pj;
                maxdelta_buffer[nbloop % SOLVER_CONVERGENCE_WINDOW] = std::max(maxdelta_buffer[nbloop % SOLVER_CONVERGENCE_WINDOW], glm::length(delta_pj));
            }
        }

        residuals_buffer[nbloop % SOLVER_CONVERGENCE_WINDOW] = sqrt(residuals_buffer[nbloop % SOLVER_CONVERGENCE_WINDOW]);
        auto rpair = std::minmax_element(residuals_buffer.begin(), residuals_buffer.end());
        rmin = *rpair.first;
        rmax = *rpair.second;

        dmax = *std::max_element(maxdelta_buffer.begin(), maxdelta_buffer.end());

        nbloop++;
    } while (rmax > MAXIMUM_RESIDUAL_THRESHOLD || rmax - rmin > RESIDUAL_RANGE_THRESHOLD || dmax > MAXIMUM_DELTA_THRESHOLD);

    // (12)-(15)
    for (uint pj = 0; pj < N; pj++) {
        m_velocities[pj] = (new_positions[pj] - m_positions[pj]) / _delta_time; // (13)
        m_positions[pj] = new_positions[pj];                                    // (14)
    }

    // TODO: (16) Velocity update
}

void DynamicObject::addVertex(const glm::dvec3 &_position, const glm::dvec3 &_velocity, double _mass, bool _fixed) {
    N++;
    m_positions.push_back(_position);
    m_velocities.push_back(_velocity);
    m_masses.push_back(_mass);
    m_weights.push_back(_fixed ? 0. : 1. / _mass);
    m_fixed.push_back(_fixed);
}

void DynamicObject::setVertexFixed(uint _pj, bool _fixed) {
    m_fixed[_pj] = _fixed;
    m_weights[_pj] = _fixed ? 0. : 1. / m_masses[_pj];
}

void DynamicObject::addConstraint(
    uint _cardinality,
    const constraint_function &_function,
    const gradient_function &_gradient,
    const std::vector<uint> &_indices,
    double _stiffness,
    const ConstraintType &_type) {
    M++;
    m_cardinalities.push_back(_cardinality);
    m_functions.push_back(_function);
    m_gradients.push_back(_gradient);
    m_indices.push_back(_indices);
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(_type);
}

void DynamicObject::addDistanceConstraint(uint _p0, uint _p1, double _stiffness, double _targeted_distance) {
    M++;
    m_cardinalities.push_back(2);
    m_indices.push_back({_p0, _p1});
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(EQUALITY_CONSTRAINT);

    m_functions.push_back([_targeted_distance](const std::vector<glm::dvec3> &_p) {
        return glm::distance(_p[0], _p[1]) - _targeted_distance;
    });
    m_gradients.push_back([_targeted_distance](const std::vector<glm::dvec3> &_p) {
        glm::dvec3 n = glm::normalize(_p[0] - _p[1]);
        return std::vector<glm::dvec3>{n, -n};
    });
}
void DynamicObject::addDistanceConstraint(uint _p0, uint _p1, double _stiffness) {
    addDistanceConstraint(_p0, _p1, _stiffness, glm::distance(m_positions[_p0], m_positions[_p1]));
}

void DynamicObject::addBendingConstraint(uint _p0, uint _p1, uint _p2, uint _p3, double _stiffness, double _targeted_angle) {
    M++;
    m_cardinalities.push_back(4);
    m_indices.push_back({_p0, _p1, _p2, _p3});
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(EQUALITY_CONSTRAINT);

    m_functions.push_back([_targeted_angle](const std::vector<glm::dvec3> &_p) {
        glm::dvec3 e = glm::normalize(_p[1] - _p[0]); // arête commune

        glm::dvec3 n1 = glm::normalize(glm::cross(_p[2] - _p[0], _p[2] - _p[1]));
        glm::dvec3 n2 = glm::normalize(glm::cross(_p[3] - _p[1], _p[3] - _p[0]));
        double cosTheta = glm::clamp(glm::dot(n1, n2), -1., 1.);
        double theta = acos(cosTheta);
        double sign = glm::dot(glm::cross(n1, n2), e);
        if (sign < 0)
            theta = -theta;
        return theta - _targeted_angle;
    });
    m_gradients.push_back([_targeted_angle](const std::vector<glm::dvec3> &_p) {
        // bridson model
        // p0 and p1 : common edge
        glm::dvec3 e = _p[1] - _p[0];
        double elen = glm::length(e);

        glm::dvec3 n1 = glm::cross(_p[2] - _p[0], _p[2] - _p[1]);
        glm::dvec3 n2 = glm::cross(_p[3] - _p[1], _p[3] - _p[0]);
        double n1sq = length2(n1);
        double n2sq = length2(n2);

        // gp2 = u1, gp3 = u2, gp0 = u3, gp1 = u4
        glm::dvec3 gp2 = elen * (n1 / n1sq);
        glm::dvec3 gp3 = elen * (n2 / n2sq);
        glm::dvec3 gp0 = glm::dot(_p[2] - _p[1], e) / elen * (n1 / n1sq) + glm::dot(_p[3] - _p[1], e) / elen * (n2 / n2sq);
        glm::dvec3 gp1 = -glm::dot(_p[2] - _p[0], e) / elen * (n1 / n1sq) - glm::dot(_p[3] - _p[0], e) / elen * (n2 / n2sq);

        // glm::dvec3 sum = gp0 + gp1 + gp2 + gp3;
        // if (glm::length(sum) > 1.e-4) {
        //     std::cout << "ERREUR : sum != 0" << std::endl;
        // }

        return std::vector<glm::dvec3>{-gp0, -gp1, -gp2, -gp3};
    });
}

void DynamicObject::addBendingConstraint(uint _p0, uint _p1, uint _p2, uint _p3, double _stiffness) {
    glm::dvec3 e = glm::normalize(m_positions[_p1] - m_positions[_p0]);
    glm::dvec3 n1 = glm::normalize(glm::cross(m_positions[_p2] - m_positions[_p0], m_positions[_p2] - m_positions[_p1]));
    glm::dvec3 n2 = glm::normalize(glm::cross(m_positions[_p3] - m_positions[_p1], m_positions[_p3] - m_positions[_p0]));
    double cosTheta = glm::clamp(glm::dot(n1, n2), -1., 1.);
    double theta = acos(cosTheta);
    double sign = glm::dot(glm::cross(n1, n2), e);
    if (sign < 0)
        theta = -theta;
    addBendingConstraint(_p0, _p1, _p2, _p3, _stiffness, theta);
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
    updateRenderedConstraints();

    glBindVertexArray(0);
}

void DynamicObject::updateRenderedPositions() {
    std::vector<glm::vec3> positions_float(m_positions.begin(), m_positions.end());
    glBindBuffer(GL_ARRAY_BUFFER, m_positions_VBO);
    glBufferData(GL_ARRAY_BUFFER, positions_float.size() * sizeof(glm::vec3), positions_float.data(), GL_DYNAMIC_DRAW);
}

void DynamicObject::updateRenderedConstraints() {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_lines_EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_lines.size() * sizeof(glm::uvec2), m_lines.data(), GL_STATIC_DRAW);
}

void DynamicObject::render() {
    glBindVertexArray(m_VAO); // Activate the VAO storing geometry data
    if (m_lines.empty()) {
        glDrawArrays(GL_POINTS, 0, m_positions.size());
    } else {
        glDrawElements(GL_LINES, m_lines.size() * 2, GL_UNSIGNED_INT, 0);
    }
}

void DynamicObject::clear() {
    N = 0;
    m_positions.clear();
    m_velocities.clear();
    m_masses.clear();
    m_weights.clear();
    m_fixed.clear();

    M = 0;
    m_cardinalities.clear();
    m_functions.clear();
    m_gradients.clear();
    m_indices.clear();
    m_stiffnesses.clear();
    m_types.clear();
    m_lines.clear();

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
}
