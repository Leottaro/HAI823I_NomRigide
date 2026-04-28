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
bool DynamicObject::update(double _delta_time, uint _solver_iterations, const std::vector<StaticBody> &static_bodies) {
    std::vector<glm::dvec3> new_positions(N); // p_i

    // (5) external forces (gravity, etc...) (for now, just gravity)
    for (uint pj = 0; pj < N; pj++)
        m_velocities[pj] = m_fixed[pj] ? m_velocities[pj] : m_velocities[pj] + _delta_time * glm::dvec3(0., -9.807, 0.);

    // (6)
    dampVelocities();

    // (7)
    for (uint pj = 0; pj < N; pj++)
        new_positions[pj] = m_fixed[pj] ? m_positions[pj] : m_positions[pj] + _delta_time * m_velocities[pj];

    // (8)
    std::unordered_map<uint, glm::dvec3> collisions_responses;
    detectPointTriangleCollision(new_positions, static_bodies, collisions_responses);
    detectEdgeEdgeCollision(new_positions, static_bodies, collisions_responses);
    detectTrianglePointCollision(new_positions, static_bodies, collisions_responses);
    detectSelfPointTriangleCollision(new_positions, collisions_responses);

    // (9)-(11)
    for (uint i = 0; i < _solver_iterations; i++) {
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
    }

    // (12)-(15)
    for (uint pj = 0; pj < N; pj++) {
        m_velocities[pj] = (new_positions[pj] - m_positions[pj]) / _delta_time; // (13)
        m_positions[pj] = new_positions[pj];                                    // (14)
        if (m_positions[pj] != m_positions[pj]) {
            std::cerr << m_positions[pj] << " m_positions[pj]." << std::endl;
            return false;
        }
    }

    // (16)
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

    // cancel collision constraitns
    Mcoll = 0;
    m_cardinalities.resize(M);
    m_indices.resize(M);
    m_stiffnesses.resize(M);
    m_types.resize(M);
    m_debug_types.resize(M);
    m_functions.resize(M);
    m_gradients.resize(M);

    return true;
}

inline glm::dmat3 getTildeMatrix(const glm::dvec3 &_p) {
    return glm::dmat3(0, -_p.z, _p.y, _p.z, 0, -_p.x, -_p.y, _p.x, 0);
}

void DynamicObject::addVertex(const glm::dvec3 &_position, const glm::dvec3 &_velocity, double _mass, bool _fixed) {
    N++;
    m_positions.push_back(_position);
    m_velocities.push_back(_velocity);
    m_masses.push_back(_mass);
    m_weights.push_back(_fixed ? 0. : 1. / _mass);
    m_fixed.push_back(_fixed);
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
    m_debug_types.push_back(CUSTOM_CONSTRAINT);
}

void DynamicObject::addDistanceConstraint(uint _p0, uint _p1, double _stiffness, double _targeted_distance) {
    M++;
    m_cardinalities.push_back(2);
    m_indices.push_back({_p0, _p1});
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(EQUALITY_CONSTRAINT);
    m_debug_types.push_back(DISTANCE_CONSTRAINT);

    m_functions.push_back([_targeted_distance](const std::vector<glm::dvec3> &_p) {
        return glm::distance(_p[0], _p[1]) - _targeted_distance;
    });
    m_gradients.push_back([](const std::vector<glm::dvec3> &_p) {
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
    m_debug_types.push_back(BENDING_CONSTRAINT);

    m_functions.push_back([_targeted_angle](const std::vector<glm::dvec3> &_p) {
        glm::dvec3 e = glm::normalize(_p[1] - _p[0]); // arête commune

        glm::dvec3 n1 = glm::normalize(glm::cross(_p[2] - _p[0], _p[2] - _p[1]));
        glm::dvec3 n2 = glm::normalize(glm::cross(_p[3] - _p[1], _p[3] - _p[0]));
        double cosTheta = glm::dot(n1, n2);
        double theta = acos(cosTheta);
        double sign = glm::dot(glm::cross(n1, n2), e);
        if (sign < 0)
            theta = -theta;
        return theta - _targeted_angle;
    });
    m_gradients.push_back([](const std::vector<glm::dvec3> &_p) {
        // bridson model
        // p0 and p1 : common edge
        glm::dvec3 e = _p[1] - _p[0];
        double elen = glm::length(e);

        glm::dvec3 n1 = glm::cross(_p[2] - _p[0], _p[2] - _p[1]);
        glm::dvec3 n2 = glm::cross(_p[3] - _p[1], _p[3] - _p[0]);
        double n1sq = glm::length2(n1);
        double n2sq = glm::length2(n2);

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

void DynamicObject::addCollisionConstraint(uint _p0, glm::dvec3 _intersection, glm::dvec3 _normal) {
    Mcoll++;
    m_cardinalities.push_back(1);
    m_indices.push_back({_p0});
    m_stiffnesses.push_back(1.);
    m_types.push_back(INEQUALITY_CONSTRAINT);
    m_debug_types.push_back(VERTEX_COLLISION_CONSTRAINT);

    m_functions.push_back([_intersection, _normal](const std::vector<glm::dvec3> &_p) {
        return glm::dot(_p[0] - _intersection, _normal);
    });
    m_gradients.push_back([_normal](const std::vector<glm::dvec3> &_p) {
        return std::vector<glm::dvec3>{_normal};
    });
}

void DynamicObject::addEdgeCollisionConstraint(uint _p0, uint _p1,
                                               double _t1, glm::dvec3 _point1, glm::dvec3 _normal1,
                                               double _t2, glm::dvec3 _point2, glm::dvec3 _normal2) {
    // Create first contact constraint
    Mcoll++;
    m_cardinalities.push_back(2);
    m_indices.push_back({_p0, _p1});
    m_stiffnesses.push_back(1.0);
    m_types.push_back(INEQUALITY_CONSTRAINT);
    m_debug_types.push_back(EDGE_COLLISION_CONSTRAINT);

    double thickness = m_surface_thickness;
    m_functions.push_back([thickness, _t1, _point1, _normal1](const std::vector<glm::dvec3> &_p) {
        glm::dvec3 edge_pt = (1.0 - _t1) * _p[0] + _t1 * _p[1];
        double dist = glm::dot(edge_pt - _point1, _normal1);
        return dist - thickness;
    });

    m_gradients.push_back([_t1, _normal1](const std::vector<glm::dvec3> &_p) {
        return std::vector<glm::dvec3>{
            (1.0 - _t1) * _normal1,
            _t1 * _normal1};
    });

    // Create second contact constraint
    Mcoll++;
    m_cardinalities.push_back(2);
    m_indices.push_back({_p0, _p1});
    m_stiffnesses.push_back(1.0);
    m_types.push_back(INEQUALITY_CONSTRAINT);
    m_debug_types.push_back(EDGE_COLLISION_CONSTRAINT);

    m_functions.push_back([thickness, _t2, _point2, _normal2](const std::vector<glm::dvec3> &_p) {
        glm::dvec3 edge_pt = (1.0 - _t2) * _p[0] + _t2 * _p[1];
        double dist = glm::dot(edge_pt - _point2, _normal2);
        return dist - thickness;
    });

    m_gradients.push_back([_t2, _normal2](const std::vector<glm::dvec3> &_p) {
        return std::vector<glm::dvec3>{
            (1.0 - _t2) * _normal2,
            _t2 * _normal2};
    });
}

void DynamicObject::addStaticPointDynamicTriangleConstraint(uint _p0, uint _p1, uint _p2, glm::dvec3 _static_point, glm::dvec3 _barycentrics, glm::dvec3 _normal) {
    Mcoll++;
    m_cardinalities.push_back(3);
    m_indices.push_back({_p0, _p1, _p2});
    m_stiffnesses.push_back(1.);
    m_types.push_back(INEQUALITY_CONSTRAINT);
    m_debug_types.push_back(TRAINGLE_COLLISION_CONSTRAINT);

    double thickness = m_surface_thickness;
    m_functions.push_back([thickness, _static_point, _barycentrics, _normal](const std::vector<glm::dvec3> &_p) {
        glm::dvec3 surface_pt = _barycentrics[0] * _p[0] + _barycentrics[1] * _p[1] + _barycentrics[2] * _p[2];
        return glm::dot(surface_pt - _static_point, _normal) - thickness;
    });

    m_gradients.push_back([_barycentrics, _normal](const std::vector<glm::dvec3> &_p) {
        return std::vector<glm::dvec3>{
            _barycentrics[0] * _normal,
            _barycentrics[1] * _normal,
            _barycentrics[2] * _normal};
    });
}

void DynamicObject::addSelfCollisionConstraint(uint _q, uint _p0, uint _p1, uint _p2) {
    Mcoll++;
    m_cardinalities.push_back(4);
    m_indices.push_back({_q, _p0, _p1, _p2});
    m_stiffnesses.push_back(1.);
    m_types.push_back(INEQUALITY_CONSTRAINT);
    m_debug_types.push_back(SELF_COLLISION_CONSTRAINT);

    double thickness = m_surface_thickness;
    m_functions.push_back([thickness](const std::vector<glm::dvec3> &_p) {
        const glm::dvec3 &q = _p[0];
        const glm::dvec3 &p1 = _p[1];
        const glm::dvec3 &p2 = _p[2];
        const glm::dvec3 &p3 = _p[3];

        const glm::dvec3 e2 = p2 - p1;
        const glm::dvec3 e3 = p3 - p1;
        glm::dvec3 n = glm::cross(e2, e3);
        double n_len = glm::length(n);

        if (n_len < 1e-8)
            return 0.0;

        return glm::dot(q - p1, n) / n_len - thickness;
    });

    m_gradients.push_back([](const std::vector<glm::dvec3> &_p) {
        const glm::dvec3 &q = _p[0];
        const glm::dvec3 &p1 = _p[1];
        const glm::dvec3 &p2 = _p[2];
        const glm::dvec3 &p3 = _p[3];

        const glm::dvec3 e2 = p2 - p1;
        const glm::dvec3 e3 = p3 - p1;
        const glm::dvec3 x = q - p1;

        glm::dvec3 n = glm::cross(e2, e3);
        double n_len = glm::length(n);

        if (n_len < 1e-8)
            return std::vector<glm::dvec3>{glm::dvec3(0), glm::dvec3(0), glm::dvec3(0), glm::dvec3(0)};

        glm::dvec3 n_hat = n / n_len;

        // d(C)/d(q) = n_hat
        glm::dvec3 grad_q = n_hat;

        // d(C)/d(p2): from both dot(x,n)/|n| differentiated w.r.t. p2
        // d(n)/d(p2) = tilde(e3)^T  (i.e. cross(*, e3) applied to basis)
        // Gives: cross(x, e3) / n_len - n_hat * dot(n_hat, cross(x, e3)) / n_len
        //      = (I - n_hat*n_hat^T) * cross(x, e3) / n_len
        glm::dvec3 grad_p2 = (glm::cross(x, e3) - glm::dot(n_hat, glm::cross(x, e3)) * n_hat) / n_len;

        // d(C)/d(p3): similarly with cross(e2, x)
        glm::dvec3 grad_p3 = (glm::cross(e2, x) - glm::dot(n_hat, glm::cross(e2, x)) * n_hat) / n_len;

        // d(C)/d(p1): p1 affects x = q-p1 and both edges e2, e3
        // = -grad_q - grad_p2 - grad_p3  (sum of gradients = 0 for translation invariance)
        glm::dvec3 grad_p1 = -grad_q - grad_p2 - grad_p3;

        return std::vector<glm::dvec3>{grad_q, grad_p1, grad_p2, grad_p3};
    });
}

void DynamicObject::addVolumeConstraint(std::vector<glm::uvec3> _indices, double _stiffness, double _pressure, double _targeted_volume) {
    M++;
    m_cardinalities.push_back(getPositions().size());
    std::vector<uint> all_indices(N);
    std::iota(all_indices.begin(), all_indices.end(), 0);
    m_indices.push_back(all_indices);
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(EQUALITY_CONSTRAINT);
    m_debug_types.push_back(VOLUME_CONSTRAINT);

    m_functions.push_back([_targeted_volume, _pressure, _indices](const std::vector<glm::dvec3> &_p) {
        double V = 0;
        for (size_t i = 0; i < _indices.size(); i++) {
            const glm::dvec3 p1 = _p[_indices[i][0]];
            const glm::dvec3 p2 = _p[_indices[i][1]];
            const glm::dvec3 p3 = _p[_indices[i][2]];
            V += glm::dot(glm::cross(p1, p2), p3) / 6.;
        }
        return V - _pressure * _targeted_volume;
    });
    m_gradients.push_back([_indices](const std::vector<glm::dvec3> &_p) {
        std::vector<glm::dvec3> grads(_p.size(), glm::dvec3(0.0));
        for (size_t i = 0; i < _indices.size(); i++) {
            uint i1 = _indices[i][0];
            uint i2 = _indices[i][1];
            uint i3 = _indices[i][2];

            glm::dvec3 p1 = _p[i1];
            glm::dvec3 p2 = _p[i2];
            glm::dvec3 p3 = _p[i3];

            grads[i1] += glm::cross(p2, p3) / 6.;
            grads[i2] += glm::cross(p3, p1) / 6.;
            grads[i3] += glm::cross(p1, p2) / 6.;
        }
        return grads;
    });
}

void DynamicObject::addVolumeConstraint(std::vector<glm::uvec3> _indices, double _stiffness, double _pressure) {
    double V = 0;
    for (size_t i = 0; i < _indices.size(); i++) {
        const glm::dvec3 p1 = m_positions[_indices[i][0]];
        const glm::dvec3 p2 = m_positions[_indices[i][1]];
        const glm::dvec3 p3 = m_positions[_indices[i][2]];
        V += glm::dot(glm::cross(p1, p2), p3) / 6.;
    }
    addVolumeConstraint(_indices, _stiffness, _pressure, V);
}

// Object interaction

void DynamicObject::findNearestPointToLine(const glm::dvec3 &_position, const glm::dvec3 &_direction, uint &point, double &distance, glm::dvec3 &projection) const {
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

bool DynamicObject::updateInteractions(GLFWwindow *_window, const glm::dvec3 &_camera_pos, const glm::dvec3 &_cursor_worldpos) {
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
