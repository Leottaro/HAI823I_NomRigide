#include "DynamicObject.hpp"

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
void DynamicObject::dampVelocities(float k_damping) { // k_damping = 1. -> rigid body
    // (1) to (5)
    float total_mass = 0.f;
    glm::vec3 xcm = glm::vec3(0.); // (1) : global linear velocity
    glm::vec3 vcm = glm::vec3(0.); // (2)
    glm::vec3 L = glm::vec3(0.);   // (3)
    glm::mat3 I = glm::mat3(0.);   // (4)
    for (int i = 0; i < N; i++) {
        xcm += m_positions[i] * m_masses[i];  // (1)
        vcm += m_velocities[i] * m_masses[i]; // (2)
        // (3)
        glm::vec3 ri = m_positions[i] - xcm;
        L += glm::cross(ri, (m_masses[i] * m_velocities[i]));
        // (4)
        glm::mat3 r_tilde_i = glm::mat3(0, -ri.z, ri.y, ri.z, 0, -ri.x, -ri.y, ri.x, 0);
        I += r_tilde_i * glm::transpose(r_tilde_i) * m_masses[i];
    }
    xcm /= total_mass;                     // (2)
    vcm /= total_mass;                     // (3)
    glm::vec3 omega = glm::inverse(I) * L; // (5): angular velocity

    for (int i = 0; i < N; i++) { // (6)
        glm::vec3 ri = m_positions[i] - xcm;
        glm::vec3 dvi = vcm + glm::cross(omega, ri) - m_velocities[i]; // (7)
        m_velocities[i] += k_damping * dvi;                            // (8)
    } // (9)
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
    // (1)-(3) : Initialize the state variables
    std::vector<glm::vec3> positions(N);
    std::vector<glm::vec3> velocities(N);
    std::vector<float> weights(N);
    for (int i = 0; i < N; i++) {
        positions[i] = m_positions[i];
        velocities[i] = m_velocities[i];
        weights[i] = 1.f / m_masses[i];
    }

    std::vector<glm::vec3> new_positions(N);
    while (1) { // (4)
        // (5) external forces (gravity, etc...) (for now, just gravity)
        for (int i = 0; i < N; i++)
            velocities[i] += _delta_time * 9.807f;
        dampVelocities(1.f); // TODO: (6) damp les velocities ou les m_velocities ????
        // (7)
        for (int i = 0; i < N; i++)
            new_positions[i] = m_positions[i] + _delta_time * m_velocities[i];
        // TODO: (8) Collisions ???
        // TODO: (9)-(12) Solver (Eigen ???)
        for (int i = 0; i < N; i++) { // (12)
            velocities[i] = (new_positions[i] - velocities[i]) / _delta_time;
            positions[i] = new_positions[i];
        } // (15)
        // TODO: (16) Velocity update
    } // (17)
}