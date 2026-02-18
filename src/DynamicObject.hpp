#pragma once

#include "Mesh.hpp"
#include "Transformation.hpp"
#include <functional>

enum ConstraintType {
    EQUALITY_CONSTRAINT,
    INEQUALITY_CONSTRAINT,
};

class DynamicObject : Mesh {
    // Verticies
    // std::vector<glm::vec3> m_positions;  // xi: in Mesh
    int N;                               // number of vertices
    std::vector<glm::vec3> m_velocities; // vi
    std::vector<float> m_masses;         // mi

    // Constraints
    int M;                                                                         // number of contraints
    std::vector<uint> m_cardinalities;                                             // nj: The number of impacted vertices
    std::vector<std::function<float(const std::vector<glm::vec3> &)>> m_functions; // Cj: The constraint itself. Input's size must match the cardinality
    std::vector<std::vector<uint>> m_indices;                                      // Indices of impacted vertices
    std::vector<float> m_stiffnesses;                                              // kj: Strength in [0;1]
    std::vector<ConstraintType> m_types;                                           // Either Equality (=0) or Inequality (>=0)

    // "3.5. Damping" of ./articles/Position_Based_Dynamics.pdf
    void dampVelocities(float k_damping = 1.f); // k_damping = 1. -> rigid body
public:
    // "3.1. Algorithm Overview" of ./articles/Position_Based_Dynamics.pdf
    void update(float _delta_time);
};
