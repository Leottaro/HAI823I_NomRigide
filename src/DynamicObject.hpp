#pragma once

#include "Mesh.hpp"
#include "Transformation.hpp"
#include <functional>

typedef std::function<float(const std::vector<glm::vec3> &)> constraint_function;
typedef std::function<glm::vec3(const std::vector<glm::vec3> &, uint)> gradient_function;

enum ConstraintType {
    EQUALITY_CONSTRAINT,
    INEQUALITY_CONSTRAINT,
};

class DynamicObject {
    // Verticies
    uint N = 0;                          // number of vertices
    std::vector<glm::vec3> m_positions;  // xi
    std::vector<glm::vec3> m_velocities; // vi
    std::vector<float> m_masses;         // mi
    std::vector<float> m_weights;        // wi
    std::vector<bool> m_fixed;           // if the vertex is fixed

    // Constraints
    uint M = 0;                                   // number of contraints
    std::vector<uint> m_cardinalities;            // nj: The number of impacted vertices
    std::vector<constraint_function> m_functions; // Cj: The constraint itself. Input's size must match the cardinality
    std::vector<gradient_function> m_gradients;   // Cj: The gradient (evolution) of the constraint. Input's size must match the cardinality
    std::vector<std::vector<uint>> m_indices;     // Indices of impacted vertices
    std::vector<float> m_stiffnesses;             // kj: Strength in [0;1]
    std::vector<ConstraintType> m_types;          // Either Equality (=0) or Inequality (>=0)

    // "3.5. Damping" of ./articles/Position_Based_Dynamics.pdf
    void dampVelocities(float k_damping = 1.f); // k_damping = 1. -> rigid body

    void fillMissingVertexInfos() {
        m_velocities.resize(N);
        m_masses.resize(N);
    }

public:
    // "3.1. Algorithm Overview" of ./articles/Position_Based_Dynamics.pdf
    void update(float _delta_time);

    void addVertex(const glm::vec3 &_position, const glm::vec3 &_velocity, float _mass, bool _fixed);
    void setVertexFixed(uint _pj, bool _fixed);

    void addConstraint(
        uint _cardinality,
        const constraint_function &_function,
        const gradient_function &_gradient,
        const std::vector<uint> &_indices,
        float _stiffness,
        const ConstraintType &_type);
    void addDistanceConstraint(
        float _targeted_distance,
        const std::vector<uint> &_indices,
        float _stiffness,
        const ConstraintType &_type);

    // OpenGL interface
private:
    GLuint m_VAO;
    GLuint m_positions_VBO;

public:
    void init();
    void render();
    void clear();
};
