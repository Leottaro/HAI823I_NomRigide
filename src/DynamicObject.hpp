#pragma once

// GLEW
#include <GL/glew.h>

// GLM
#include <glm/glm.hpp>
#include <glm/ext.hpp>

// GLFW
#include <GLFW/glfw3.h>

// EIGEN
#include <Eigen/Dense>

// IMGUI
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

// USUAL INCLUDES
#include <functional>
#include <vector>

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
    // GETTERS
    uint getN() const { return N; };
    const std::vector<glm::vec3> &getPositions() const { return m_positions; };
    const std::vector<glm::vec3> &getVelocities() const { return m_velocities; };
    const std::vector<float> &getMasses() const { return m_masses; };
    const std::vector<float> &getWeights() const { return m_weights; };
    const std::vector<bool> &getFixed() const { return m_fixed; };
    uint getM() const { return M; };
    const std::vector<uint> &getCardinalities() const { return m_cardinalities; };
    const std::vector<constraint_function> &getFunctions() const { return m_functions; };
    const std::vector<gradient_function> &getGradients() const { return m_gradients; };
    const std::vector<std::vector<uint>> &getIndices() const { return m_indices; };
    const std::vector<float> &getStiffnesses() const { return m_stiffnesses; };
    const std::vector<ConstraintType> &getTypes() const { return m_types; };

    // "3.1. Algorithm Overview" of ./articles/Position_Based_Dynamics.pdf
    void update(float _delta_time);

    void addVertex(const glm::vec3 &_position, const glm::vec3 &_velocity, float _mass, bool _fixed);
    void setVertexPosition(uint _pj, glm::vec3 _position) { m_positions[_pj] = _position; }
    void setVertexVelocity(uint _pj, glm::vec3 _velocity) { m_velocities[_pj] = _velocity; }
    void setVertexMass(uint _pj, float _mass) { m_masses[_pj] = _mass; }
    void setVertexWeight(uint _pj, float _weight) { m_weights[_pj] = _weight; }
    void setVertexFixed(uint _pj, bool _fixed);

    void addConstraint(
        uint _cardinality,
        const constraint_function &_function,
        const gradient_function &_gradient,
        const std::vector<uint> &_indices,
        float _stiffness,
        const ConstraintType &_type);
    void addDistanceConstraint(uint _p0, uint _p1, float _stiffness, float _targeted_distance);
    void addDistanceConstraint(uint _p0, uint _p1, float _stiffness); // the targeted distance is set to the current distance between p0 and p1

    // OpenGL interface
private:
    GLuint m_VAO;
    GLuint m_positions_VBO;

    GLuint m_lines_EBO;
    std::vector<glm::uvec2> m_lines;

public:
    void initRendering();
    void updateRenderedPositions();
    void updateRenderedConstraints();
    void render();
    void clear();
};
