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
#include "Mesh.hpp"
#include "Transformation.hpp"
#include <functional>
#include <vector>

struct StaticBody {
    Mesh *m_mesh;
    Transformation *m_transformation;

    StaticBody(Mesh *_mesh, Transformation *_transformation) : m_mesh(_mesh), m_transformation(_transformation) {}
};

typedef std::function<double(const std::vector<glm::dvec3> &)> constraint_function;
typedef std::function<std::vector<glm::dvec3>(const std::vector<glm::dvec3> &)> gradient_function;

enum ConstraintType {
    EQUALITY_CONSTRAINT,
    INEQUALITY_CONSTRAINT,
};

class DynamicObject {
    // Verticies
    uint N = 0;                           // number of vertices
    std::vector<glm::dvec3> m_positions;  // xi
    std::vector<glm::dvec3> m_velocities; // vi
    std::vector<double> m_masses;         // mi
    std::vector<double> m_weights;        // wi
    std::vector<bool> m_fixed;            // if the vertex is fixed

    // Constraints
    uint M = 0, Mcoll = 0;                        // number of contraints
    std::vector<uint> m_cardinalities;            // nj: The number of impacted vertices
    std::vector<constraint_function> m_functions; // Cj: The constraint itself. Input's size must match the cardinality
    std::vector<gradient_function> m_gradients;   // Cj: The gradient (evolution) of the constraint. Input's size must match the cardinality
    std::vector<std::vector<uint>> m_indices;     // Indices of impacted vertices
    std::vector<double> m_stiffnesses;            // kj: Strength in [0;1]
    std::vector<ConstraintType> m_types;          // Either Equality (=0) or Inequality (>=0)

    // Collisions parameters
    double m_ambient_friction_coefficient = 0.01;
    double m_friction_coefficient = 0.5;
    double m_restitution_coefficient = 0.5;

    // "3.5. Damping" of ./articles/Position_Based_Dynamics.pdf
    void dampVelocities(double k_damping = 1.); // k_damping = 1. -> rigid body

    void fillMissingVertexInfos() {
        m_velocities.resize(N);
        m_masses.resize(N);
    }

    void addCollisionConstraint(uint _p0, glm::dvec3 _intersection, glm::dvec3 _normal, double _stiffness);
    void addEdgeCollisionConstraint(uint _p0, uint _p1, double _alpha, glm::dvec3 _surface_point, glm::dvec3 _normal, double _stiffness);

public:
    // GETTERS
    uint getN() const { return N; };
    const std::vector<glm::dvec3> &getPositions() const { return m_positions; };
    const std::vector<glm::dvec3> &getVelocities() const { return m_velocities; };
    const std::vector<double> &getMasses() const { return m_masses; };
    const std::vector<double> &getWeights() const { return m_weights; };
    const std::vector<bool> &getFixed() const { return m_fixed; };
    uint getM() const { return M; };
    const std::vector<uint> &getCardinalities() const { return m_cardinalities; };
    const std::vector<constraint_function> &getFunctions() const { return m_functions; };
    const std::vector<gradient_function> &getGradients() const { return m_gradients; };
    const std::vector<std::vector<uint>> &getIndices() const { return m_indices; };
    const std::vector<double> &getStiffnesses() const { return m_stiffnesses; };
    const std::vector<ConstraintType> &getTypes() const { return m_types; };

    void setAmbientFrictionCoefficient(double _coeff) { m_ambient_friction_coefficient = _coeff; }
    double getAmbientFrictionCoefficient() { return m_ambient_friction_coefficient; }
    void setFrictionCoefficient(double _coeff) { m_friction_coefficient = _coeff; }
    double getFrictionCoefficient() { return m_friction_coefficient; }
    void setRestitutionCoefficient(double _coeff) { m_restitution_coefficient = _coeff; }
    double getRestitutionCoefficient() { return m_restitution_coefficient; }

    // "3.1. Algorithm Overview" of ./articles/Position_Based_Dynamics.pdf
    bool update(double _delta_time, const std::vector<StaticBody> &static_bodies);

    void addVertex(const glm::dvec3 &_position, const glm::dvec3 &_velocity, double _mass, bool _fixed);
    void setVertexPosition(uint _pj, glm::dvec3 _position) { m_positions[_pj] = _position; }
    void setVertexVelocity(uint _pj, glm::dvec3 _velocity) { m_velocities[_pj] = _velocity; }
    void setVertexMass(uint _pj, double _mass) { m_masses[_pj] = _mass; }
    void setVertexWeight(uint _pj, double _weight) { m_weights[_pj] = _weight; }
    void setVertexFixed(uint _pj, bool _fixed);

    void addConstraint(
        uint _cardinality,
        const constraint_function &_function,
        const gradient_function &_gradient,
        const std::vector<uint> &_indices,
        double _stiffness,
        const ConstraintType &_type);
    void addDistanceConstraint(uint _p0, uint _p1, double _stiffness, double _targeted_distance);
    void addDistanceConstraint(uint _p0, uint _p1, double _stiffness); // the targeted distance is set to the current distance between p0 and p1
    void addBendingConstraint(uint _p0, uint _p1, uint _p2, uint _p3, double _stiffness, double _targeted_angle);
    void addBendingConstraint(uint _p0, uint _p1, uint _p2, uint _p3, double _stiffness); // the targeted angle is set to the current angle between p0,p2,p1 normal and p0,p3,p1 normal

    // Objects creation
    static DynamicObject bodyFromMesh(const StaticBody &_static_body, float _distance_stifness, float _angle_stifness);
    static DynamicObject rigidBodyFromMesh(const StaticBody &_static_body) { return bodyFromMesh(_static_body, 1.f, 1.f); }

    // OpenGL interface

private:
    GLuint m_VAO;
    GLuint m_positions_VBO;

    GLuint m_lines_EBO;
    std::vector<glm::uvec2> m_lines;

public:
    void addDrawLine(uint _p0, uint _p1) { m_lines.push_back(glm::uvec2(_p0, _p1)); };

    void initRendering();
    void updateRenderedPositions();
    void updateRenderedConstraints();
    void render();
    void clear();
};
