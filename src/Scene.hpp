#pragma once

#include <vector>
#include <string>
#include <unordered_set>

#include "DynamicObject.hpp"
#include "ShaderProgram.hpp"

struct StaticBodyDesc {
    std::string name;
    uint mesh_i;
    Transformation *transfo;
};

struct DynamicObjectDesc : StaticBodyDesc {
    float distance_stiffness;
    float angle_stiffness;
    float volume_stiffness;
    float volume_pressure;

    std::unordered_set<uint> fixed_vertices = {};
    float ambient_friction_coefficient = 0.01;
    float friction_coefficient = 0.5;
    float restitution_coefficient = 0.5;
};

// struct DynamicObjectDesc {
//     Mesh *mesh;
//     Transformation *transfo;
//     char *name;
//     float distance_stiffness;
//     float angle_stiffness;

//     DynamicObjectDesc(Mesh *mesh, Transformation *transfo, char *name, float distance_stiffness, float angle_stiffness)
//         : mesh(mesh), transfo(transfo), name(name), distance_stiffness(distance_stiffness), angle_stiffness(angle_stiffness) {}
// };

class Scene {
private:
    std::vector<Mesh> m_meshes = {};

    // static bodies
    std::vector<StaticBodyDesc> m_static_bodies_desc = {};
    std::vector<Transformation> m_static_transformations = {};
    std::vector<StaticBody> m_static_bodies = {};

    // dynamic objects
    std::vector<DynamicObjectDesc> m_dynamic_objects_desc = {};
    std::vector<DynamicObject> m_dynamic_objects;

public:
    uint solver_iterations = 100;
    bool do_fixed_delta_time = false;
    double fixed_delta_time = 1.e-6;

    inline void addMesh(const Mesh &_mesh) { m_meshes.push_back(_mesh); }
    inline const Mesh &getMesh(uint i) const { return m_meshes[i]; }

    inline std::vector<StaticBodyDesc> &getStaticBodiesDesc() { return m_static_bodies_desc; }
    inline void addStaticBody(const StaticBodyDesc &_desc) { m_static_bodies_desc.push_back(_desc); }

    inline std::vector<DynamicObjectDesc> &getDynamicObjectsDesc() { return m_dynamic_objects_desc; }
    inline void addDynamicObject(const DynamicObjectDesc &_desc) { m_dynamic_objects_desc.push_back(_desc); }

    void resetObjects();

    bool updateInterface();
    void staticBodyInterface(StaticBodyDesc &object);
    void dynamicObjectInterface(DynamicObjectDesc &object);

    bool updateInteractions(GLFWwindow *_window, const glm::dvec3 &_camera_pos, const glm::dvec3 &_cursor_worldpos);
    bool updateSimulation(float _deltaTime);

    void init();
    void render(const ShaderProgram &_dynamic_shader, const ShaderProgram &_mesh_shader, const glm::mat4 &_projection, const glm::mat4 &_view) const;
    void clear();
};