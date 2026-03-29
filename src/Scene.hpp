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

class Scene {
private:
    // meshes
    std::vector<MeshType> m_meshes_type = {};
    std::vector<Mesh> m_meshes = {};

    // static bodies
    std::vector<StaticBodyDesc> m_static_bodies_desc = {};
    std::vector<bool> m_static_realtime_reset = {};
    std::vector<StaticBody> m_static_bodies = {};
    std::vector<Transformation> m_static_transformations = {};

    // dynamic objects
    std::vector<DynamicObjectDesc> m_dynamic_objects_desc = {};
    std::vector<DynamicObject> m_dynamic_objects;

public:
    uint solver_iterations = 100;
    bool do_fixed_delta_time = false;
    double fixed_delta_time = 1.e-6;

    inline void addMesh(const MeshType &_type) { m_meshes_type.push_back(_type); }
    inline std::vector<MeshType> &getMeshesType() { return m_meshes_type; }
    inline std::vector<Mesh> &getCurrentMeshes() { return m_meshes; }

    inline void addStaticBody(const StaticBodyDesc &_desc) { m_static_bodies_desc.push_back(_desc); }
    inline std::vector<StaticBodyDesc> &getStaticBodiesDesc() { return m_static_bodies_desc; }
    inline std::vector<StaticBody> &getCurrentStaticBodies() { return m_static_bodies; }

    inline void addDynamicObject(const DynamicObjectDesc &_desc) { m_dynamic_objects_desc.push_back(_desc); }
    inline std::vector<DynamicObjectDesc> &getDynamicObjectsDesc() { return m_dynamic_objects_desc; }
    inline std::vector<DynamicObject> &getCurrentDynamicObjects() { return m_dynamic_objects; }

    void resetMesh(uint _i);
    void resetStaticBody(uint _i);
    void resetDynamicObject(uint _i);
    void resetObjects();

    bool updateInterface();
    void meshTypeInterface(uint _i);
    void staticBodyInterface(uint _i);
    void dynamicObjectInterface(uint _i);

    bool updateInteractions(GLFWwindow *_window, const glm::dvec3 &_camera_pos, const glm::dvec3 &_cursor_worldpos);
    bool updateSimulation(float _deltaTime);

    void render(const ShaderProgram &_dynamic_shader, const ShaderProgram &_mesh_shader, const glm::mat4 &_projection, const glm::mat4 &_view) const;
    void clear();
};