#pragma once

#include <vector>
#include <string>
#include <unordered_set>
#include <unordered_map>

#include "DynamicObject.hpp"
#include "ShaderProgram.hpp"

struct StaticBodyDesc {
    std::string name;
    uint mesh_i;
    Transformation transfo{};
    bool real_time{false};

    StaticBodyDesc(const std::string &_name, uint _mesh_i) : name(_name), mesh_i(_mesh_i) {}
};

#define DYNAMIC_OBJECT_PRESEST_N 4
#define IMGUI_DYNAMIC_OBJECT_PRESEST "CustomBody\0RigidBody\0ClothObject\0SoftBody\0"
enum DynamicObjectDescPreset {
    CustomBody,
    RigidBody,
    ClothObject,
    SoftBody
};

struct DynamicObjectDesc {
    std::string name;
    uint mesh_i;
    DynamicObjectDescPreset preset;

    float distance_stiffness;
    float angle_stiffness;
    float volume_stiffness;
    float volume_pressure;
    float damping_coefficient{0.05};

    DynamicRenderType render_type{DynamicRenderType::Auto};
    Transformation transfo{};
    float friction_coefficient{0.5};
    float restitution_coefficient{0.5};

    std::unordered_set<uint> fixed_vertices{};

    DynamicObjectDesc(const std::string &_name, uint _mesh_i, float _distance_stiffness, float _angle_stiffness, float _volume_stiffness, float _volume_pressure) : name(_name), mesh_i(_mesh_i), preset(DynamicObjectDescPreset::CustomBody), distance_stiffness(_distance_stiffness), angle_stiffness(_angle_stiffness), volume_stiffness(_volume_stiffness), volume_pressure(_volume_pressure) {}
    DynamicObjectDesc(const std::string &_name, uint _mesh_i, DynamicObjectDescPreset _preset) : name(_name), mesh_i(_mesh_i), preset(_preset) {
        applyPreset();
    }

    void applyPreset() {
        switch (preset) {
        case DynamicObjectDescPreset::CustomBody:
            break;
        case DynamicObjectDescPreset::RigidBody:
            distance_stiffness = 1.;
            angle_stiffness = 1.;
            volume_stiffness = 1.;
            volume_pressure = 1.;
            damping_coefficient = 1.;
            break;
        case DynamicObjectDescPreset::ClothObject:
            distance_stiffness = .9;
            angle_stiffness = .9;
            volume_stiffness = 0.;
            volume_pressure = 1.;
            break;
        case DynamicObjectDescPreset::SoftBody:
            distance_stiffness = .9;
            angle_stiffness = .9;
            volume_stiffness = .9;
            volume_pressure = 1.;
            break;
        default:
            throw new std::runtime_error("unimplemented DynamicObjectDescPreset in DynamicObjectDesc constructor.");
        }
    }
};

class Scene {
private:
    // meshes
    // uint m_meshes_count = 0;
    std::vector<MeshType> m_meshes_type = {};
    std::vector<Mesh> m_meshes = {};
    MeshType m_new_mesh_type;

    // static bodies
    // uint m_static_bodies_count = 0;
    std::vector<StaticBodyDesc> m_static_bodies_desc = {};
    std::string m_new_static_name;
    std::vector<uint> m_static_bodies_mesh_i = {};
    std::vector<Transformation> m_static_bodies_transfo = {};

    // dynamic objects
    // uint m_dynamic_objects_count = 0;
    std::vector<DynamicObjectDesc> m_dynamic_objects_desc = {};
    std::string m_new_dynamic_name;
    std::vector<int> m_dynamic_fixed_input_buffers;
    std::vector<DynamicObject> m_dynamic_objects;

public:
    uint solver_iterations = 100;
    bool do_fixed_delta_time = false;
    double fixed_delta_time = 1.e-6;

    inline void addMesh(const MeshType &_type) { m_meshes_type.push_back(_type); }
    inline std::vector<MeshType> &getMeshesType() { return m_meshes_type; }
    inline std::vector<Mesh> &getCurrentMeshes() { return m_meshes; }

    inline Transformation *addStaticBody(const StaticBodyDesc &_desc) {
        m_static_bodies_desc.push_back(_desc);
        return &m_static_bodies_desc.back().transfo;
    }
    inline std::vector<StaticBodyDesc> &getStaticBodiesDesc() { return m_static_bodies_desc; }

    inline Transformation *addDynamicObject(const DynamicObjectDesc &_desc) {
        m_dynamic_objects_desc.push_back(_desc);
        return &m_dynamic_objects_desc.back().transfo;
    }
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