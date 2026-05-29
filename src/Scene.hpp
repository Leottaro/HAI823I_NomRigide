#pragma once

#include <vector>
#include <string>
#include <set>
#include <map>

#include "DynamicObject.hpp"
#include "ShaderProgram.hpp"
#include "Camera.hpp"
struct StaticBodyDesc {
    std::string name;
    uint mesh_i;
    Transformation transfo{};
    bool real_time{true};

    StaticBodyDesc(const std::string& _name, uint _mesh_i) : name(_name), mesh_i(_mesh_i) {}
};

#define DYNAMIC_OBJECT_PRESEST_N 5
#define IMGUI_DYNAMIC_OBJECT_PRESEST "CustomBody\0RigidBody\0ClothObject\0ClothBalloon\0SoftBody\0"
enum DynamicObjectDescPreset {
    CustomBody,
    RigidBody,
    Cloth,
    ClothBalloon,
    SoftBody
};

struct DynamicObjectDesc {
    std::string name;
    uint mesh_i;
    DynamicObjectDescPreset preset;

    DynamicRenderType render_type{DynamicRenderType::Auto};
    glm::vec3 render_color{1.f};

    float distance_stiffness;
    float angle_stiffness;
    float volume_stiffness;
    float volume_pressure;
    float damping_coefficient{0.05};
    float surface_thickness{1.e-1};
    float collision_detection_margin{1.e-1};
    float vertex_mass{1.f};

    Transformation transfo{};
    float friction_coefficient{0.5};
    float restitution_coefficient{0.5};

    std::set<uint> fixed_vertices{};

    DynamicObjectDesc(const std::string& _name, uint _mesh_i, float _distance_stiffness, float _angle_stiffness, float _volume_stiffness, float _volume_pressure) : name(_name), mesh_i(_mesh_i), preset(DynamicObjectDescPreset::CustomBody), distance_stiffness(_distance_stiffness), angle_stiffness(_angle_stiffness), volume_stiffness(_volume_stiffness), volume_pressure(_volume_pressure) {}
    DynamicObjectDesc(const std::string& _name, uint _mesh_i, DynamicObjectDescPreset _preset) : name(_name), mesh_i(_mesh_i), preset(_preset) {
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
        case DynamicObjectDescPreset::Cloth:
            distance_stiffness = .9;
            angle_stiffness = 0.;
            volume_stiffness = 0.;
            volume_pressure = 1.;
            damping_coefficient = 0.;
            break;
        case DynamicObjectDescPreset::ClothBalloon:
            distance_stiffness = 0.5;
            angle_stiffness = 0.;
            volume_stiffness = 0.9;
            volume_pressure = 1.5;
            damping_coefficient = 0.;
            break;
        case DynamicObjectDescPreset::SoftBody:
            distance_stiffness = .9;
            angle_stiffness = .9;
            volume_stiffness = .9;
            volume_pressure = 1.;
            damping_coefficient = 0.05;
            break;
        default:
            throw new std::runtime_error("unimplemented DynamicObjectDescPreset in DynamicObjectDesc constructor.");
        }
    }
};

class Scene {
private:
    // meshes
    // uint m_meshes_count0;
    std::vector<MeshType> m_meshes_type{};
    std::vector<Mesh> m_meshes{};
    MeshType m_new_mesh_type;

    // static bodies
    // uint m_static_bodies_count0;
    std::vector<StaticBodyDesc> m_static_bodies_desc{};
    std::string m_new_static_name;
    std::vector<uint> m_static_bodies_mesh_i{};
    std::vector<Transformation> m_static_bodies_transfo{};

    // dynamic objects
    // uint m_dynamic_objects_count0;
    std::vector<DynamicObjectDesc> m_dynamic_objects_desc{};
    std::string m_new_dynamic_name;
    std::vector<int> m_dynamic_fixed_input_buffers;
    std::vector<DynamicObject> m_dynamic_objects;

public:
    uint solver_iterations{10};
    uint num_subSteps{4};
    ConstraintSolverType constraint_solver{GaussSeidelSolver};
    bool do_fixed_delta_time{false};
    double fixed_delta_time{1.e-6};
    bool do_gravity{true};
    bool do_point_triangle_collision{true};
    bool do_edge_triangle_collision{true};
    bool do_triangle_point_collision{true};
    bool do_self_collision{false};
    bool do_inter_dynamic_collision{false};

    inline void addMesh(const MeshType& _type) { m_meshes_type.push_back(_type); }
    inline std::vector<MeshType>& getMeshesType() { return m_meshes_type; }
    inline std::vector<Mesh>& getCurrentMeshes() { return m_meshes; }

    inline Transformation* addStaticBody(const StaticBodyDesc& _desc) {
        m_static_bodies_desc.push_back(_desc);
        return &m_static_bodies_desc.back().transfo;
    }
    inline std::vector<StaticBodyDesc>& getStaticBodiesDesc() { return m_static_bodies_desc; }

    inline Transformation* addDynamicObject(const DynamicObjectDesc& _desc) {
        m_dynamic_objects_desc.push_back(_desc);
        return &m_dynamic_objects_desc.back().transfo;
    }
    inline std::vector<DynamicObjectDesc>& getDynamicObjectsDesc() { return m_dynamic_objects_desc; }
    inline std::vector<DynamicObject>& getCurrentDynamicObjects() { return m_dynamic_objects; }

    void resetMesh(uint _i);
    void resetStaticBody(uint _i);
    void resetDynamicObject(uint _i);
    void resetObjects();

    bool updateInterface();
    void meshTypeInterface(uint _i);
    void staticBodyInterface(uint _i);
    void dynamicObjectInterface(uint _i);

    bool updateInteractions(GLFWwindow* _window, const glm::dvec3& _camera_pos, const glm::dvec3& _cursor_worldpos);
    bool updateSimulation(float _deltaTime, float _fullDeltaTime, bool _is_first_step);
    void updateAllRendredPositions();

    void render(const ShaderProgram& _dynamic_shader, const ShaderProgram& _mesh_shader, const Camera& _camera) const;
    void clear();
};
