#pragma once

#include <vector>
#include <string>

#include "DynamicObject.hpp"
#include "ShaderProgram.hpp"

struct StaticBodyDesc {
    std::string name;
    uint mesh_i;
    Transformation transfo;
};

struct DynamicObjectDesc {
    std::string name;
    DynamicObject object;
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
    std::vector<StaticBodyDesc> m_pre_static_bodies = {};
    std::vector<StaticBodyDesc> m_static_bodies = {};

    std::vector<DynamicObjectDesc> m_pre_dynamic_objects = {};
    std::vector<DynamicObjectDesc> m_dynamic_objects;
    bool m_should_rerender = false;

public:
    inline void addMesh(const Mesh &_mesh) { m_meshes.push_back(_mesh); }
    inline const Mesh &getMesh(uint i) const { return m_meshes[i]; }

    inline std::vector<StaticBodyDesc> &getStaticBodiesDesc() { return m_static_bodies; }
    inline void addStaticBody(const std::string &_name, uint _mesh_i, const Transformation &_transfo) { m_pre_static_bodies.push_back({_name, _mesh_i, _transfo}); }

    inline std::vector<DynamicObjectDesc> &getDynamicObjects() { return m_dynamic_objects; }
    inline void addDynamicObject(const std::string &_name, const DynamicObject &_object) { m_pre_dynamic_objects.push_back({_name, _object}); }

    void resetObjects();
    bool updateInterface();
    bool updateSimulation(float _deltaTime);

    void init();
    void render(const ShaderProgram &_dynamic_shader, const ShaderProgram &_mesh_shader, const glm::mat4 &_projection, const glm::mat4 &_view) const;
    void clear();
};