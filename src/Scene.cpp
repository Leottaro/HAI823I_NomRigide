#include "Scene.hpp"

#include <variant>
#include <iostream>

void Scene::resetObjects() {
    m_meshes.resize(m_meshes_type.size());
    for (uint i = 0; i < m_meshes_type.size(); i++) {
        m_meshes[i].clear();
        m_meshes[i] = Mesh(m_meshes_type[i]);
        m_meshes[i].init();
    }

    m_static_bodies.resize(m_static_bodies_desc.size());
    m_static_transformations.resize(m_static_bodies_desc.size());
    for (uint i = 0; i < m_static_bodies_desc.size(); i++) {
        m_static_bodies[i].m_mesh = &m_meshes[m_static_bodies_desc[i].mesh_i];
        m_static_transformations[i] = *m_static_bodies_desc[i].transfo;
        m_static_bodies[i].m_transformation = &m_static_transformations[i];
    }

    m_dynamic_objects.resize(m_dynamic_objects_desc.size());
    for (uint i = 0; i < m_dynamic_objects_desc.size(); i++) {
        m_dynamic_objects[i].clear();
        m_dynamic_objects[i] = DynamicObject::bodyFromMesh({&m_meshes[m_dynamic_objects_desc[i].mesh_i], m_dynamic_objects_desc[i].transfo}, m_dynamic_objects_desc[i].distance_stiffness, m_dynamic_objects_desc[i].angle_stiffness, m_dynamic_objects_desc[i].volume_stiffness, m_dynamic_objects_desc[i].volume_pressure);
        for (uint pj : m_dynamic_objects_desc[i].fixed_vertices) {
            m_dynamic_objects[i].setVertexFixed(pj, true);
        }
        m_dynamic_objects[i].setAmbientFrictionCoefficient(m_dynamic_objects_desc[i].ambient_friction_coefficient);
        m_dynamic_objects[i].setFrictionCoefficient(m_dynamic_objects_desc[i].friction_coefficient);
        m_dynamic_objects[i].setRestitutionCoefficient(m_dynamic_objects_desc[i].restitution_coefficient);

        m_dynamic_objects[i].initRendering();
    }
    // std::cout << "Initialisation terminée avec " << static_bodies.size() << " objets statiques et " << dynamic_objects.size() << " objets dynamiques." << std::endl;
}

void Scene::meshTypeInterface(uint _mesh_i) {
    ImGui::Text("%d:", _mesh_i);
    ImGui::SameLine();
    int _type_int = meshTypeToInt(m_meshes_type[_mesh_i]);
    if (ImGui::Combo(("type##" + std::to_string(_mesh_i)).c_str(), &_type_int, ALL_MESH_TYPES) && _type_int != meshTypeToInt(m_meshes_type[_mesh_i])) {
        m_meshes_type[_mesh_i] = meshTypeFromInt(_type_int);
    }

    ImGui::Indent();
    std::visit(
        [_mesh_i](auto &mesh_spec) {
            using T = std::decay_t<decltype(mesh_spec)>;
            if constexpr (std::is_same_v<T, LoadedMesh>) {
                char path_buffer[256];
                std::strncpy(path_buffer, mesh_spec.path.c_str(), sizeof(path_buffer) - 1);
                path_buffer[sizeof(path_buffer) - 1] = '\0';
                if (ImGui::InputText(("path##" + std::to_string(_mesh_i)).c_str(), path_buffer, sizeof(path_buffer))) {
                    mesh_spec.path = path_buffer;
                }
            } else if constexpr (std::is_same_v<T, SingleTriangleMesh>) {
            } else if constexpr (std::is_same_v<T, SimpleGridMesh>) {
                glm::ivec2 dimensions(mesh_spec.nx, mesh_spec.nz);
                if (ImGui::DragInt2(("x,z##" + std::to_string(_mesh_i)).c_str(), glm::value_ptr(dimensions), 0.01, 2, INT32_MAX)) {
                    mesh_spec.nx = std::max(2, dimensions.x);
                    mesh_spec.nz = std::max(2, dimensions.y);
                }
            } else if constexpr (std::is_same_v<T, SimpleTerrainMesh>) {
                glm::ivec2 dimensions(mesh_spec.nx, mesh_spec.nz);
                if (ImGui::DragInt2(("x,z##" + std::to_string(_mesh_i)).c_str(), glm::value_ptr(dimensions), 0.01, 2, INT32_MAX)) {
                    mesh_spec.nx = std::max(2, dimensions.x);
                    mesh_spec.nz = std::max(2, dimensions.y);
                }
                ImGui::DragFloat2(("y range##" + std::to_string(_mesh_i)).c_str(), glm::value_ptr(mesh_spec.y_range), 0.001, -FLT_MAX, FLT_MAX);
            } else if constexpr (std::is_same_v<T, CubeMesh>) {
                int details = mesh_spec.n;
                if (ImGui::DragInt(("details##" + std::to_string(_mesh_i)).c_str(), &details, 0.01, 2, INT32_MAX)) {
                    mesh_spec.n = std::max(2, details);
                }
            } else if constexpr (std::is_same_v<T, CubeSphereMesh>) {
                int details = mesh_spec.n;
                if (ImGui::DragInt(("details##" + std::to_string(_mesh_i)).c_str(), &details, 0.01, 3, INT32_MAX)) {
                    mesh_spec.n = std::max(3, details);
                }
            }
        },
        m_meshes_type[_mesh_i]);
    ImGui::Unindent();
}

void Scene::staticBodyInterface(StaticBodyDesc &_object) {
    ImGui::Text("%s: ", _object.name.c_str());

    int mi = _object.mesh_i;
    if (ImGui::InputInt(("mesh i##" + _object.name).c_str(), &mi, 1, 1)) {
        _object.mesh_i = std::clamp(mi, 0, int(m_meshes.size() - 1));
    }

    ImGui::Spacing();
    ImGui::DragFloat3(("Position##" + _object.name).c_str(), glm::value_ptr(_object.transfo->getTranslation()), .01f, -FLT_MAX, FLT_MAX);
    ImGui::DragFloat3(("Scale##" + _object.name).c_str(), glm::value_ptr(_object.transfo->getScale()), .01f, -FLT_MAX, FLT_MAX);
    glm::vec3 ea_degree = glm::degrees(_object.transfo->getEulerAngles());
    if (ImGui::DragFloat3(("Pitch Yaw Roll##" + _object.name).c_str(), glm::value_ptr(ea_degree), .5f, -360.f, 360.f)) {
        _object.transfo->setEulerAngles(glm::radians(ea_degree));
        _object.transfo->updateRotation();
    }
}

void Scene::dynamicObjectInterface(DynamicObjectDesc &_object) {
    staticBodyInterface(_object);

    ImGui::Spacing();
    ImGui::DragFloat(("Ambiant friction##" + _object.name).c_str(), &_object.ambient_friction_coefficient, .01f, 0.f, 1.f);
    ImGui::DragFloat(("Friction##" + _object.name).c_str(), &_object.friction_coefficient, .01f, 0.f, 1.f);
    ImGui::DragFloat(("Restitution##" + _object.name).c_str(), &_object.restitution_coefficient, .01f, 0.f, 1.f);

    ImGui::Spacing();
    ImGui::DragFloat(("distance stiffness##" + _object.name).c_str(), &_object.distance_stiffness, 0.001f, 0.f, 1.f);
    ImGui::DragFloat(("angle stiffness##" + _object.name).c_str(), &_object.angle_stiffness, 0.001f, 0.f, 1.f);
    ImGui::DragFloat(("volume stiffness##" + _object.name).c_str(), &_object.volume_stiffness, 0.001f, 0.f, 1.f);
    ImGui::DragFloat(("volume pressure##" + _object.name).c_str(), &_object.volume_pressure, 0.001f, 0.f, 1.f);
}

bool Scene::updateInterface() {
    bool disable_mouse_actions = false;
    if (ImGui::Begin("Scene")) {
        disable_mouse_actions = ImGui::IsWindowHovered() || ImGui::IsAnyItemHovered() || ImGui::IsAnyItemActive() || ImGui::IsAnyItemFocused();
        ImGui::SeparatorText("Global settings");
        int si = solver_iterations;
        if (ImGui::DragInt("Solver iterations", &si, 0.5f, 1, 1000)) {
            solver_iterations = si;
        };
        ImGui::Checkbox("Do fixed delta time", &do_fixed_delta_time);
        if (do_fixed_delta_time) {
            if (ImGui::InputDouble("Fixed delta time", &fixed_delta_time, 1.e-6, 0.0001)) {
                fixed_delta_time = std::clamp(fixed_delta_time, 1.e-6, 1.);
            }
        }
        if (ImGui::Button("Reset Objects")) {
            resetObjects();
        }

        ImGui::Spacing();
        ImGui::SeparatorText("Meshes");
        ImGui::Spacing();

        for (uint i = 0; i < m_meshes_type.size(); i++) {
            if (i > 0) {
                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();
            }
            meshTypeInterface(i);
        }

        ImGui::Spacing();
        ImGui::SeparatorText("Static Bodies");
        ImGui::Spacing();

        for (uint i = 0; i < m_static_bodies_desc.size(); i++) {
            if (i > 0) {
                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();
            }
            staticBodyInterface(m_static_bodies_desc[i]);
        }

        ImGui::Spacing();
        ImGui::SeparatorText("Dynamic Objects");
        ImGui::Spacing();

        for (uint i = 0; i < m_dynamic_objects_desc.size(); i++) {
            if (i > 0) {
                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();
            }
            dynamicObjectInterface(m_dynamic_objects_desc[i]);
        }
    }
    ImGui::End();
    return disable_mouse_actions;
}

bool Scene::updateInteractions(GLFWwindow *_window, const glm::dvec3 &_camera_pos, const glm::dvec3 &_cursor_worldpos) {
    bool res = false;
    for (DynamicObject &object : m_dynamic_objects) {
        if (object.updateInteractions(_window, _camera_pos, _cursor_worldpos)) {
            object.updateRenderedPositions();
            res = true;
        }
    }
    return res;
}

bool Scene::updateSimulation(float _deltaTime) {
    if (do_fixed_delta_time) {
        _deltaTime = fixed_delta_time;
    }
    for (DynamicObject &obj : m_dynamic_objects) {
        if (!obj.update(_deltaTime, solver_iterations, m_static_bodies)) {
            obj.updateRenderedPositions();
            return false;
        }
        obj.updateRenderedPositions();
    }

    return true;
}

void Scene::render(const ShaderProgram &_dynamic_shader, const ShaderProgram &_mesh_shader, const glm::mat4 &_projection, const glm::mat4 &_view) const {
    // DYNAMIC OBJECTS RENDERING
    _dynamic_shader.use();
    _dynamic_shader.set("projection", _projection);
    _dynamic_shader.set("view", _view);
    for (const DynamicObject &object : m_dynamic_objects) {
        object.render();
    }

    // STATIC OBJECTS RENDERING
    _mesh_shader.use();
    _mesh_shader.set("projection", _projection);
    for (const StaticBody &static_body : m_static_bodies) {
        glm::mat4 model = static_body.m_transformation->computeTransformationMatrix();
        glm::mat4 model_view = _view * model;
        glm::mat4 normal_mat = glm::transpose(glm::inverse(model_view));
        _mesh_shader.set("model_view", model_view);
        _mesh_shader.set("normal_mat", normal_mat);
        static_body.m_mesh->render();
    }
}

void Scene::clear() {
    for (Mesh &mesh : m_meshes) {
        mesh.clear();
    }
    for (DynamicObject &object : m_dynamic_objects) {
        object.clear();
    }
}