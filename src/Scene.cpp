#include "Scene.hpp"

#include <variant>
#include <iostream>
#include <unordered_map>

void Scene::resetMesh(uint _i) {
    m_meshes[_i].clear();
    m_meshes[_i] = Mesh(m_meshes_type[_i]);
    m_meshes[_i].init();
}
void Scene::resetStaticBody(uint _i) {
    m_static_bodies_mesh_i[_i] = m_static_bodies_desc[_i].mesh_i;
    m_static_bodies_transfo[_i] = m_static_bodies_desc[_i].transfo;
}
void Scene::resetDynamicObject(uint _i) {
    m_dynamic_objects[_i].clear();
    m_dynamic_objects[_i] = DynamicObject::bodyFromMesh({&m_meshes[m_dynamic_objects_desc[_i].mesh_i], &m_dynamic_objects_desc[_i].transfo}, m_dynamic_objects_desc[_i].distance_stiffness, m_dynamic_objects_desc[_i].angle_stiffness, m_dynamic_objects_desc[_i].volume_stiffness, m_dynamic_objects_desc[_i].volume_pressure);
    for (uint pj : m_dynamic_objects_desc[_i].fixed_vertices) {
        m_dynamic_objects[_i].setVertexFixed(pj, true);
    }
    m_dynamic_objects[_i].setAmbientFrictionCoefficient(m_dynamic_objects_desc[_i].ambient_friction_coefficient);
    m_dynamic_objects[_i].setFrictionCoefficient(m_dynamic_objects_desc[_i].friction_coefficient);
    m_dynamic_objects[_i].setRestitutionCoefficient(m_dynamic_objects_desc[_i].restitution_coefficient);

    m_dynamic_objects[_i].initRendering();
}

void Scene::resetObjects() {
    m_meshes.resize(m_meshes_type.size());
    for (uint i = 0; i < m_meshes_type.size(); i++)
        resetMesh(i);

    m_static_bodies_mesh_i.resize(m_static_bodies_desc.size());
    m_static_bodies_transfo.resize(m_static_bodies_desc.size());
    for (uint i = 0; i < m_static_bodies_desc.size(); i++)
        resetStaticBody(i);

    m_dynamic_objects.resize(m_dynamic_objects_desc.size());
    m_dynamic_fixed_input_buffers.resize(m_dynamic_objects_desc.size());
    for (uint i = 0; i < m_dynamic_objects_desc.size(); i++)
        resetDynamicObject(i);

    // std::cout << "Initialisation terminée avec " << static_bodies.size() << " objets statiques et " << dynamic_objects.size() << " objets dynamiques." << std::endl;
}

void Scene::meshTypeInterface(uint _mesh_i) {
    ImGui::Text("%d:", _mesh_i);
    ImGui::SameLine();
    int _type_int = meshTypeToInt(m_meshes_type[_mesh_i]);
    if (ImGui::Combo(("type##mesh" + std::to_string(_mesh_i)).c_str(), &_type_int, IMGUI_MESH_TYPES) && _type_int != meshTypeToInt(m_meshes_type[_mesh_i])) {
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
                if (ImGui::InputText(("path##mesh" + std::to_string(_mesh_i)).c_str(), path_buffer, sizeof(path_buffer))) {
                    mesh_spec.path = path_buffer;
                }
            } else if constexpr (std::is_same_v<T, SingleTriangleMesh>) {
            } else if constexpr (std::is_same_v<T, SimpleGridMesh>) {
                glm::ivec2 dimensions(mesh_spec.nx, mesh_spec.nz);
                if (ImGui::DragInt2(("x,z##mesh" + std::to_string(_mesh_i)).c_str(), glm::value_ptr(dimensions), 0.01, 2, INT32_MAX)) {
                    mesh_spec.nx = std::max(2, dimensions.x);
                    mesh_spec.nz = std::max(2, dimensions.y);
                }
            } else if constexpr (std::is_same_v<T, SimpleTerrainMesh>) {
                glm::ivec2 dimensions(mesh_spec.nx, mesh_spec.nz);
                if (ImGui::DragInt2(("x,z##mesh" + std::to_string(_mesh_i)).c_str(), glm::value_ptr(dimensions), 0.01, 2, INT32_MAX)) {
                    mesh_spec.nx = std::max(2, dimensions.x);
                    mesh_spec.nz = std::max(2, dimensions.y);
                }
                ImGui::DragFloat2(("y range##mesh" + std::to_string(_mesh_i)).c_str(), glm::value_ptr(mesh_spec.y_range), 0.001, -FLT_MAX, FLT_MAX);
            } else if constexpr (std::is_same_v<T, CubeMesh>) {
                int details = mesh_spec.n;
                if (ImGui::DragInt(("details##mesh" + std::to_string(_mesh_i)).c_str(), &details, 0.01, 2, INT32_MAX)) {
                    mesh_spec.n = std::max(2, details);
                }
            } else if constexpr (std::is_same_v<T, CubeSphereMesh>) {
                int details = mesh_spec.n;
                if (ImGui::DragInt(("details##mesh" + std::to_string(_mesh_i)).c_str(), &details, 0.01, 3, INT32_MAX)) {
                    mesh_spec.n = std::max(3, details);
                }
            }
        },
        m_meshes_type[_mesh_i]);

    if (ImGui::Button(("apply##mesh" + std::to_string(_mesh_i)).c_str())) {
        resetMesh(_mesh_i);
        for (uint i = 0; i < m_static_bodies_desc.size(); i++)
            if (m_static_bodies_desc[i].mesh_i == _mesh_i)
                resetStaticBody(i);
        for (uint i = 0; i < m_dynamic_objects_desc.size(); i++)
            if (m_dynamic_objects_desc[i].mesh_i == _mesh_i)
                resetDynamicObject(i);
    }

    ImGui::SameLine();
    if (ImGui::Button(("remove##mesh" + std::to_string(_mesh_i)).c_str())) {
        m_meshes_type.erase(m_meshes_type.begin() + _mesh_i);
        m_meshes.erase(m_meshes.begin() + _mesh_i);

        for (uint i = 0; i < m_static_bodies_desc.size(); i++) {
            if (m_static_bodies_desc[i].mesh_i == _mesh_i) {
                m_static_bodies_desc.erase(m_static_bodies_desc.begin() + i);
                m_static_bodies_mesh_i.erase(m_static_bodies_mesh_i.begin() + i);
                m_static_bodies_transfo.erase(m_static_bodies_transfo.begin() + i);
            } else if (m_static_bodies_desc[i].mesh_i > _mesh_i) {
                m_static_bodies_desc[i].mesh_i--;
            }
        }

        for (uint i = 0; i < m_dynamic_objects_desc.size(); i++) {
            if (m_dynamic_objects_desc[i].mesh_i == _mesh_i) {
                m_dynamic_objects_desc.erase(m_dynamic_objects_desc.begin() + i);
                m_dynamic_objects.erase(m_dynamic_objects.begin() + i);
            } else if (m_dynamic_objects_desc[i].mesh_i > _mesh_i) {
                m_dynamic_objects_desc[i].mesh_i--;
            }
        }
    }

    ImGui::Unindent();
}

void Scene::staticBodyInterface(uint _i) {
    StaticBodyDesc &object = m_static_bodies_desc[_i];
    ImGui::Text("%s: ", object.name.c_str());

    ImGui::SameLine();
    if (ImGui::Button(("remove##static" + std::to_string(_i)).c_str())) {
        m_static_bodies_desc.erase(m_static_bodies_desc.begin() + _i);
        m_static_bodies_mesh_i.erase(m_static_bodies_mesh_i.begin() + _i);
        m_static_bodies_transfo.erase(m_static_bodies_transfo.begin() + _i);
    }

    ImGui::SameLine();
    ImGui::BeginDisabled(m_static_bodies_desc[_i].real_time);
    bool is_apply_pressed = ImGui::Button(("apply##static" + object.name).c_str());
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::Checkbox(("real time apply##static" + object.name).c_str(), &m_static_bodies_desc[_i].real_time);

    if (is_apply_pressed || m_static_bodies_desc[_i].real_time) {
        resetStaticBody(_i);
    }

    int mi = object.mesh_i;
    if (ImGui::InputInt(("mesh i##static" + object.name).c_str(), &mi, 1, 1)) {
        object.mesh_i = std::clamp(mi, 0, int(m_meshes.size() - 1));
    }

    ImGui::Spacing();
    ImGui::DragFloat3(("Position##static" + object.name).c_str(), glm::value_ptr(object.transfo.getTranslation()), .01f, -FLT_MAX, FLT_MAX);
    ImGui::DragFloat3(("Scale##static" + object.name).c_str(), glm::value_ptr(object.transfo.getScale()), .01f, -FLT_MAX, FLT_MAX);
    glm::vec3 ea_degree = glm::degrees(object.transfo.getEulerAngles());
    if (ImGui::DragFloat3(("Pitch Yaw Roll##static" + object.name).c_str(), glm::value_ptr(ea_degree), .5f, -360.f, 360.f)) {
        object.transfo.setEulerAngles(glm::radians(ea_degree));
        object.transfo.updateRotation();
    }
}

void Scene::dynamicObjectInterface(uint _i) {
    DynamicObjectDesc &object = m_dynamic_objects_desc[_i];
    ImGui::Text("%s: ", object.name.c_str());

    ImGui::SameLine();
    if (ImGui::Button(("remove##dynamic" + std::to_string(_i)).c_str())) {
        m_dynamic_objects_desc.erase(m_dynamic_objects_desc.begin() + _i);
        m_dynamic_objects.erase(m_dynamic_objects.begin() + _i);
    }

    ImGui::SameLine();
    if (ImGui::Button(("apply##dynamic" + object.name).c_str())) {
        resetDynamicObject(_i);
    }

    int mi = object.mesh_i;
    if (ImGui::InputInt(("mesh i##dynamic" + object.name).c_str(), &mi, 1, 1)) {
        object.mesh_i = std::clamp(mi, 0, int(m_meshes.size() - 1));
    }

    ImGui::Spacing();
    ImGui::DragFloat3(("Position##dynamic" + object.name).c_str(), glm::value_ptr(object.transfo.getTranslation()), .01f, -FLT_MAX, FLT_MAX);
    ImGui::DragFloat3(("Scale##dynamic" + object.name).c_str(), glm::value_ptr(object.transfo.getScale()), .01f, -FLT_MAX, FLT_MAX);
    glm::vec3 ea_degree = glm::degrees(object.transfo.getEulerAngles());
    if (ImGui::DragFloat3(("Pitch Yaw Roll##dynamic" + object.name).c_str(), glm::value_ptr(ea_degree), .5f, -360.f, 360.f)) {
        object.transfo.setEulerAngles(glm::radians(ea_degree));
        object.transfo.updateRotation();
    }

    ImGui::Spacing();
    ImGui::DragFloat(("Ambiant friction##dynamic" + object.name).c_str(), &object.ambient_friction_coefficient, .01f, 0.f, 1.f);
    ImGui::DragFloat(("Friction##dynamic" + object.name).c_str(), &object.friction_coefficient, .01f, 0.f, 1.f);
    ImGui::DragFloat(("Restitution##dynamic" + object.name).c_str(), &object.restitution_coefficient, .01f, 0.f, 1.f);

    ImGui::Spacing();
    ImGui::DragFloat(("distance stiffness##dynamic" + object.name).c_str(), &object.distance_stiffness, 0.001f, 0.f, 1.f);
    ImGui::DragFloat(("angle stiffness##dynamic" + object.name).c_str(), &object.angle_stiffness, 0.001f, 0.f, 1.f);
    ImGui::DragFloat(("volume stiffness##dynamic" + object.name).c_str(), &object.volume_stiffness, 0.001f, 0.f, 1.f);
    ImGui::DragFloat(("volume pressure##dynamic" + object.name).c_str(), &object.volume_pressure, 0.001f, 0.f, 1.f);

    ImGui::Spacing();
    int render_type_int = int(object.render_type);
    if (ImGui::Combo(("render type##dynamic" + object.name).c_str(), &render_type_int, IMGUI_DYNAMIC_RENDER_TYPES)) {
        object.render_type = DynamicRenderType(render_type_int);
    }

    ImGui::Spacing();
    ImGui::Text("Fixed Vertices: ");
    // Input field to add new fixed vertex
    if (ImGui::Button(("Add##fixed" + object.name).c_str())) {
        uint pj = static_cast<uint>(m_dynamic_fixed_input_buffers[_i]);
        object.fixed_vertices.insert(pj);
        m_dynamic_objects[_i].setVertexFixed(pj, true);
        m_dynamic_fixed_input_buffers[_i] = 0;
    }
    ImGui::SameLine();
    if (ImGui::InputInt(("Add vertex index##dynamic" + object.name).c_str(), &m_dynamic_fixed_input_buffers[_i], 1, 1)) {
        m_dynamic_fixed_input_buffers[_i] = std::max(0, m_dynamic_fixed_input_buffers[_i]);
    }

    // Display and allow removal of fixed vertices
    ImGui::Text("Fixed vertices:");
    ImGui::Indent();
    std::vector<uint> vertices_to_remove;
    for (uint pj : object.fixed_vertices) {
        ImGui::BulletText("%d", pj);
        ImGui::SameLine();
        if (ImGui::SmallButton(("remove##vertex" + std::to_string(pj) + "##" + object.name).c_str())) {
            vertices_to_remove.push_back(pj);
        }
    }
    for (uint pj : vertices_to_remove) {
        m_dynamic_objects[_i].setVertexFixed(pj, false);
        object.fixed_vertices.erase(pj);
    }
    ImGui::Unindent();

    ImGui::Spacing();
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
        int mesh_type_int = meshTypeToInt(m_new_mesh_type);
        if (ImGui::Combo("##add_mesh", &mesh_type_int, IMGUI_MESH_TYPES) && mesh_type_int != meshTypeToInt(m_new_mesh_type)) {
            m_new_mesh_type = meshTypeFromInt(mesh_type_int);
        }
        ImGui::SameLine();
        if (ImGui::Button("Add##mesh")) {
            m_meshes_type.push_back(m_new_mesh_type);
            m_meshes.push_back(Mesh());
            for (uint i = 0; i < m_meshes_type.size(); i++)
                resetMesh(i);
        }
        for (uint i = 0; i < m_meshes_type.size(); i++) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();
            meshTypeInterface(i);
        }

        ImGui::Spacing();
        ImGui::SeparatorText("Static Bodies");
        ImGui::Spacing();
        char new_static_name_buffer[256];
        std::strncpy(new_static_name_buffer, m_new_static_name.c_str(), sizeof(new_static_name_buffer) - 1);
        new_static_name_buffer[sizeof(new_static_name_buffer) - 1] = '\0';
        if (ImGui::InputText("##newstaticname", new_static_name_buffer, sizeof(new_static_name_buffer))) {
            m_new_static_name = new_static_name_buffer;
        }
        ImGui::SameLine();
        if (ImGui::Button("Add##static")) {
            m_static_bodies_desc.push_back(StaticBodyDesc(m_new_static_name, 0));
            m_static_bodies_mesh_i.push_back(0);
            m_static_bodies_transfo.push_back({});
            resetStaticBody(m_static_bodies_desc.size() - 1);
        }
        for (uint i = 0; i < m_static_bodies_desc.size(); i++) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();
            staticBodyInterface(i);
        }

        ImGui::Spacing();
        ImGui::SeparatorText("Dynamic Objects");
        ImGui::Spacing();
        char new_dynamic_name_buffer[256];
        std::strncpy(new_dynamic_name_buffer, m_new_dynamic_name.c_str(), sizeof(new_dynamic_name_buffer) - 1);
        new_dynamic_name_buffer[sizeof(new_dynamic_name_buffer) - 1] = '\0';
        if (ImGui::InputText("##newdynamicname", new_dynamic_name_buffer, sizeof(new_dynamic_name_buffer))) {
            m_new_dynamic_name = new_dynamic_name_buffer;
        }
        ImGui::SameLine();
        if (ImGui::Button("Add##dynamic")) {
            m_dynamic_objects_desc.push_back(DynamicObjectDesc(m_new_dynamic_name, 0, .9f, .9f, .9f, 1.f));
            m_dynamic_objects.push_back(DynamicObject());
            resetStaticBody(m_dynamic_objects_desc.size() - 1);
        }
        for (uint i = 0; i < m_dynamic_objects_desc.size(); i++) {
            if (i > 0) {
                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();
            }
            dynamicObjectInterface(i);
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
    std::vector<StaticBody> static_bodies(m_static_bodies_desc.size());
    for (uint i = 0; i < m_static_bodies_desc.size(); i++) {
        static_bodies[i].m_mesh = &m_meshes[m_static_bodies_mesh_i[i]];
        static_bodies[i].m_transformation = &m_static_bodies_transfo[i];
    }

    for (DynamicObject &obj : m_dynamic_objects) {
        if (!obj.update(do_fixed_delta_time ? fixed_delta_time : _deltaTime, solver_iterations, static_bodies)) {
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
    for (uint i = 0; i < m_dynamic_objects_desc.size(); i++) {
        m_dynamic_objects[i].render(m_dynamic_objects_desc[i].render_type);
    }

    // STATIC OBJECTS RENDERING
    _mesh_shader.use();
    _mesh_shader.set("projection", _projection);
    for (uint i = 0; i < m_static_bodies_desc.size(); i++) {
        glm::mat4 model = m_static_bodies_transfo[i].computeTransformationMatrix();
        glm::mat4 model_view = _view * model;
        glm::mat4 normal_mat = glm::transpose(glm::inverse(model_view));
        _mesh_shader.set("model_view", model_view);
        _mesh_shader.set("normal_mat", normal_mat);
        m_meshes[m_static_bodies_mesh_i[i]].render();
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