#include "Scene.hpp"

#include <iostream>

void Scene::resetObjects() {
    m_static_bodies = std::vector<StaticBodyDesc>(m_pre_static_bodies);
    m_dynamic_objects = std::vector<DynamicObjectDesc>(m_pre_dynamic_objects);
    for (DynamicObjectDesc &obj : m_dynamic_objects) {
        obj.object.updateRenderedPositions();
        // obj.object.updateRenderedConstraints();
    }
    // std::cout << "Initialisation terminée avec " << static_bodies.size() << " objets statiques et " << dynamic_objects.size() << " objets dynamiques." << std::endl;
}

bool Scene::updateInterface() {
    float disable_mouse_actions = false;
    if (ImGui::Begin("Scene")) {
        disable_mouse_actions = ImGui::IsWindowHovered() || ImGui::IsAnyItemHovered() || ImGui::IsAnyItemActive() || ImGui::IsAnyItemFocused();
        ImGui::SeparatorText("Static Bodies");
        ImGui::Spacing();
        for (StaticBodyDesc &obj : m_pre_static_bodies) {
            ImGui::Text("%s", obj.name.c_str());
            ImGui::DragFloat3("Position", glm::value_ptr(obj.transfo.getTranslation()), .01f, -10.f, 10.f);
        }

        ImGui::Spacing();
        ImGui::SeparatorText("Dynamic Objects");
        for (DynamicObjectDesc &obj : m_pre_dynamic_objects) {
            ImGui::Text("%s", obj.name.c_str());

            glm::vec3 frictions_truc(obj.object.getAmbientFrictionCoefficient(), obj.object.getFrictionCoefficient(), obj.object.getRestitutionCoefficient());
            if (ImGui::DragFloat("Ambiant friction", &frictions_truc[0], .01f, 0.f, 1.f) ||
                ImGui::DragFloat("Friction", &frictions_truc[1], .01f, 0.f, 1.f) ||
                ImGui::DragFloat("Restitution", &frictions_truc[2], .01f, 0.f, 1.f)) {
                obj.object.setAmbientFrictionCoefficient(frictions_truc[0]);
                obj.object.setFrictionCoefficient(frictions_truc[1]);
                obj.object.setRestitutionCoefficient(frictions_truc[2]);
            }
        }

        if (ImGui::Button("Reset Objects")) {
            resetObjects();
        }
    }
    ImGui::End();
    return disable_mouse_actions;
}

bool Scene::updateInteractions(GLFWwindow *_window, const glm::dvec3 &_camera_pos, const glm::dvec3 &_cursor_worldpos) {
    bool res = false;
    for (DynamicObjectDesc &object_desc : m_dynamic_objects) {
        if (object_desc.object.updateInteractions(_window, _camera_pos, _cursor_worldpos)) {
            object_desc.object.updateRenderedPositions();
            res = true;
        }
    }
    return res;
}

bool Scene::updateSimulation(float _deltaTime) {
    std::vector<StaticBody> static_bodies(m_static_bodies.size());
    for (uint i = 0; i < m_static_bodies.size(); i++) {
        static_bodies[i].m_mesh = &m_meshes[m_static_bodies[i].mesh_i];
        static_bodies[i].m_transformation = &m_static_bodies[i].transfo;
    }

    for (DynamicObjectDesc &obj : m_dynamic_objects) {
        if (!obj.object.update(_deltaTime, static_bodies)) {
            obj.object.updateRenderedPositions();
            return false;
        }
        obj.object.updateRenderedPositions();
    }

    return true;
}

void Scene::init() {
    for (Mesh &mesh : m_meshes) {
        mesh.init();
    }
    for (DynamicObjectDesc &object_desc : m_pre_dynamic_objects) {
        object_desc.object.initRendering();
    }
}

void Scene::render(const ShaderProgram &_dynamic_shader, const ShaderProgram &_mesh_shader, const glm::mat4 &_projection, const glm::mat4 &_view) const {
    // DYNAMIC OBJECTS RENDERING
    _dynamic_shader.use();
    _dynamic_shader.set("projection", _projection);
    _dynamic_shader.set("view", _view);
    for (const DynamicObjectDesc &object_desc : m_dynamic_objects) {
        object_desc.object.render();
    }

    // STATIC OBJECTS RENDERING
    _mesh_shader.use();
    _mesh_shader.set("projection", _projection);
    for (const StaticBodyDesc &static_body : m_static_bodies) {
        glm::mat4 model = static_body.transfo.computeTransformationMatrix();
        glm::mat4 model_view = _view * model;
        glm::mat4 normal_mat = glm::transpose(glm::inverse(model_view));
        _mesh_shader.set("model_view", model_view);
        _mesh_shader.set("normal_mat", normal_mat);
        m_meshes[static_body.mesh_i].render();
    }
}

void Scene::clear() {
    for (Mesh &mesh : m_meshes) {
        mesh.clear();
    }
    for (DynamicObjectDesc &object_desc : m_pre_dynamic_objects) {
        object_desc.object.clear();
    }
}